# imports
import servo_driver_imp
import rgb_led_imp
import gyro_imp
import time
import threading

# define servo indexes
CAM_X = 0
CAM_Y = 1

FL_MOTOR_1 = 2  # front left shoulder
FL_MOTOR_2 = 3  # front left upper leg
FL_MOTOR_3 = 4  # front left lower leg

FR_MOTOR_1 = 5  # front right shoulder
FR_MOTOR_2 = 6  # front right upper leg
FR_MOTOR_3 = 7  # front right lower leg

BL_MOTOR_1 = 8   # back left shoulder
BL_MOTOR_2 = 9   # back left upper leg
BL_MOTOR_3 = 10  # back left lower leg

BR_MOTOR_1 = 11  # back right shoulder
BR_MOTOR_2 = 12  # back right upper leg
BR_MOTOR_3 = 13  # back right lower leg

# define stand angles - first calibration
STAND_ANGLES = {
    CAM_X:      90,  #0
    CAM_Y:      90,  #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 50,  #3
    FL_MOTOR_3: 90,  #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 130, #6
    FR_MOTOR_3: 90, #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 50,  #9
    BL_MOTOR_3: 90,  #10
    BR_MOTOR_1: 80,  #11
    BR_MOTOR_2: 130, #12
    BR_MOTOR_3: 90,  #13
}

# define stand angles - second calibration, better stability
BETTER_STAND_ANGLES = {
    CAM_X:      90,  #0
    CAM_Y:      90,  #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 70,  #3
    FL_MOTOR_3: 50,  #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 110, #6
    FR_MOTOR_3: 150, #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 70,  #9
    BL_MOTOR_3: 40,  #10
    BR_MOTOR_1: 90,  #11
    BR_MOTOR_2: 90,  #12
    BR_MOTOR_3: 160, #13
}

# define resting angles
resting_angles = {
    CAM_X:      90,  #0
    CAM_Y:      90,  #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 180, #3 - left upper leg rests at 180
    FL_MOTOR_3: 180, #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 0,   #6 - right upper leg rests at 0
    FR_MOTOR_3: 0,   #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 180, #9 - left upper leg rests at 180
    BL_MOTOR_3: 180, #10
    BR_MOTOR_1: 110, #11
    BR_MOTOR_2: 0,   #12 - right upper leg rests at 0
    BR_MOTOR_3: 0,   #13
}

# set to True to use the better standing angles instead of the default ones
better_standing = True

# gyro offset calibration
# these values will be replaced automatically when the robot stands
GYRO_PITCH_OFFSET = 3.57
GYRO_ROLL_OFFSET = 5.39

# number of samples used for automatic gyro calibration
GYRO_CALIBRATION_SAMPLES = 30
GYRO_CALIBRATION_INTERVAL = 0.1

# define leveling vars
level_gain = 1.2 # how much correction per degree of tilt
level_deadband = 1.0 # ignore tilt smaller than this in degrees
level_max_correction_shoulders = 15.0 # max degrees shoulders can move from stand angle
level_max_correction_legs = 20.0 # max degrees upper legs can move from stand angle
level_interval = 0.15 # seconds between each gyro read and correction

roll_gain = level_gain # if corrections make roll worse flip this to negative
pitch_gain = level_gain * 1.5 # higher than roll since robot was falling forward, flip to negative if needed

leg_roll_gain = level_gain * 0.6 # leg correction is subtler than shoulder, raise if not enough
leg_pitch_gain = level_gain * 0.6 # same as above for pitch axis

full_leveling = False # False = shoulders only, True = shoulders + upper legs

# flag to control the leveling thread
leveling_active = False

# fn to get the active stand angles based on better_standing flag
def get_active_stand_angles():
    if better_standing:
        return BETTER_STAND_ANGLES
    return STAND_ANGLES

# fn to init all servo ranges
def init_servo_ranges():
    print("Initialising servo ranges...")
    for index in get_active_stand_angles().keys():
        servo_driver_imp.set_servo_range(index, 180)
        time.sleep(0.05)
    print("All servo ranges set.")

# helper fn to clamp a value between a min and max
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to automatically calibrate the gyro based on the current standing position
def calibrate_level_offset():
    global GYRO_PITCH_OFFSET
    global GYRO_ROLL_OFFSET

    print("Calibrating gyro level position...")
    print("Keep the robot completely still.")

    pitch_total = 0.0
    roll_total = 0.0
    valid_samples = 0

    for i in range(GYRO_CALIBRATION_SAMPLES):
        data = gyro_imp.get_pitch_roll()

        if data is not None:
            pitch_total += data["pitch"]
            roll_total += data["roll"]
            valid_samples += 1

            print(
                "Calibration "
                + str(i + 1)
                + "/"
                + str(GYRO_CALIBRATION_SAMPLES)
                + " > pitch: "
                + str(round(data["pitch"], 2))
                + ", roll: "
                + str(round(data["roll"], 2))
            )
        else:
            print("Could not read gyro during calibration.")

        time.sleep(GYRO_CALIBRATION_INTERVAL)

    if valid_samples == 0:
        print("Gyro calibration failed - no valid readings.")
        return False

    GYRO_PITCH_OFFSET = pitch_total / valid_samples
    GYRO_ROLL_OFFSET = roll_total / valid_samples

    print("Gyro calibration complete.")
    print("Pitch offset: " + str(round(GYRO_PITCH_OFFSET, 3)))
    print("Roll offset: " + str(round(GYRO_ROLL_OFFSET, 3)))

    return True

# fn to apply shoulder-only correction
def apply_shoulder_correction(roll_corr, pitch_corr):
    angles = get_active_stand_angles()

    fl_new_angle = clamp_value(
        angles[FL_MOTOR_1] - roll_corr + pitch_corr,
        angles[FL_MOTOR_1] - level_max_correction_shoulders,
        angles[FL_MOTOR_1] + level_max_correction_shoulders
    )

    fr_new_angle = clamp_value(
        angles[FR_MOTOR_1] + roll_corr + pitch_corr,
        angles[FR_MOTOR_1] - level_max_correction_shoulders,
        angles[FR_MOTOR_1] + level_max_correction_shoulders
    )

    bl_new_angle = clamp_value(
        angles[BL_MOTOR_1] - roll_corr - pitch_corr,
        angles[BL_MOTOR_1] - level_max_correction_shoulders,
        angles[BL_MOTOR_1] + level_max_correction_shoulders
    )

    br_new_angle = clamp_value(
        angles[BR_MOTOR_1] + roll_corr - pitch_corr,
        angles[BR_MOTOR_1] - level_max_correction_shoulders,
        angles[BR_MOTOR_1] + level_max_correction_shoulders
    )

    print("Shoulder correction > fl : " + str(fl_new_angle) + ", fr : " + str(fr_new_angle) + ", bl : " + str(bl_new_angle) + ", br : " + str(br_new_angle))

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, int(fl_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, int(fr_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, int(bl_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, int(br_new_angle))

# fn to apply upper leg correction on top of shoulder correction
# left side upper legs (3, 9) are inverse - lower angle means more extended
# right side upper legs (6, 12) are normal - higher angle means more extended
def apply_leg_correction(roll_corr, pitch_corr):
    angles = get_active_stand_angles()

    fl_leg_angle = clamp_value(
        angles[FL_MOTOR_2] - roll_corr + pitch_corr,
        angles[FL_MOTOR_2] - level_max_correction_legs,
        angles[FL_MOTOR_2] + level_max_correction_legs
    )

    bl_leg_angle = clamp_value(
        angles[BL_MOTOR_2] - roll_corr - pitch_corr,
        angles[BL_MOTOR_2] - level_max_correction_legs,
        angles[BL_MOTOR_2] + level_max_correction_legs
    )

    fr_leg_angle = clamp_value(
        angles[FR_MOTOR_2] + roll_corr + pitch_corr,
        angles[FR_MOTOR_2] - level_max_correction_legs,
        angles[FR_MOTOR_2] + level_max_correction_legs
    )

    br_leg_angle = clamp_value(
        angles[BR_MOTOR_2] + roll_corr - pitch_corr,
        angles[BR_MOTOR_2] - level_max_correction_legs,
        angles[BR_MOTOR_2] + level_max_correction_legs
    )

    print("Leg correction > fl : " + str(fl_leg_angle) + ", fr : " + str(fr_leg_angle) + ", bl : " + str(bl_leg_angle) + ", br : " + str(br_leg_angle))

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, int(fl_leg_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, int(fr_leg_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, int(bl_leg_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, int(br_leg_angle))

# fn to read gyro and apply correction based on full_leveling setting
def apply_level_correction():
    print("Reading gyro for level correction...")

    data = gyro_imp.get_pitch_roll()

    if data is None:
        print("Could not read gyro, skipping correction...")
        return

    # subtract offset so readings are relative to the current natural standing position
    pitch = data["pitch"] - GYRO_PITCH_OFFSET
    roll = data["roll"] - GYRO_ROLL_OFFSET

    print("Gyro pitch : " + str(pitch) + ", roll : " + str(roll) + " (offset applied)")

    # apply deadband, ignore small tilts
    effective_roll = roll if abs(roll) > level_deadband else 0.0
    effective_pitch = pitch if abs(pitch) > level_deadband else 0.0

    if effective_roll == 0.0 and effective_pitch == 0.0:
        print("Within deadband, no correction needed.")
        return

    # calculate and apply shoulder correction
    shoulder_roll_corr = roll_gain * effective_roll
    shoulder_pitch_corr = pitch_gain * effective_pitch
    apply_shoulder_correction(shoulder_roll_corr, shoulder_pitch_corr)

    # also apply leg correction if full leveling is enabled
    if full_leveling:
        leg_roll_corr = leg_roll_gain * effective_roll
        leg_pitch_corr = leg_pitch_gain * effective_pitch
        apply_leg_correction(leg_roll_corr, leg_pitch_corr)

    print("Correction applied.")

# fn that runs in a background thread and keeps leveling the robot continuously
def leveling_thread_fn():
    print("Leveling thread started...")
    while leveling_active:
        apply_level_correction()
        time.sleep(level_interval)
    print("Leveling thread stopped.")

# fn to start the leveling thread
def start_leveling():
    global leveling_active

    leveling_active = True

    t = threading.Thread(target=leveling_thread_fn, daemon=True)
    t.start()

    if full_leveling:
        print("Continuous full leveling started (shoulders + legs).")
    else:
        print("Continuous shoulder leveling started.")

    rgb_led_imp.blink_warning()
    return t

# fn to stop the leveling thread
def stop_leveling():
    global leveling_active
    leveling_active = False
    print("Stopping leveling thread...")

# fn to set the sholders
def set_sholders():
    print("Running set sholders fn...")
    angles = get_active_stand_angles()

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, angles[FL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, angles[FR_MOTOR_1])

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, angles[BL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, angles[BR_MOTOR_1])

    print("Finished setting sholders.")
    rgb_led_imp.blink_oke()

# fn to set main leg to neutral
def set_main_legs():
    print("Running setting main legs..")
    angles = get_active_stand_angles()

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, angles[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, angles[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, angles[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, angles[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to set lower leg to neutral
def set_lower_legs():
    print("Running setting lower legs..")
    angles = get_active_stand_angles()

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3, angles[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3, angles[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3, angles[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3, angles[BR_MOTOR_3])

    print("Finished setting lower legs.")
    rgb_led_imp.blink_oke()

# fn to rest the middle legs
def rest_main_legs():
    print("Running rest main legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, resting_angles[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, resting_angles[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, resting_angles[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, resting_angles[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to rest lower leg
def resting_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3, resting_angles[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3, resting_angles[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3, resting_angles[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3, resting_angles[BR_MOTOR_3])

    print("Finished resting lower legs.")
    rgb_led_imp.blink_oke()

# loop to not end script
run_loop = True
level_thread = None

# show script started indication
rgb_led_imp.move_pixel_from_left_to_right(105,0,105)

try:

    # init servo ranges first before any movement
    init_servo_ranges()
    time.sleep(0.5)

    while run_loop:

        print("[=======[New cycle]=========]")
        if better_standing:
            print("Using better standing angles.")
        else:
            print("Using default standing angles.")

        # stand up
        set_sholders()
        set_main_legs()
        set_lower_legs()

        # wait for the robot to settle before calibrating the gyro
        print("Robot standing, waiting for it to settle...")
        time.sleep(1.0)

        # calibrate the gyro for the current standing position
        calibrate_level_offset()

        # start continuous leveling in background after standing and calibration
        level_thread = start_leveling()

        rgb_led_imp.animate_running_process()

        user_end_script = input("End script? y/n > ")
        print(user_end_script)

        if user_end_script == "y":
            print("Ending script...")
            run_loop = False

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Script ended - stopping")

    # stop leveling thread before moving legs
    stop_leveling()
    time.sleep(0.5)

    resting_lower_legs()
    rest_main_legs()

    print("Legs resting now release the motors...")
    time.sleep(1)
    servo_driver_imp.cleanup_servos()
    time.sleep(1)
    rgb_led_imp.turn_off_strip()
    print("Finished")
