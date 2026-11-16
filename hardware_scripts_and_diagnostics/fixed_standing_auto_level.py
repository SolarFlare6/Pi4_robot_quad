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

# define stand angles - manually calibrated from servo control script
STAND_ANGLES = {
    CAM_X:      90,  #0
    CAM_Y:      90,  #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 50,  #3
    FL_MOTOR_3: 90,  #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 130, #6
    FR_MOTOR_3: 90,  #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 50,  #9
    BL_MOTOR_3: 90,  #10
    BR_MOTOR_1: 80,  #11
    BR_MOTOR_2: 130, #12
    BR_MOTOR_3: 90,  #13
}

# define resting angles
# left side upper legs rest at 180, right side upper legs rest at 0
# shoulders stay in place, lower legs stay at 90
resting_angles = {
    CAM_X:      90,  #0
    CAM_Y:      90,  #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 180, #3 - left upper leg rests at 180
    FL_MOTOR_3: 180,  #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 0,   #6 - right upper leg rests at 0
    FR_MOTOR_3: 0,  #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 180, #9 - left upper leg rests at 180
    BL_MOTOR_3: 180,  #10
    BR_MOTOR_1: 110,  #11
    BR_MOTOR_2: 0,   #12 - right upper leg rests at 0
    BR_MOTOR_3: 0,  #13
}

# define leveling vars
level_gain = 1.2 # gain : how much correction per degree of tilt
level_deadband = 1.0 # deadband : ignore tilt smaller than this in degrees
level_max_correction = 15.0 # max correction : max degrees the shoulders can deviate from stand angle
level_interval = 0.15 # interval : seconds between each gyro read and correction 

# pitch gain higher since robot was falling forward
# if corrections make lean worse flip the sign of the relevant gain to negative
roll_gain = level_gain
pitch_gain = level_gain * 1.5

# flag to control the leveling thread
leveling_active = False

# fn to init all servo ranges
def init_servo_ranges():
    print("Initialising servo ranges...")
    for index in STAND_ANGLES.keys():
        servo_driver_imp.set_servo_range(index, 180)
        time.sleep(0.05)
    print("All servo ranges set.")

# helper fn to clamp a value between a min and max
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to apply one round of gyro correction to the shoulder motors
# roll = left/right tilt, pitch = front/back tilt
# only shoulders are adjusted, legs stay locked in stand position
def apply_level_correction():
    print("Reading gyro for level correction...")

    data = gyro_imp.get_pitch_roll()

    if data is None:
        print("Could not read gyro, skipping correction...")
        return

    pitch = data["pitch"]
    roll = data["roll"]

    print("Gyro pitch : " + str(pitch) + ", roll : " + str(roll))

    # apply deadband, ignore small tilts
    effective_roll = roll if abs(roll) > level_deadband else 0.0
    effective_pitch = pitch if abs(pitch) > level_deadband else 0.0

    if effective_roll == 0.0 and effective_pitch == 0.0:
        print("Within deadband, no correction needed.")
        return

    # calculate correction amounts
    roll_corr = roll_gain * effective_roll
    pitch_corr = pitch_gain * effective_pitch

    # calculate new shoulder angles clamped to safe range
    fl_new_angle = clamp_value(
        STAND_ANGLES[FL_MOTOR_1] - roll_corr + pitch_corr,
        STAND_ANGLES[FL_MOTOR_1] - level_max_correction,
        STAND_ANGLES[FL_MOTOR_1] + level_max_correction
    )
    fr_new_angle = clamp_value(
        STAND_ANGLES[FR_MOTOR_1] + roll_corr + pitch_corr,
        STAND_ANGLES[FR_MOTOR_1] - level_max_correction,
        STAND_ANGLES[FR_MOTOR_1] + level_max_correction
    )
    bl_new_angle = clamp_value(
        STAND_ANGLES[BL_MOTOR_1] - roll_corr - pitch_corr,
        STAND_ANGLES[BL_MOTOR_1] - level_max_correction,
        STAND_ANGLES[BL_MOTOR_1] + level_max_correction
    )
    br_new_angle = clamp_value(
        STAND_ANGLES[BR_MOTOR_1] + roll_corr - pitch_corr,
        STAND_ANGLES[BR_MOTOR_1] - level_max_correction,
        STAND_ANGLES[BR_MOTOR_1] + level_max_correction
    )

    print("Applying correction > fl : " + str(fl_new_angle) + ", fr : " + str(fr_new_angle) + ", bl : " + str(bl_new_angle) + ", br : " + str(br_new_angle))

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, int(fl_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, int(fr_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, int(bl_new_angle))
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, int(br_new_angle))

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
    print("Continuous leveling started.")
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

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, STAND_ANGLES[FL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, STAND_ANGLES[FR_MOTOR_1])

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, STAND_ANGLES[BL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, STAND_ANGLES[BR_MOTOR_1])

    print("Finished setting sholders.")
    rgb_led_imp.blink_oke()

# fn to set main leg to neutral
def set_main_legs():
    print("Running setting main legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, STAND_ANGLES[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, STAND_ANGLES[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, STAND_ANGLES[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, STAND_ANGLES[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to set lower leg to neutral
def set_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3, STAND_ANGLES[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3, STAND_ANGLES[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3, STAND_ANGLES[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3, STAND_ANGLES[BR_MOTOR_3])

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

try:

    # init servo ranges first before any movement
    init_servo_ranges()
    time.sleep(0.5)

    while run_loop:

        print("[=======[New cycle]=========]")

        # stand up
        set_sholders()
        set_main_legs()
        set_lower_legs()

        # start continuous leveling in background after standing
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
