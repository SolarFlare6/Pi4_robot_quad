# imports
import servo_driver_imp
import rgb_led_imp
import gyro_imp
import kinematics_imp
import time
import threading

# define servo indexes
FL_MOTOR_1 = 2
FL_MOTOR_2 = 3
FL_MOTOR_3 = 4
FR_MOTOR_1 = 5
FR_MOTOR_2 = 6
FR_MOTOR_3 = 7
BL_MOTOR_1 = 8
BL_MOTOR_2 = 9
BL_MOTOR_3 = 10
BR_MOTOR_1 = 11
BR_MOTOR_2 = 12
BR_MOTOR_3 = 13

# resting angles
resting_angles = {
    FL_MOTOR_1: 110,
    FL_MOTOR_2: 180,
    FL_MOTOR_3: 180,
    FR_MOTOR_1: 110,
    FR_MOTOR_2: 0,
    FR_MOTOR_3: 0,
    BL_MOTOR_1: 130,
    BL_MOTOR_2: 180,
    BL_MOTOR_3: 180,
    BR_MOTOR_1: 110,
    BR_MOTOR_2: 0,
    BR_MOTOR_3: 0,
}

# gyro offset calibration
GYRO_PITCH_OFFSET = 3.57
GYRO_ROLL_OFFSET = 5.39

# leveling vars
level_gain = 0.5
level_deadband = 1.5
level_max_corr = 8.0
level_interval = 0.15

roll_gain = level_gain
pitch_gain = level_gain * 1.5

# flag to control leveling thread
leveling_active = False

# fn to init all servo ranges
def init_servo_ranges():
    print("Initialising servo ranges...")

    for index in range(14):
        servo_driver_imp.set_servo_range(index, 180)
        time.sleep(0.05)

    print("All servo ranges set.")

# helper fn to clamp a value
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to apply one round of shoulder correction
def apply_level_correction():
    print("Reading gyro for level correction...")

    data = gyro_imp.get_pitch_roll()

    if data is None:
        print("Could not read gyro, skipping correction...")
        return

    pitch = data["pitch"] - GYRO_PITCH_OFFSET
    roll = data["roll"] - GYRO_ROLL_OFFSET

    print("Gyro pitch : " + str(pitch) + ", roll : " + str(roll) + " (offset applied)")

    effective_roll = roll if abs(roll) > level_deadband else 0.0
    effective_pitch = pitch if abs(pitch) > level_deadband else 0.0

    if effective_roll == 0.0 and effective_pitch == 0.0:
        print("Within deadband, no correction needed.")
        return

    # get standing shoulder angles directly from IK
    stand = kinematics_imp.get_stand_servo_angles()

    if stand is None:
        print("Could not calculate IK stand angles, skipping correction...")
        return

    roll_corr = roll_gain * effective_roll
    pitch_corr = pitch_gain * effective_pitch

    fl_new = clamp_value(
        stand[FL_MOTOR_1] - roll_corr + pitch_corr,
        stand[FL_MOTOR_1] - level_max_corr,
        stand[FL_MOTOR_1] + level_max_corr
    )

    fr_new = clamp_value(
        stand[FR_MOTOR_1] + roll_corr + pitch_corr,
        stand[FR_MOTOR_1] - level_max_corr,
        stand[FR_MOTOR_1] + level_max_corr
    )

    bl_new = clamp_value(
        stand[BL_MOTOR_1] - roll_corr - pitch_corr,
        stand[BL_MOTOR_1] - level_max_corr,
        stand[BL_MOTOR_1] + level_max_corr
    )

    br_new = clamp_value(
        stand[BR_MOTOR_1] + roll_corr - pitch_corr,
        stand[BR_MOTOR_1] - level_max_corr,
        stand[BR_MOTOR_1] + level_max_corr
    )

    print("Shoulder correction > fl : " + str(fl_new) + ", fr : " + str(fr_new) + ", bl : " + str(bl_new) + ", br : " + str(br_new))

    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, int(fl_new))
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, int(fr_new))
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, int(bl_new))
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, int(br_new))

    print("Correction applied.")

# fn that runs in background
def leveling_thread_fn():
    print("Leveling thread started...")

    while leveling_active:
        apply_level_correction()
        time.sleep(level_interval)

    print("Leveling thread stopped.")

# fn to start leveling
def start_leveling():
    global leveling_active

    leveling_active = True

    t = threading.Thread(target=leveling_thread_fn, daemon=True)
    t.start()

    print("Continuous leveling started.")

    rgb_led_imp.blink_warning()

    return t

# fn to stop leveling
def stop_leveling():
    global leveling_active

    leveling_active = False

    print("Stopping leveling thread...")

# fn to stand up using ik
def stand_up():
    print("Standing up using IK...")

    kinematics_imp.ik_stand()

    print("Stand up finished.")

    rgb_led_imp.blink_oke()

# fn to rest the middle legs
def rest_main_legs():
    print("Running rest main legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_2,
        resting_angles[FL_MOTOR_2]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_2,
        resting_angles[FR_MOTOR_2]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_2,
        resting_angles[BL_MOTOR_2]
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_2,
        resting_angles[BR_MOTOR_2]
    )

    print("Finished.")

    rgb_led_imp.blink_oke()

# fn to rest the lower legs
def resting_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_3,
        resting_angles[FL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_3,
        resting_angles[FR_MOTOR_3]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_3,
        resting_angles[BL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_3,
        resting_angles[BR_MOTOR_3]
    )

    print("Finished resting lower legs.")

    rgb_led_imp.blink_oke()

# loop to not end script
run_loop = True
level_thread = None

try:
    # init servo ranges before any movement
    init_servo_ranges()
    time.sleep(0.5)

    # verify ik calibration before moving
    kinematics_imp.verify_ik_calibration()
    time.sleep(0.5)

    while run_loop:
        print("[=======[New cycle]=========]")

        # stand using ik
        stand_up()

        # start continuous gyro leveling
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