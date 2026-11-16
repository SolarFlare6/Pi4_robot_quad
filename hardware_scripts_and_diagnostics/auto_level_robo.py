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

# define stand angles
STAND_ANGLES = {
    CAM_X:      90, #0
    CAM_Y:      90, #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 90,  #3
    FL_MOTOR_3: 0,   #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 90,  #6
    FR_MOTOR_3: 180, #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 90,  #9
    BL_MOTOR_3: 0,   #10
    BR_MOTOR_1: 110, #11
    BR_MOTOR_2: 90,  #12
    BR_MOTOR_3: 180, #13
}

# define resting angles
resting_angles = {
    CAM_X:      90, #0
    CAM_Y:      90, #1
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 180, #3
    FL_MOTOR_3: 180, #4
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 0,   #6
    FR_MOTOR_3: 0,   #7
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 180, #9
    BL_MOTOR_3: 180, #10
    BR_MOTOR_1: 110, #11
    BR_MOTOR_2: 0,   #12
    BR_MOTOR_3: 0,   #13
}

# define leveling vars
# gain : how much correction per degree of tilt, start at 0.5 and raise if under-correcting
# deadband : ignore tilt smaller than this in degrees, raise if jittering on flat ground
# max correction : max degrees the shoulders can move from their stand angle
# level interval : seconds between each correction read while the robot is standing
level_gain = 0.5
level_deadband = 2.0
level_max_correction = 20.0
level_interval = 0.3

# roll gain and pitch gain can be set to negative to flip correction direction
# if a correction makes the lean worse flip the sign here
roll_gain = level_gain
pitch_gain = level_gain

# flag to control the leveling thread
# set to True while robot is standing, False when sitting down
leveling_active = False

# helper fn to clamp a value between a min and max
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to apply one round of gyro correction to the shoulder motors
# roll = left/right tilt, pitch = front/back tilt
# if corrections make things worse flip roll_gain or pitch_gain to negative above
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

    # calculate new shoulder angles with correction applied and clamped to safe range
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
# while leveling_active is True it reads the gyro and corrects every level_interval seconds
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

    # set front sholders
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1, STAND_ANGLES[FL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1, STAND_ANGLES[FR_MOTOR_1])

    time.sleep(0.5)

    # set back sholders
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1, STAND_ANGLES[BL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1, STAND_ANGLES[BR_MOTOR_1])

    print("Finished setting sholders.")
    rgb_led_imp.blink_oke()

# fn to set main leg to neutral
def set_main_legs():
    print("Running setting main legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, STAND_ANGLES[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, STAND_ANGLES[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, STAND_ANGLES[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, STAND_ANGLES[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to set lower leg to neutral
def set_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3, STAND_ANGLES[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3, STAND_ANGLES[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3, STAND_ANGLES[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3, STAND_ANGLES[BR_MOTOR_3])

    print("Finished setting lower legs.")
    rgb_led_imp.blink_oke()

# fn to rest the middle legs
def rest_main_legs():
    print("Running rest main legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2, resting_angles[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2, resting_angles[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2, resting_angles[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2, resting_angles[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to rest lower leg
def resting_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3, resting_angles[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3, resting_angles[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3, resting_angles[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3, resting_angles[BR_MOTOR_3])

    print("Finished resting lower legs.")
    rgb_led_imp.blink_oke()


# install with : pip install cactus-needle
# when needle is installed enable this var to use needle model
use_needle = False

def setup_needle():
    print("Setting up needle 2 ai controller...")

    try:
        import needle

        # define the robot functions as tools so needle knows what it can call
        @needle.tool
        def stand_up():
            "Make the robot dog stand up from a resting position."
            set_sholders()
            set_main_legs()
            set_lower_legs()
            start_leveling()
            return {"status": "standing"}

        @needle.tool
        def sit_down():
            "Make the robot dog sit down and go to resting position."
            stop_leveling()
            time.sleep(0.5)
            resting_lower_legs()
            rest_main_legs()
            return {"status": "resting"}

        @needle.tool
        def get_tilt():
            "Get the current pitch and roll tilt of the robot from the gyroscope."
            data = gyro_imp.get_pitch_roll()
            return data

        # init the needle agent with the robot tools
        agent = needle.Needle(tools=[stand_up, sit_down, get_tilt])
        print("Needle 2 ready.")
        return agent

    except Exception as e:
        print("Could not load needle 2 : " + str(e))
        print("Make sure it is installed with : pip install cactus-needle")
        return None

# needle end

# loop to not end script
run_loop = True
level_thread = None

try:
    while run_loop:

        print("[=======[New cycle]=========]")

        # check if needle is enabled and run in ai mode
        if use_needle:
            needle_agent = setup_needle()
            if needle_agent is not None:
                print("Needle ai mode active - type a command or end to stop")
                user_input = input("Command > ")
                print(user_input)
                if user_input == "end":
                    run_loop = False
                else:
                    result = needle_agent.run(user_input)
                    print("Needle result : " + str(result["results"]))
            else:
                print("Needle failed to load, falling back to manual mode...")
                use_needle = False

        # manual mode - stand up and hold until user ends
        else:
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
