# imports
import servo_driver_imp
import rgb_led_imp
import time

# Servo indexs
CAM_X = 0
CAM_Y = 1

FL_MOTOR_1  = 2   # front-left  hip/shoulder
FL_MOTOR_2  = 3   # front-left  upper leg
FL_MOTOR_3  = 4   # front-left  lower leg

FR_MOTOR_1  = 5   # front-right hip/shoulder
FR_MOTOR_2  = 6   # front-right upper leg
FR_MOTOR_3  = 7   # front-right lower leg

BL_MOTOR_1  = 8   # back-left   hip/shoulder 
BL_MOTOR_2  = 9   # back-left   upper leg
BL_MOTOR_3  = 10  # back-left   lower leg

BR_MOTOR_1  = 11  # back-right  hip/shoulder
BR_MOTOR_2  = 12  # back-right  upper leg
BR_MOTOR_3  = 13  # back-right  lower leg

# stand angle
STAND_ANGLES = {
    # camera angles
    CAM_X:      90, #0
    CAM_Y:      90, #1
    
    # front left motors
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 90, #3
    FL_MOTOR_3: 0, #4

    # front right motors
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 90, #6
    FR_MOTOR_3: 180, #7

    # back left motors
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 90, #9
    BL_MOTOR_3: 0, #10

    # back right motors
    BR_MOTOR_1: 110, #11
    BR_MOTOR_2: 90, #12
    BR_MOTOR_3: 180, #13
}

# resting angle
resting_angles = {
    # camera angles
    CAM_X:      90, #0
    CAM_Y:      90, #1
    
    # front left motors
    FL_MOTOR_1: 110, #2
    FL_MOTOR_2: 180, #3
    FL_MOTOR_3: 180, #4

    # front right motors
    FR_MOTOR_1: 110, #5
    FR_MOTOR_2: 0, #6
    FR_MOTOR_3: 0, #7

    # back left motors
    BL_MOTOR_1: 130, #8
    BL_MOTOR_2: 180, #9
    BL_MOTOR_3: 180, #10

    # back right motors
    BR_MOTOR_1: 110, #11
    BR_MOTOR_2: 0, #12
    BR_MOTOR_3: 0, #13
}

# TODO : add the return to normal positions for the motors when the script ends

# fn to set the sholders
def set_sholders():
    print("Running set sholders fn...")

    # set front sholders
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_1,STAND_ANGLES[FL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_1,STAND_ANGLES[FR_MOTOR_1])

    time.sleep(0.5)

    # set back sholders
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_1,STAND_ANGLES[BL_MOTOR_1])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_1,STAND_ANGLES[BR_MOTOR_1])

    print("Finished setting sholders.")
    rgb_led_imp.blink_oke()

# fn to set main leg to neutral
def set_main_legs():
    print("Running setting main legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2,STAND_ANGLES[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2,STAND_ANGLES[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2,STAND_ANGLES[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2,STAND_ANGLES[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to set lower leg to neutral
def set_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3,STAND_ANGLES[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3,STAND_ANGLES[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3,STAND_ANGLES[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3,STAND_ANGLES[BR_MOTOR_3])

    print("Finished setting lower legs.")
    rgb_led_imp.blink_oke()


# fn to rest the middle legs
def rest_main_legs():
    print("Running rest main legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_2,resting_angles[FL_MOTOR_2])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_2,resting_angles[FR_MOTOR_2])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_2,resting_angles[BL_MOTOR_2])
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_2,resting_angles[BR_MOTOR_2])

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to rest lower leg
def resting_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    # set front motors
    servo_driver_imp.driver_set_servo_angle_with_index(FL_MOTOR_3,resting_angles[FL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(FR_MOTOR_3,resting_angles[FR_MOTOR_3])

    time.sleep(0.9) # small delay to reduce power draw in the moment

    # set back motors
    servo_driver_imp.driver_set_servo_angle_with_index(BL_MOTOR_3,resting_angles[BL_MOTOR_3])
    time.sleep(0.5)
    servo_driver_imp.driver_set_servo_angle_with_index(BR_MOTOR_3,resting_angles[BR_MOTOR_3])

    print("Finished resting lower legs.")
    rgb_led_imp.blink_oke()

# loop to not end script
run_loop = True

try:
    while run_loop:

        # init all motors
        set_sholders()
        set_main_legs()
        set_lower_legs()
        
        print("[=======[New cycle]=========]")
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
    resting_lower_legs()
    rest_main_legs()
    print("Legs resting now release the motors...")
    time.sleep(1)
    servo_driver_imp.cleanup_servos()
    time.sleep(1)
    rgb_led_imp.turn_off_strip()
    print("Finished")
