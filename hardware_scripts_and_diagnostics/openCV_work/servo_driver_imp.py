#  imports
from adafruit_servokit import ServoKit
import time

# init servo kit
try:
    print("Init servo driver object")
    kit = ServoKit(channels=16)
    print("Init succesful.")
except Exception as e:
    print("Error init kit object : ", e)
    kit = None
    print("Kit obj set as None")


# function to set servo actuation range
def set_servo_range(servo_index, max_range):

    print("Setting servo " + str(servo_index) + " range to : " + str(max_range))

    if (kit != None):
        try:
            kit.servo[int(servo_index)].set_pulse_width_range(500,2500)
            kit.servo[int(servo_index)].actuation_range = int(max_range)
            print("Servo range set successfully.")
        except Exception as e:
            print("Could not set servo range : ", e)


# set servo angle fn
def driver_set_servo_angle_with_index(index, angle):
    print("Setting the angle of servo with index " + str(index) + ", to : " + str(angle))
    if (kit != None):
        try:
            kit.servo[int(index)].angle = int(angle)
        except Exception as e:
            print("Could not set servo.")
    print("Operation finished.")


# fn to snap servo to the left side
def snap_servo_left(servo_index):
    print("Snaping servo to the left!!!")
    driver_set_servo_angle_with_index(servo_index,180)


# fn to snap servo to the right side
def snap_servo_right(servo_index):
    print("Snapping servo to the right!!!")
    driver_set_servo_angle_with_index(servo_index,0)


# pan to left
def pan_to_left(current_rot,servo_index,pan_speed=0.01):
    print("Running fn to pan to left...")
    rot_i = int(current_rot)
    while (rot_i <= 180):
        driver_set_servo_angle_with_index(servo_index,rot_i)
        rot_i += 10
        time.sleep(float(pan_speed));
    print("Finished panning to left")


# pan to Right
def pan_to_right(current_rot,servo_index,pan_speed=0.01):
    print("Running fn to pan to right...")
    rot_i = int(current_rot)
    while (rot_i >= 0):
        driver_set_servo_angle_with_index(servo_index,rot_i)
        rot_i -= 10
        time.sleep(float(pan_speed));
    print("Finished panning to right")


# function to cleanup or release the servo motors
def cleanup_servos():
    
    print("Cleanup servo driver function called !!")
    
    for i in range(16):
        try:
            kit.servo[i].angle = None
            time.sleep(0.1)
        except:
            pass
    
    print("Servo cleanup finished.")
