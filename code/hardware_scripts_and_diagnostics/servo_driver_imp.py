#  imports
from adafruit_servokit import ServoKit
import time

# init servo kit
kit = ServoKit(channels=16)

# set servo angle fn
def driver_set_servo_angle_with_index(index, angle):
    print("Setting the angle of servo with index " + str(index) + ", to : " + str(angle))
    kit.servo[int(index)].angle = int(angle)
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
    #Set_servo_rotation(90)
    rot_i = int(current_rot)
    while (rot_i <= 180):
        driver_set_servo_angle_with_index(servo_index,rot_i)
        rot_i += 10
        time.sleep(float(pan_speed));
    print("Finished panning to left")

# pan to Right
def pan_to_right(current_rot,servo_index,pan_speed=0.01):
    print("Running fn to pan to right...")
    #Set_servo_rotation(90)
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