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


# set servo angle fn (supports floating-point sub-degree precision on PCA9685)
def driver_set_servo_angle_with_index(index, angle):
    print("Setting the angle of servo with index " + str(index) + ", to : " + str(round(float(angle), 2)))
    if (kit != None):
        try:
            kit.servo[int(index)].angle = float(angle)
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

# [[[[[[[[[[[ Inverse motor fn here ]]]]]]]]]]]

# move left side motor forward (inverse motor_fn)
def move_invers_motor_foraward(motor_index,current_rot,increment=10):
    print("Move invers motor forward fn called !!")

    if (kit.servo[int(motor_index)].angle != None):
        current_angle =  kit.servo[int(motor_index)].angle
    else:
        current_angle = current_rot

    print("Moving motor : " + str(motor_index) + " with angle " + str(current_angle) + " forward by increment of " + str(increment))

    angle_to_set_to = current_angle-increment
    
    driver_set_servo_angle_with_index(motor_index,angle_to_set_to)

# move left side motor backward (inverse motor_fn)
def move_invers_motor_backward(motor_index,current_rot,increment=10):
    print("Move invers motor backward fn called !!")

    if (kit.servo[int(motor_index)].angle != None):
        current_angle =  kit.servo[int(motor_index)].angle
    else:
        current_angle = current_rot

    print("Moving motor : " + str(motor_index) + " with angle " + str(current_angle) + " backward by increment of " + str(increment))

    angle_to_set_to = current_angle+increment
    
    driver_set_servo_angle_with_index(motor_index,angle_to_set_to)

# move the inverse motor with index to starting position
def inverse_motor_to_start(motor_index):
    print("Inverse motor to start fn called !!")
    driver_set_servo_angle_with_index(motor_index,180)

def inverse_motor_to_end(motor_index):
    print("Inverse motor to end fn called !!")
    driver_set_servo_angle_with_index(motor_index,0)


#[[[[[[[[ Normal motor fn here ]]]]]]]] 

# move right side motor forward (normal motor_fn)
def move_normal_motor_foraward(motor_index,current_rot,increment=10):
    print("Move normal motor forward fn called !!")

    if (kit.servo[int(motor_index)].angle != None):
        current_angle =  kit.servo[int(motor_index)].angle
    else:
        current_angle = current_rot

    print("Moving motor : " + str(motor_index) + " with angle " + str(current_angle) + " forward by increment of " + str(increment))

    angle_to_set_to = current_angle+increment
    
    driver_set_servo_angle_with_index(motor_index,angle_to_set_to)

# move right side motor backward (normal motor_fn)
def move_normal_motor_backward(motor_index,current_rot,increment=10):
    print("Move normal motor backward fn called !!")

    if (kit.servo[int(motor_index)].angle != None):
        current_angle =  kit.servo[int(motor_index)].angle
    else:
        current_angle = current_rot

    print("Moving motor : " + str(motor_index) + " with angle " + str(current_angle) + " backward by increment of " + str(increment))

    angle_to_set_to = current_angle-increment
    
    driver_set_servo_angle_with_index(motor_index,angle_to_set_to)

# move the normal motor with index to starting position
def normal_motor_to_start(motor_index):
    print("Normal motor to start fn called !!")
    driver_set_servo_angle_with_index(motor_index,0)

# move normal motor to final position
def normal_motor_to_end(motor_index):
    print("Normal motor to end fn called !!")
    driver_set_servo_angle_with_index(motor_index,180)


# =============> Concurent motor movment fns <=====================

# move front middle legs fn (direction = fwd,bck)
def move_front_middle_legs(direction,increment=10):
    print("Fn to move front middle legs called !!")
    print("Moving in direction : " + str(direction))

    amount_to_move = increment

    if (direction == "fwd"):
        print("Moving front middle legs forward.")

        move_invers_motor_foraward(3,180,amount_to_move)
        move_normal_motor_foraward(6,0,amount_to_move)

    if (direction == "bck"):
        print("Moving front middle legs back.")
        
        move_invers_motor_backward(3,0,amount_to_move)
        move_normal_motor_backward(6,180,amount_to_move)
    
    print("Fn finished !!")

# move back middle legs fn (direction = fwd,bck)
def move_back_middle_legs(direction,increment=10):
    print("Fn to move back middle legs called !!")
    print("Moving in direction : " + str(direction))

    amount_to_move = increment

    if (direction == "fwd"):
        print("Moving back middle legs forward.")

        move_invers_motor_foraward(9,180,amount_to_move)
        move_normal_motor_foraward(12,0,amount_to_move)

    if (direction == "bck"):
        print("Moving back middle legs back.")
        
        move_invers_motor_backward(9,0,amount_to_move)
        move_normal_motor_backward(12,180,amount_to_move)
    
    print("Fn finished !!")

# < lower legs fns >

# move front lower legs fn (direction = fwd,bck)
def move_front_lower_legs(direction,increment=10):
    print("Fn to move front lower legs called !!")
    print("Moving in direction : " + str(direction))

    amount_to_move = increment

    if (direction == "fwd"):
        print("Moving front lower legs forward.")

        move_invers_motor_foraward(4,180,amount_to_move)
        move_normal_motor_foraward(7,0,amount_to_move)

    if (direction == "bck"):
        print("Moving front lower legs back.")
        
        move_invers_motor_backward(4,0,amount_to_move)
        move_normal_motor_backward(7,180,amount_to_move)
    
    print("Fn finished !!")

# move back lower legs fn (direction = fwd,bck)
def move_back_lower_legs(direction,increment=10):
    print("Fn to move back lower legs called !!")
    print("Moving in direction : " + str(direction))

    amount_to_move = increment

    if (direction == "fwd"):
        print("Moving back lower legs forward.")

        move_invers_motor_foraward(10,180,amount_to_move)
        move_normal_motor_foraward(13,0,amount_to_move)

    if (direction == "bck"):
        print("Moving back lower legs back.")
        
        move_invers_motor_backward(10,0,amount_to_move)
        move_normal_motor_backward(13,180,amount_to_move)
    
    print("Fn finished !!")

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

