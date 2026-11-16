# servo import header with predefined fn
import RPi.GPIO as gpio
import time
import random

# define board mode
gpio.setmode(gpio.BCM)

# setup the pins
servo_pin = 24
gpio.setup(servo_pin,gpio.OUT)
pwm = gpio.PWM(servo_pin,50)
pwm.start(0)

# angle function
def Set_servo_rotation(ang):
    duty = int(ang) / 18 + 2
    gpio.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    gpio.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)
    print("Set servo angle to :")
    print(ang)

def snap_servo_left():
    print("Snaping servo to the left!!!")
    Set_servo_rotation(180)
    
def snap_servo_right():
    print("Snapping servo to the right!!!")
    Set_servo_rotation(0)

# pan to left
def pan_to_left(current_rot):
    print("Running fn to pan to left...")
    #Set_servo_rotation(90)
    rot_i = int(current_rot)
    while (rot_i <= 180):
        Set_servo_rotation(rot_i)
        rot_i += 10
        time.sleep(0.2);
    print("Finished panning to left")
    

# pan to Right
def pan_to_right(current_rot):
    print("Running fn to pan to right...")
    #Set_servo_rotation(90)
    rot_i = int(current_rot)
    while (rot_i >= 0):
        Set_servo_rotation(rot_i)
        rot_i -= 10
        time.sleep(0.2);
    print("Finished panning to right")

def center_servo():
    print("Centering servo")
    Set_servo_rotation(90)

# cleanup fn
def servo_cleanup():
    print("Called servo cleanup!!!")
    Set_servo_rotation(90);
    pwm.stop();
    gpio.cleanup();
