#  imports
from adafruit_servokit import ServoKit
import time

# init servo kit
kit = ServoKit(channels=16)

# run servo code
print("Running servo code ...")
print("Setting servo angle to 0")
kit.servo[0].angle = 0
time.sleep(1)
print("Setting servo angle to 90")
kit.servo[0].angle = 90
time.sleep(1)
print("Setting servo angle to 180")
kit.servo[0].angle = 180
time.sleep(1)
print("Setting servo angle to 90")
kit.servo[0].angle = 90

print("End of script for servo driver testing")
