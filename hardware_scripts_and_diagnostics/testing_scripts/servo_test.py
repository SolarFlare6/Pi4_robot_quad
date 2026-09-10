from gpiozero import Servo
from time import sleep

# Define servo on GPIO 24
servo = Servo(24)

try:
    while True:
        servo.min()  # Turn to minimum angle (-90 degrees)
        sleep(1)
        servo.mid()  # Turn to center (0 degrees)
        sleep(1)
        servo.max()  # Turn to maximum angle (+90 degrees)
        sleep(1)
except KeyboardInterrupt:
    print("Program stopped")

