# this is the import file for the rgb led strip to show signalisation for the robot
# library imports and other
from rpi_ws281x import PixelStrip, Color
import time

# define vars for function
LED_COUNT = 8        # change to LED count
LED_PIN = 18          # GPIO18 ONLY
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 64
LED_INVERT = False
LED_CHANNEL = 0

# define led strip
strip = PixelStrip(
    LED_COUNT,
    LED_PIN,
    LED_FREQ_HZ,
    LED_DMA,
    LED_INVERT,
    LED_BRIGHTNESS,
    LED_CHANNEL
)

# init the strip
strip.begin()

# function to turn on leds
def turn_on_strip_with_color(red, green, blue):
	print("Running function to turn on the whole led strip/plate with values > " + "red : " + str(red) + ", green : " + str(green) + ", blue : " + str(blue) + "...")
	for i in range(LED_COUNT):
		strip.setPixelColor(i, Color(red, green, blue))
	strip.show()


# function to turn off the leds
def turn_off_strip():
	print("Running function to turn off the whole led strip/plate...")
	for i in range(LED_COUNT):
		strip.setPixelColor(i, Color(0, 0, 0))
	strip.show()


# function to turn on specific led
def turn_on_led_at_index(index, red, green, blue):
	print("Turning on led at index : " + str(index))
	if (index < 0 or index >= LED_COUNT):
		print("Index out of range")
	else:
		strip.setPixelColor(index, Color(red, green, blue))
		strip.show()


# function to turn off specific led
def turn_off_led_at_index(index):
	print("Turning off led at index : " + str(index))
	strip.setPixelColor(index, Color(0, 0, 0))
	strip.show()

def move_pixel_from_left_to_right(red, green, blue):
    turn_off_strip()
    for i in range(LED_COUNT):
        if (i==0):
            turn_on_led_at_index(i,red,green,blue)
            turn_off_led_at_index(i+1)
        if (not(i==0) and not(i==LED_COUNT)):
            turn_off_led_at_index(i-1)
            turn_on_led_at_index(i,red,green,blue)
            turn_off_led_at_index(i+1)
        if (i==LED_COUNT):
            turn_off_led_at_index(i-1)
            turn_on_led_at_index(i,red,green,blue)
        time.sleep(0.5)
    print("Finished animate!!")

def move_pixel_from_right_to_left(red, green, blue):
    turn_off_strip()
    i = 8
    while (i>=0):
        if (i==0):
            turn_on_led_at_index(i,red,green,blue)
            turn_off_led_at_index(i+1)
        if (not(i==0) and not(i==LED_COUNT)):
            turn_off_led_at_index(i-1)
            turn_on_led_at_index(i,red,green,blue)
            turn_off_led_at_index(i+1)
        if (i==LED_COUNT):
            turn_off_led_at_index(i-1)
            turn_on_led_at_index(i,red,green,blue)
        i-=1
        time.sleep(0.5)
    print("Finished animate!!")

def animate_running_process():
    print("Playing animation for a process in progress...")
    move_pixel_from_left_to_right(0,105,105)

def flash_alert():
    print("Flash alert...")
    turn_on_strip_with_color(255,0,0)
    time.sleep(0.8)
    turn_off_strip()

def blink_oke():
    print("Show confirmation...")
    turn_on_strip_with_color(0,255,0)
    time.sleep(0.5)
    turn_off_strip()

def blink_warning():
    print("Blink warining...")
    turn_on_strip_with_color(255,255,0)
    time.sleep(0.2)
    turn_off_strip()

# TODO : add animation function


