# robot_pi/device_server/robot_conf_fn.py
"""Quadruped Robot Action Functions called when actions are triggered from the MachineMake mobile app.

Bridges app commands directly to your custom locomotion placeholders, voice synthesizer,
and real hardware drivers (PCA9685 servos, WS281x RGB LED strip, Tonal Buzzer, Speaker Audio, MPU6050).
"""

import logging

logger = logging.getLogger("robot.conf_fn")

_robot_controller = None


def set_robot_controller(controller):
    global _robot_controller
    _robot_controller = controller


def get_robot_controller():
    return _robot_controller


def _get_loco():
    try:
        from navigation import locomotion_placeholders
        return locomotion_placeholders
    except Exception:
        return None


# -- Locomotion & High-Level Actions (Delegates to locomotion_placeholders.py) --

def walk_forward(distance=1.0):
    print("walk forward fn called !!")
    loco = _get_loco()
    if loco:
        loco.walk_forward(distance=distance)


def walk_backward(distance=1.0):
    print("walk backward fn called !!")
    loco = _get_loco()
    if loco:
        loco.walk_backward(distance=distance)


def walk_left(distance=1.0):
    print("walk left fn called !!")
    loco = _get_loco()
    if loco:
        loco.walk_left(distance=distance)


def walk_right(distance=1.0):
    print("walk right fn called !!")
    loco = _get_loco()
    if loco:
        loco.walk_right(distance=distance)


def walk(direction="forward", distance=1.0):
    print("walk fn called !!")
    d = str(direction).lower().strip()
    if d in ("forward", "fwd"):
        walk_forward(distance)
    elif d in ("backward", "back", "bwd"):
        walk_backward(distance)
    elif d == "left":
        walk_left(distance)
    elif d == "right":
        walk_right(distance)


def turn_left(angle=45.0):
    print("turn left fn called !!")
    loco = _get_loco()
    if loco:
        loco.turn_left(angle=angle)


def turn_right(angle=45.0):
    print("turn right fn called !!")
    loco = _get_loco()
    if loco:
        loco.turn_right(angle=angle)


def turn(direction="left", angle=45.0):
    print("turn fn called !!")
    d = str(direction).lower().strip()
    if d == "left":
        turn_left(angle)
    elif d == "right":
        turn_right(angle)


def stand():
    print("stand fn called !!")
    loco = _get_loco()
    if loco:
        loco.stand()


def sit():
    print("sit fn called !!")
    loco = _get_loco()
    if loco:
        loco.sit()


def emergency_stop():
    print("emergency stop fn called !!")
    loco = _get_loco()
    if loco:
        loco.emergency_stop()
    else:
        cleanup_servos()


def cleanup_servos():
    print("cleanup servos fn called !!")
    try:
        from cleanup import cleanup_all_hardware
        cleanup_all_hardware()
    except Exception:
        try:
            from drivers import servo_driver_imp
            servo_driver_imp.cleanup_servos()
        except Exception:
            pass


def set_gait(mode="walk"):
    print("set gait fn called !!")
    loco = _get_loco()
    if loco:
        loco.set_gait(mode=mode)


def set_pose(pitch=0.0, roll=0.0, yaw=0.0, height=0.0):
    print("set pose fn called !!")
    loco = _get_loco()
    if loco:
        loco.set_pose(pitch=pitch, roll=roll, yaw=yaw, height=height)


def set_stabilizer(enabled=True):
    print("set stabilizer fn called !!")


def set_autonomy(enabled=True):
    print(f"set autonomy fn called (enabled={enabled}) !!")


# -- PCA9685 Servo Control & Realtime Mirroring --

def set_servo_angle(channel=0, angle=90.0):
    print(f"set servo angle fn called (ch={channel}, angle={angle}) !!")
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.driver_set_servo_angle_with_index(int(channel), int(angle))
    except Exception:
        pass


def set_servo_pulse(channel=0, pulse_us=1500):
    print("set servo pulse fn called !!")
    calc_angle = (max(500, min(2500, int(pulse_us))) - 500) * 180.0 / 2000.0
    set_servo_angle(channel, calc_angle)


def snap_servo_left(index=0):
    print("snap servo left fn called !!")
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.snap_servo_left(int(index))
    except Exception:
        pass


def snap_servo_right(index=0):
    print("snap servo right fn called !!")
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.snap_servo_right(int(index))
    except Exception:
        pass


def pan_to_left(index=0):
    print("pan to left fn called !!")
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.pan_to_left(90, int(index))
    except Exception:
        pass


def pan_to_right(index=0):
    print("pan to right fn called !!")
    try:
        from drivers import servo_driver_imp
        servo_driver_imp.pan_to_right(90, int(index))
    except Exception:
        pass


def get_servo_angles():
    print("get servo angles fn called !!")


# -- WS281x RGB LED Lighting (GPIO 10) --

def turn_on_strip_with_color(r=255, g=255, b=255):
    print("turn on strip with color fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.turn_on_strip_with_color(int(r), int(g), int(b))
    except Exception:
        pass


def turn_off_strip():
    print("turn off strip fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.turn_off_strip()
    except Exception:
        pass


def turn_on_led_at_index(index=0, r=255, g=255, b=255):
    print("turn on led at index fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.turn_on_led_at_index(int(index), int(r), int(g), int(b))
    except Exception:
        pass


def turn_off_led_at_index(index=0):
    print("turn off led at index fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.turn_off_led_at_index(int(index))
    except Exception:
        pass


def animate_running_process(r=0, g=105, b=105):
    print(f"animate running process fn called with color ({r}, {g}, {b}) !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.animate_running_process(int(r), int(g), int(b))
    except Exception:
        pass


def animate_with_color(r=0, g=105, b=105):
    print(f"animate with color fn called ({r}, {g}, {b}) !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.animate_with_color(int(r), int(g), int(b))
    except Exception:
        pass


def flash_alert():
    print("flash alert fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.flash_alert()
    except Exception:
        pass


def blink_oke():
    print("blink oke fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.blink_oke()
    except Exception:
        pass


def blink_warning():
    print("blink warning fn called !!")
    try:
        from drivers import rgb_led_imp
        rgb_led_imp.blink_warning()
    except Exception:
        pass


def set_led(on=True):
    print("set led fn called !!")
    if on:
        turn_on_strip_with_color(0, 255, 0)
    else:
        turn_off_strip()


# -- Audio Center (Speaker & GPIO 23 Tonal Buzzer) --

def play_tone(tone="C4"):
    print("play tone fn called !!")
    try:
        from drivers import mini_speaker_imp
        mini_speaker_imp.play_tone(tone)
    except Exception:
        pass


def play_list_of_notes(notes=None):
    print("play list of notes fn called !!")
    try:
        from drivers import mini_speaker_imp
        mini_speaker_imp.play_list_of_notes(notes or ["C4", "E4", "G4"])
    except Exception:
        pass


def play_freq(frequency=1000, duration=0.5):
    print("play freq fn called !!")
    try:
        from drivers import mini_speaker_imp
        mini_speaker_imp.play_tone_adv(frequency, duration)
    except Exception as e:
        logger.debug("play_freq error: %s", e)


def stop_tone():
    print("stop tone fn called !!")
    try:
        from drivers import mini_speaker_imp
        mini_speaker_imp.tb.stop()
    except Exception:
        pass


def play_audio(file_path=""):
    print("play audio fn called !!")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.play_audio(file_path)
    except Exception as e:
        logger.debug("play_audio error: %s", e)


def stop_audio():
    print("stop audio fn called !!")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.stop_audio()
    except Exception as e:
        logger.debug("stop_audio error: %s", e)


def set_volume(volume=1.7):
    print(f"set volume fn called (volume={volume}) !!")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.set_volume(float(volume))
    except Exception as e:
        logger.debug("set_volume error: %s", e)


def speak(text="", volume=1.7):
    print(f"speak fn called with text='{text}', volume={volume} !!")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.robot_speak_with_volume(str(text), vol=float(volume))
    except Exception as e:
        logger.debug("speak error: %s", e)


# -- Camera & Video Streaming --

def camera_stream_start(resolution="640x480", fps=30):
    print("camera stream start fn called !!")


def camera_stream_stop():
    print("camera stream stop fn called !!")


def camera_snapshot():
    print("camera snapshot fn called !!")


def take_picture():
    print("take picture fn called !!")


def track_object(x=None, y=None, w=None, h=None, u=None, v=None, bbox=None, label=None):
    print(f"track object fn called (x={x}, y={y}, w={w}, h={h}, u={u}, v={v}, bbox={bbox}, label={label}) !!")


def select_object(x=None, y=None, w=None, h=None, u=None, v=None, bbox=None, label=None):
    print(f"select object fn called (x={x}, y={y}, w={w}, h={h}, u={u}, v={v}, bbox={bbox}, label={label}) !!")


def stop_tracking():
    print("stop tracking fn called !!")


def cancel_tracking():
    print("cancel tracking fn called !!")


def get_tracked_objects():
    print("get tracked objects fn called !!")


# -- Sensors & Telemetry (MPU6050) --

def get_sensor_accel():
    print("get sensor accel fn called !!")
    try:
        from drivers import gyro_imp
        return gyro_imp.get_sensor_accel()
    except Exception:
        return None


def get_sensor_gyro():
    print("get sensor gyro fn called !!")
    try:
        from drivers import gyro_imp
        return gyro_imp.get_sensor_gyro()
    except Exception:
        return None


def get_pitch_roll():
    print("get pitch roll fn called !!")
    try:
        from drivers import gyro_imp
        return gyro_imp.get_pitch_roll()
    except Exception:
        return None


def get_orientation():
    print("get orientation fn called !!")
    return get_pitch_roll()


def get_battery():
    print("get battery fn called !!")


# -- Needle AI Intent Execution --

def needle_prompt(prompt=""):
    print(f"needle prompt fn called: {prompt} !!")
    try:
        from drivers import upgraded_audio_interface_imp
        upgraded_audio_interface_imp.robot_speak(str(prompt))
    except Exception:
        pass


# -- Hardware Control (GPIO & PWM) --

def gpio_write(pin=0, state=False):
    print("gpio write fn called !!")


def gpio_read(pin=0):
    print("gpio read fn called !!")


def pwm_set(channel=0, value=0.0):
    print("pwm set fn called !!")


# -- System Power Operations --

def shutdown():
    print("shutdown fn called !!")


def reboot():
    print("reboot fn called !!")
