# import audio module
import speaker_audio_imp
import time

# halt the script to init audio import
time.sleep(1)

# imports
import mini_speaker_imp
import rgb_led_imp
import servo_driver_imp
import gyro_imp
import RPi.GPIO as GPIO 

# define vars
run_loop = True

# def vars for code segments
run_servo_code = False
run_led_code = False
run_speaker_code = False
run_gyro_code = False

print("Manual robot script started...")

try:
    while run_loop:

        print("[=======[New cycle]=========]")

        user_end_script = input("End script? y/n > ")
        print(user_end_script)

        if not (user_end_script == "y"):

            user_input_run_servo_code = input("Run servo code? y/n > ")
            print(user_input_run_servo_code)

            user_input_run_speaker_code = input("Run speaker code? y/n > ")
            print(user_input_run_speaker_code)

            user_input_run_led_code = input("Run led code? y/n > ")
            print(user_input_run_led_code)

            user_input_run_gyro_code = input("Run gyro code? y/n > ")
            print(user_input_run_gyro_code)

            if user_input_run_servo_code == "y":
                run_servo_code = True
            else:
                run_servo_code = False

            if user_input_run_speaker_code == "y":
                run_speaker_code = True
            else:
                run_speaker_code = False

            if user_input_run_led_code == "y":
                run_led_code = True
            else:
                run_led_code = False

            if user_input_run_gyro_code == "y":
                run_gyro_code = True
            else:
                run_gyro_code = False

        if run_speaker_code:

            user_input_speaker_mode = input("Enter beep/mus/audio : ")
            print(user_input_speaker_mode)

            if user_input_speaker_mode == "beep":
                print("Playing beep...")
                mini_speaker_imp.play_tone(70)
                rgb_led_imp.blink_oke()

            if user_input_speaker_mode == "mus":
                print("Playing notes list...")
                mini_speaker_imp.play_list_of_notes(
                    ['C4', 'D4', 'E4', 'F4', 'G4', 'A4', 'B4', 'C5']
                )
                rgb_led_imp.blink_oke()

            if user_input_speaker_mode == "audio":

                speaker_audio_imp.play_audio("cool_robo_beat.ogg")
                time.sleep(0.2)
                rgb_led_imp.blink_oke()

        if run_servo_code:

            user_input_servo_mode = input("Enter angle/snap/pan : ")
            print(user_input_servo_mode)

            servo_index = input("Enter servo index : ")
            print(servo_index)

            if user_input_servo_mode == "angle":

                servo_angle = input("Enter servo angle : ")
                print(servo_angle)

                servo_driver_imp.driver_set_servo_angle_with_index(
                    servo_index,
                    servo_angle
                )

                rgb_led_imp.blink_oke()

            if user_input_servo_mode == "snap":

                inp_l_or_r = input("Enter l/r : ")
                print(inp_l_or_r)

                if inp_l_or_r == "l":
                    servo_driver_imp.snap_servo_left(
                        servo_index
                    )

                if inp_l_or_r == "r":
                    servo_driver_imp.snap_servo_right(
                        servo_index
                    )

                rgb_led_imp.blink_oke()

            if user_input_servo_mode == "pan":

                current_rot = input("Enter current rotation : ")
                print(current_rot)

                inp_l_or_r = input("Enter l/r : ")
                print(inp_l_or_r)

                pan_speed = input("Enter pan speed (e.g. 0.01) : ")
                print(pan_speed)

                if inp_l_or_r == "l":
                    servo_driver_imp.pan_to_left(
                        current_rot,
                        servo_index,
                        pan_speed
                    )

                if inp_l_or_r == "r":
                    servo_driver_imp.pan_to_right(
                        current_rot,
                        servo_index,
                        pan_speed
                    )

                rgb_led_imp.blink_oke()

            time.sleep(0.8)

        if run_led_code:

            user_input_led_mode = input("Enter strip/index/anim : ")
            print(user_input_led_mode)

            red_val = input("Red val : ")
            print(red_val)

            green_val = input("Green val : ")
            print(green_val)

            blue_val = input("Blue val : ")
            print(blue_val)

            if user_input_led_mode == "strip":
                print("Strip on!")

                rgb_led_imp.turn_on_strip_with_color(
                    red_val,
                    green_val,
                    blue_val
                )

            if user_input_led_mode == "index":
                print("Turning on led at given index...")

                led_index = input("Enter index from 0 to 7 : ")
                print(led_index)

                rgb_led_imp.turn_on_led_at_index(
                    led_index,
                    red_val,
                    green_val,
                    blue_val
                )

            if user_input_led_mode == "anim":
                print("Running animation code for led plate...")

                rgb_led_imp.move_pixel_from_left_to_right(
                    red_val,
                    green_val,
                    blue_val
                )

            time.sleep(0.8)

        if run_gyro_code:

            user_input_gyro_mode = input(
                "Enter accel/gyro/tilt : "
            )
            print(user_input_gyro_mode)

            if user_input_gyro_mode == "accel":
                data = gyro_imp.get_sensor_accel()
                print(data)

            if user_input_gyro_mode == "gyro":
                data = gyro_imp.get_sensor_gyro()
                print(data)

            if user_input_gyro_mode == "tilt":
                data = gyro_imp.get_pitch_roll()
                print(data)

            rgb_led_imp.blink_oke()

        if user_end_script == "y":
            print("Ending script...")
            run_loop = False

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Script ended - stopping")
    time.sleep(1)
    servo_driver_imp.cleanup_servos()
    time.sleep(1)
    rgb_led_imp.turn_off_strip()
