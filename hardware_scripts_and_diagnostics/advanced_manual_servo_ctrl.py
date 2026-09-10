# imports
import servo_driver_imp
import time
import threading

print("Servo control script started...")

run_loop = True

try:

    while run_loop:

        user_mode = input("Enter mode (s/c) or q to quit : ")
        print(user_mode)

        if user_mode == "q":
            run_loop = False
            break

        if user_mode == "s":

            servo_indexes = input("Enter servo index(es) (example: 0 or 0,1,2) : ")
            print(servo_indexes)

            angle = int(input("Enter angle : "))
            print(angle)

            max_range = int(input("Enter servo range : "))
            print(max_range)

            indexes = servo_indexes.split(",")

            for index in indexes:
                servo_driver_imp.set_servo_range(
                    index.strip(),
                    max_range
                )

            for index in indexes:
                servo_driver_imp.driver_set_servo_angle_with_index(
                    index.strip(),
                    angle
                )

            print("Operation finished.")

        if user_mode == "c":

            concurrent_mode = input(
                "Enter movement (front_middle/back_middle/front_lower/back_lower) : "
            )
            print(concurrent_mode)

            direction = input("Enter direction (fwd/bck) : ")
            print(direction)

            increment = int(input("Enter increment : "))
            print(increment)

            if concurrent_mode == "front_middle":
                servo_driver_imp.move_front_middle_legs(
                    direction,
                    increment
                )

            if concurrent_mode == "back_middle":
                servo_driver_imp.move_back_middle_legs(
                    direction,
                    increment
                )

            if concurrent_mode == "front_lower":
                servo_driver_imp.move_front_lower_legs(
                    direction,
                    increment
                )

            if concurrent_mode == "back_lower":
                servo_driver_imp.move_back_lower_legs(
                    direction,
                    increment
                )

            print("Concurrent movement finished.")

        if user_mode != "single" and user_mode != "concurrent":
            print("Invalid mode.")

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Cleaning up...")

    servo_driver_imp.cleanup_servos()

    print("Script ended.")
