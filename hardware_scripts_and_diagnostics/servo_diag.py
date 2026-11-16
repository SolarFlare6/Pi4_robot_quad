# imports
import servo_driver_imp
import time

print("Servo driver test started...")

run_loop = True

try:

    while run_loop:

        servo_index = input("Enter servo index (0-15) or q to quit : ")
        print(servo_index)

        if servo_index == "q":
            run_loop = False
            break

        max_angle = int(input("Enter max angle : "))
        print(max_angle)

        servo_driver_imp.set_servo_range(
            servo_index,
            max_angle
        )

        step_size = int(input("Enter step size : "))
        print(step_size)

        delay_time = float(input("Enter delay between steps : "))
        print(delay_time)

        print("Moving servo through full range...")

        for angle in range(0, max_angle + 1, step_size):
            servo_driver_imp.driver_set_servo_angle_with_index(
                servo_index,
                angle
            )
            time.sleep(delay_time)

        for angle in range(max_angle, -1, -step_size):
            servo_driver_imp.driver_set_servo_angle_with_index(
                servo_index,
                angle
            )
            time.sleep(delay_time)

        print("Servo test finished.")

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Cleaning up...")

    servo_driver_imp.cleanup_servos()

    print("Script ended.")
