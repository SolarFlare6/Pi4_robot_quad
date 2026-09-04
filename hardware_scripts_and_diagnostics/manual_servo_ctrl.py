# imports
import servo_driver_imp

# Note this is the servo manual control script

print("Servo control script started...")

run_loop = True

try:

    while run_loop:

        servo_indexes = input("Enter servo index(es) (example: 0 or 0,1,2) or q to quit : ")
        print(servo_indexes)

        if servo_indexes == "q":
            run_loop = False
            break

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

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Cleaning up...")

    servo_driver_imp.cleanup_servos()

    print("Script ended.")
