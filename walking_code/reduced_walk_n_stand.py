# imports
import servo_driver_imp
import rgb_led_imp
import time
import math

# mode variables
reduced_mode = True # True = move using only the middle leg (thigh) motors, False = normal mode (not added to this script yet)
run_mode = "stand" # what to do after standing up : direction_test / stand / walk / backward / turn_left / turn_right
knees_energized = False # False = lower leg motors are never powered, True = they hold the collapsed angle
log_tilt = False # True prints pitch and roll from the gyro while the legs move

if log_tilt:
    import gyro_imp

# servo indexes
FL_MOTOR_1 = 2  # front left shoulder
FL_MOTOR_2 = 3  # front left upper leg (thigh)
FL_MOTOR_3 = 4  # front left lower leg (knee)

FR_MOTOR_1 = 5  # front right shoulder
FR_MOTOR_2 = 6  # front right upper leg (thigh)
FR_MOTOR_3 = 7  # front right lower leg (knee)

BL_MOTOR_1 = 8   # back left shoulder
BL_MOTOR_2 = 9   # back left upper leg (thigh)
BL_MOTOR_3 = 10  # back left lower leg (knee)

BR_MOTOR_1 = 11  # back right shoulder
BR_MOTOR_2 = 12  # back right upper leg (thigh)
BR_MOTOR_3 = 13  # back right lower leg (knee)

SHOULDERS = [FL_MOTOR_1, FR_MOTOR_1, BL_MOTOR_1, BR_MOTOR_1] # shoulder motors, held fixed in reduced mode
THIGHS = [FL_MOTOR_2, FR_MOTOR_2, BL_MOTOR_2, BR_MOTOR_2] # middle leg motors, the only ones that move in reduced mode
KNEES = [FL_MOTOR_3, FR_MOTOR_3, BL_MOTOR_3, BR_MOTOR_3] # lower leg motors, left unpowered in reduced mode

THIGH_NAMES = {FL_MOTOR_2: "front left", FR_MOTOR_2: "front right", BL_MOTOR_2: "back left", BR_MOTOR_2: "back right"}
THIGH_SIDE = {FL_MOTOR_2: "left", FR_MOTOR_2: "right", BL_MOTOR_2: "left", BR_MOTOR_2: "right"}

# reduced mode angles found by hand
REDUCED_SHOULDER_ANGLES = {FL_MOTOR_1: 110, FR_MOTOR_1: 110, BL_MOTOR_1: 130, BR_MOTOR_1: 80} # shoulders while standing on the folded legs
REDUCED_THIGH_STAND = {FL_MOTOR_2: 78, FR_MOTOR_2: 100, BL_MOTOR_2: 87, BR_MOTOR_2: 89} # thigh angles where the robot sits level
THIGH_MIN = {FL_MOTOR_2: 0, FR_MOTOR_2: 0, BL_MOTOR_2: 50, BR_MOTOR_2: 0} # lowest angle each thigh may be sent while moving
THIGH_MAX = {FL_MOTOR_2: 150, FR_MOTOR_2: 180, BL_MOTOR_2: 180, BR_MOTOR_2: 180} # highest angle each thigh may be sent while moving
KNEE_COLLAPSED = {FL_MOTOR_3: 180, FR_MOTOR_3: 0, BL_MOTOR_3: 180, BR_MOTOR_3: 0} # collapsed angles, only used if knees_energized is True

# rest pose, the pose the robot starts in and returns to
REST_SHOULDER_ANGLES = {FL_MOTOR_1: 110, FR_MOTOR_1: 110, BL_MOTOR_1: 130, BR_MOTOR_1: 110}
REST_THIGH_ANGLES = {FL_MOTOR_2: 180, FR_MOTOR_2: 0, BL_MOTOR_2: 180, BR_MOTOR_2: 0}

# 1 = raising the angle moves the foot forward, -1 = lowering it does
# worked out from the rest pose (left thighs rest at 180 pointing forward, right thighs at 0), check it with run_mode direction_test
THIGH_FORWARD_SIGN = {FL_MOTOR_2: 1, FR_MOTOR_2: -1, BL_MOTOR_2: 1, BR_MOTOR_2: -1}

# gait settings, start small and slow and raise them one at a time
swing_amplitude = 6.0 # degrees each thigh swings either side of its swing center
swing_bias = 0.0 # degrees the swing center is shifted forward (+) or backward (-), tune so the foot is lowest in the middle of the push
step_time = 3.0 # seconds for one full cycle of one leg
power_stroke_fraction = 0.7 # share of the cycle the foot moves backward (pushes the body), the rest is the faster return
walk_step_count = 3 # full cycles to run in walk, backward and turn modes
test_swing = 8.0 # degrees used by direction_test

LEG_PHASE = {FL_MOTOR_2: 0.0, BR_MOTOR_2: 0.25, FR_MOTOR_2: 0.5, BL_MOTOR_2: 0.75} # where each leg is in its cycle, one quarter cycle apart

# movement timing
loop_dt = 0.02 # seconds between servo updates
shoulder_move_time = 1.0 # seconds to move the shoulders
stand_move_time = 2.5 # seconds for each pair of thighs to stand up
rest_move_time = 2.5 # seconds for each pair of thighs to go back to rest
return_move_time = 1.5 # seconds to ease back to the stand after walking
tilt_log_interval = 0.5 # seconds between tilt prints when log_tilt is True

# state
current_angles = {} # last angle written to each servo, the robot is assumed to start in the rest pose
for index, angle in REST_SHOULDER_ANGLES.items():
    current_angles[index] = angle
for index, angle in REST_THIGH_ANGLES.items():
    current_angles[index] = angle
for index, angle in KNEE_COLLAPSED.items():
    current_angles[index] = angle
robot_standing = False # True while the robot is up on the folded legs
servos_used = False # True once the script has powered any servo


# helper fn to clamp a value between a min and max
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# helper fn to ease a 0 to 1 value so movements start and stop gently
def smooth(t):
    return 0.5 - 0.5 * math.cos(math.pi * t)

# fn to write one servo directly through the driver kit
# the driver module prints on every call and cuts angles to whole degrees, both would make the walking rough
def write_servo(index, angle):
    if servo_driver_imp.kit is None:
        return
    angle = clamp_value(float(angle), 0.0, 180.0)
    try:
        servo_driver_imp.kit.servo[index].angle = angle
        current_angles[index] = angle
    except Exception as e:
        print("Could not set servo " + str(index) + " : " + str(e))

# fn to release a servo so it is not energized
def release_servo(index):
    if servo_driver_imp.kit is None:
        return
    try:
        servo_driver_imp.kit.servo[index].angle = None
    except Exception as e:
        print("Could not release servo " + str(index) + " : " + str(e))

# fn to move several servos smoothly to their targets
# targets is a dict of servo index and target angle, the time is real time so slow i2c writes do not stretch the move
def move_servos_smooth(targets, duration):
    starts = {}
    for index in targets:
        starts[index] = current_angles[index]
    start_time = time.time()
    while True:
        t = min(1.0, (time.time() - start_time) / duration)
        eased = smooth(t)
        for index, target in targets.items():
            write_servo(index, starts[index] + (target - starts[index]) * eased)
        if t >= 1.0:
            break
        time.sleep(loop_dt)

# fn to set the servo ranges before any movement
def init_servo_ranges():
    print("Initialising servo ranges...")
    used = SHOULDERS + THIGHS
    if knees_energized:
        used = used + KNEES
    for index in used:
        servo_driver_imp.set_servo_range(index, 180)
        time.sleep(0.05)
    print("Servo ranges set.")

# fn to leave the lower leg motors unpowered, or hold them collapsed if knees_energized is True
def prepare_knees():
    if knees_energized:
        print("Holding the lower leg motors at the collapsed angles...")
        for index in KNEES:
            write_servo(index, KNEE_COLLAPSED[index])
    else:
        print("Releasing the lower leg motors, they stay unpowered in reduced mode...")
        for index in KNEES:
            release_servo(index)

# fn to stand up on the folded legs
def stand_up_reduced():
    global robot_standing
    print("Running stand up in reduced mode...")

    move_servos_smooth(REDUCED_SHOULDER_ANGLES, shoulder_move_time)
    time.sleep(0.5)

    # front thighs first
    move_servos_smooth({FL_MOTOR_2: REDUCED_THIGH_STAND[FL_MOTOR_2], FR_MOTOR_2: REDUCED_THIGH_STAND[FR_MOTOR_2]}, stand_move_time)
    time.sleep(0.5) # small delay to reduce power draw in the moment

    # back thighs
    move_servos_smooth({BL_MOTOR_2: REDUCED_THIGH_STAND[BL_MOTOR_2], BR_MOTOR_2: REDUCED_THIGH_STAND[BR_MOTOR_2]}, stand_move_time)

    robot_standing = True
    print("Finished standing up.")
    rgb_led_imp.blink_oke()

# fn to ease the thighs back to the stand angles
def return_to_stand():
    print("Returning to the reduced stand...")
    move_servos_smooth(REDUCED_THIGH_STAND, return_move_time)

# fn to go back to the rest pose
def rest_down():
    global robot_standing
    print("Running rest down...")

    # back thighs first
    move_servos_smooth({BL_MOTOR_2: REST_THIGH_ANGLES[BL_MOTOR_2], BR_MOTOR_2: REST_THIGH_ANGLES[BR_MOTOR_2]}, rest_move_time)
    time.sleep(0.5)

    # front thighs
    move_servos_smooth({FL_MOTOR_2: REST_THIGH_ANGLES[FL_MOTOR_2], FR_MOTOR_2: REST_THIGH_ANGLES[FR_MOTOR_2]}, rest_move_time)
    time.sleep(0.5)

    move_servos_smooth(REST_SHOULDER_ANGLES, shoulder_move_time)
    robot_standing = False
    print("Finished resting.")

# fn to swing each thigh a little toward its assumed forward direction, one at a time
def direction_test():
    print("Running direction test, watch each foot...")
    for index in THIGHS:
        stand_angle = REDUCED_THIGH_STAND[index]
        forward_angle = clamp_value(stand_angle + THIGH_FORWARD_SIGN[index] * test_swing, THIGH_MIN[index], THIGH_MAX[index])
        print("Thigh " + str(index) + " (" + THIGH_NAMES[index] + ") moving to " + str(forward_angle) + ", the foot should move FORWARD (toward the head)")
        move_servos_smooth({index: forward_angle}, 1.5)
        time.sleep(1.0)
        move_servos_smooth({index: stand_angle}, 1.5)
        time.sleep(0.5)
    print("If a foot moved backward, flip its sign in THIGH_FORWARD_SIGN.")

# fn for the shape of one leg cycle, 1 = foot at the forward end, -1 = foot at the back end
# the first part of the cycle is the slow push (foot moves backward), the rest is the faster return
def gait_wave(phase):
    phase = phase % 1.0
    if phase < power_stroke_fraction:
        t = phase / power_stroke_fraction
        return 1.0 - 2.0 * smooth(t)
    t = (phase - power_stroke_fraction) / (1.0 - power_stroke_fraction)
    return -1.0 + 2.0 * smooth(t)

# fn to run the gait for a number of cycles
# left_direction and right_direction : 1 = push the body forward, -1 = push it backward, so walking uses 1 and 1, turning uses opposite signs
def run_gait(cycles, left_direction, right_direction):
    print("Running gait for " + str(cycles) + " cycles...")

    # warn if the swing would run into a thigh limit
    for index in THIGHS:
        reach = swing_amplitude + abs(swing_bias)
        if REDUCED_THIGH_STAND[index] - reach < THIGH_MIN[index] or REDUCED_THIGH_STAND[index] + reach > THIGH_MAX[index]:
            print("Warning : thigh " + str(index) + " (" + THIGH_NAMES[index] + ") will be held back by its limit, lower swing_amplitude or swing_bias")

    total_time = cycles * step_time
    start_time = time.time()
    last_log = 0.0
    while True:
        elapsed = time.time() - start_time
        if elapsed >= total_time:
            break

        ramp = smooth(min(1.0, elapsed / step_time)) # eases the swing in over the first cycle

        for index in THIGHS:
            direction = left_direction if THIGH_SIDE[index] == "left" else right_direction
            wave = gait_wave(elapsed / step_time + LEG_PHASE[index])
            offset = swing_bias * ramp + direction * swing_amplitude * ramp * wave
            angle = REDUCED_THIGH_STAND[index] + THIGH_FORWARD_SIGN[index] * offset
            write_servo(index, clamp_value(angle, THIGH_MIN[index], THIGH_MAX[index]))

        if log_tilt and elapsed - last_log >= tilt_log_interval:
            last_log = elapsed
            tilt = gyro_imp.get_pitch_roll()
            print("Tilt > " + str(tilt))

        time.sleep(loop_dt)

    return_to_stand()
    print("Gait finished.")
    rgb_led_imp.blink_oke()

# fn to walk forward
def walk_forward(cycles):
    print("Walk forward")
    run_gait(cycles, 1, 1)

# fn to walk backward
def walk_backward(cycles):
    print("Walk backward")
    run_gait(cycles, -1, -1)

# fn to turn left, right legs push forward and left legs push backward
def turn_left(cycles):
    print("Turn left")
    run_gait(cycles, -1, 1)

# fn to turn right, left legs push forward and right legs push backward
def turn_right(cycles):
    print("Turn right")
    run_gait(cycles, 1, -1)

# fn to run the script
def main():
    global servos_used
    try:
        print("Reduced mode script started...")

        if not reduced_mode:
            print("reduced_mode is False - normal mode is not added to this script yet, nothing will move.")
            return

        if servo_driver_imp.kit is None:
            print("Servo driver is not ready, nothing will move.")
            return

        input("Put the robot in the rest pose, then press Enter to start > ")

        servos_used = True
        init_servo_ranges()
        prepare_knees()
        stand_up_reduced()

        if run_mode == "direction_test":
            direction_test()
        elif run_mode == "walk":
            walk_forward(walk_step_count)
        elif run_mode == "backward":
            walk_backward(walk_step_count)
        elif run_mode == "turn_left":
            turn_left(walk_step_count)
        elif run_mode == "turn_right":
            turn_right(walk_step_count)
        elif run_mode == "stand":
            print("Standing only.")
        else:
            print("Unknown run_mode : " + str(run_mode))

        input("Holding the reduced stand. Press Enter to go back to rest > ")

    except KeyboardInterrupt:
        print("Keyboard interrupt detected - stopping script...")

    except Exception as e:
        print("Error happened : ", e)

    finally:
        print("Script ended - stopping")
        if servos_used:
            try:
                if robot_standing:
                    return_to_stand()
                    rest_down()
            except Exception as e:
                print("Could not return to rest : ", e)
            time.sleep(1)
            servo_driver_imp.cleanup_servos()
            time.sleep(1)
            rgb_led_imp.turn_off_strip()
        print("Finished")

if __name__ == "__main__":
    main()
