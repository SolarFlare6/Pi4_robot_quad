# imports
import servo_driver_imp
import rgb_led_imp
import time
import math

# mode variables
reduced_mode = True # True = walk on folded legs, False = normal mode (not added to this script yet)
strictly_thighs = True # True = Option A (4-servo thigh wave with forward center shift), False = Option B (online 8-servo sin/cos oval CPG gait)
gait_pattern = "crawl" # "crawl" (4-beat 0 / 0.25 / 0.5 / 0.75) or "trot" (diagonal pairs 0 / 0.5)
run_mode = "walk" # what to do when running this file directly : stand / direction_test / shoulder_test / walk / backward / turn_left / turn_right
knees_energized = False # False = lower leg motors are never powered, True = they hold KNEE_HOLD_ANGLES
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

SHOULDERS = [FL_MOTOR_1, FR_MOTOR_1, BL_MOTOR_1, BR_MOTOR_1] # shoulder motors
THIGHS = [FL_MOTOR_2, FR_MOTOR_2, BL_MOTOR_2, BR_MOTOR_2] # middle leg motors (thighs)
KNEES = [FL_MOTOR_3, FR_MOTOR_3, BL_MOTOR_3, BR_MOTOR_3] # lower leg motors (knees)

# mapping from thigh motor index to its matching shoulder motor index
THIGH_TO_SHOULDER = {
    FL_MOTOR_2: FL_MOTOR_1,
    FR_MOTOR_2: FR_MOTOR_1,
    BL_MOTOR_2: BL_MOTOR_1,
    BR_MOTOR_2: BR_MOTOR_1,
}

THIGH_NAMES = {FL_MOTOR_2: "front left", FR_MOTOR_2: "front right", BL_MOTOR_2: "back left", BR_MOTOR_2: "back right"}
SHOULDER_NAMES = {FL_MOTOR_1: "front left", FR_MOTOR_1: "front right", BL_MOTOR_1: "back left", BR_MOTOR_1: "back right"}
THIGH_SIDE = {FL_MOTOR_2: "left", FR_MOTOR_2: "right", BL_MOTOR_2: "left", BR_MOTOR_2: "right"}

# reduced mode angles found by hand
REDUCED_SHOULDER_ANGLES = {FL_MOTOR_1: 110, FR_MOTOR_1: 110, BL_MOTOR_1: 130, BR_MOTOR_1: 80} # shoulders while standing on the folded legs
REDUCED_THIGH_STAND = {FL_MOTOR_2: 78, FR_MOTOR_2: 100, BL_MOTOR_2: 87, BR_MOTOR_2: 89} # thigh angles where the robot sits level
THIGH_MIN = {FL_MOTOR_2: 0, FR_MOTOR_2: 0, BL_MOTOR_2: 50, BR_MOTOR_2: 0} # lowest angle each thigh may be sent while moving
THIGH_MAX = {FL_MOTOR_2: 150, FR_MOTOR_2: 180, BL_MOTOR_2: 180, BR_MOTOR_2: 180} # highest angle each thigh may be sent while moving
SHOULDER_MIN = {FL_MOTOR_1: 60, FR_MOTOR_1: 60, BL_MOTOR_1: 80, BR_MOTOR_1: 40} # safe lower limit for shoulder assist
SHOULDER_MAX = {FL_MOTOR_1: 160, FR_MOTOR_1: 160, BL_MOTOR_1: 170, BR_MOTOR_1: 140} # safe upper limit for shoulder assist
KNEE_COLLAPSED = {FL_MOTOR_3: 180, FR_MOTOR_3: 0, BL_MOTOR_3: 180, BR_MOTOR_3: 0} # collapsed angles
KNEE_HOLD_ANGLES = {FL_MOTOR_3: 175, FR_MOTOR_3: 5, BL_MOTOR_3: 175, BR_MOTOR_3: 5} # 5 deg short of the stops if knees_energized is True

# rest pose, the pose the robot starts in and returns to
REST_SHOULDER_ANGLES = {FL_MOTOR_1: 110, FR_MOTOR_1: 110, BL_MOTOR_1: 130, BR_MOTOR_1: 110}
REST_THIGH_ANGLES = {FL_MOTOR_2: 180, FR_MOTOR_2: 0, BL_MOTOR_2: 180, BR_MOTOR_2: 0}

# 1 = raising the angle moves the foot forward, -1 = lowering it does
THIGH_FORWARD_SIGN = {FL_MOTOR_2: 1, FR_MOTOR_2: -1, BL_MOTOR_2: 1, BR_MOTOR_2: -1}

# 1 = raising the shoulder angle lifts the folded leg outward/up, -1 = lowering it does
SHOULDER_LIFT_SIGN = {FL_MOTOR_1: -1, FR_MOTOR_1: 1, BL_MOTOR_1: -1, BR_MOTOR_1: 1}

# gait settings
# Why swing_bias = 16.0 fixes the backward walk:
# At REDUCED_THIGH_STAND (78/100/87/89), the legs sit behind vertical (90 deg), so swinging a foot forward
# moves it toward 90 deg (making it taller so it wedges into the table and pushes the body backward).
# Shifting the walking center +16 deg forward puts all 4 legs uniformly in front of 90 deg (94/84/103/73),
# so swinging a foot BACKWARD moves it toward 90 deg (wedges into the table and pushes the body FORWARD).
swing_amplitude = 10.0 # degrees each thigh swings either side of its walking center
swing_bias = 16.0 # degrees the walking center is shifted forward of REDUCED_THIGH_STAND (puts legs in front of 90 deg)
step_time = 2.2 # seconds for one full cycle of one leg
power_stroke_fraction = 0.75 # 0.75 = 3 legs in slow stroke while 1 leg is in fast stroke (Option A)
walk_step_count = 4 # default full cycles to run in walk, backward and turn functions
test_swing = 8.0 # degrees used by direction_test
wave_time_sign = 1 # 1 = standard phase progression, -1 = reversed phase progression (press 'r' in WASD to flip)

# Option B (strictly_thighs = False) online 8-servo sin/cos CPG oval gait settings
# Thigh sweeps with -cos(phase) and shoulder lifts with +sin(phase), tracing a closed 2D oval path
shoulder_lift_angle = 10.0 # degrees amplitude for the sin(phase) shoulder lift/press cycle

# phase offset tables from standard 4-servo / 8-servo CPG quadruped controllers
CRAWL_PHASE = {FL_MOTOR_2: 0.0, BR_MOTOR_2: 0.25, FR_MOTOR_2: 0.5, BL_MOTOR_2: 0.75}
TROT_PHASE = {FL_MOTOR_2: 0.0, BR_MOTOR_2: 0.0, FR_MOTOR_2: 0.5, BL_MOTOR_2: 0.5}

# movement timing
loop_dt = 0.02 # seconds between servo updates (50 Hz)
shoulder_move_time = 1.0 # seconds to move the shoulders
stand_move_time = 2.5 # seconds for each pair of thighs to stand up
rest_move_time = 2.5 # seconds for each pair of thighs to go back to rest
return_move_time = 1.2 # seconds to ease back to the stand after walking
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

# fn to get the active phase offset dict for the current gait_pattern
def get_leg_phases():
    if gait_pattern == "trot":
        return TROT_PHASE
    return CRAWL_PHASE

# fn to toggle between "crawl" and "trot" gait patterns
def toggle_gait_pattern():
    global gait_pattern
    gait_pattern = "trot" if gait_pattern == "crawl" else "crawl"
    print("Switched gait_pattern to : " + gait_pattern)

# fn to switch between Option A (strictly thighs) and Option B (8-servo sin/cos oval CPG)
def set_strictly_thighs(enabled):
    global strictly_thighs
    strictly_thighs = bool(enabled)
    mode_label = "Option A (strictly thighs only)" if strictly_thighs else "Option B (8-servo sin/cos oval CPG)"
    print("Switched foot clearance mode to : " + mode_label)
    if strictly_thighs and robot_standing:
        move_servos_smooth(REDUCED_SHOULDER_ANGLES, 0.4)

# fn to toggle whether the knee servos are energized (held 5 deg short of the stops) or unpowered
def set_knees_energized(enabled):
    global knees_energized
    knees_energized = bool(enabled)
    if knees_energized:
        for index in KNEES:
            servo_driver_imp.set_servo_range(index, 180)
            time.sleep(0.02)
    prepare_knees()

# fn to flip wave_time_sign (+1 <-> -1) at runtime
def invert_wave_time_sign():
    global wave_time_sign
    wave_time_sign = -wave_time_sign
    print("Flipped wave_time_sign -> " + str(wave_time_sign))

# fn to flip THIGH_FORWARD_SIGN for all 4 thighs at runtime
def invert_thigh_forward_signs():
    for index in THIGHS:
        THIGH_FORWARD_SIGN[index] = -THIGH_FORWARD_SIGN[index]
    print("Flipped THIGH_FORWARD_SIGN -> " + str(THIGH_FORWARD_SIGN))

# fn to invert SHOULDER_LIFT_SIGN for all 4 shoulders at runtime
def invert_shoulder_lift_signs():
    for index in SHOULDERS:
        SHOULDER_LIFT_SIGN[index] = -SHOULDER_LIFT_SIGN[index]
    print("Inverted SHOULDER_LIFT_SIGN -> " + str(SHOULDER_LIFT_SIGN))

# fn to write one servo directly through the driver kit
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
def move_servos_smooth(targets, duration):
    if duration <= 0:
        for index, target in targets.items():
            write_servo(index, target)
        return
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
        print("Holding the lower leg motors at KNEE_HOLD_ANGLES (5 deg short of stops)...")
        for index in KNEES:
            write_servo(index, KNEE_HOLD_ANGLES[index])
    else:
        print("Releasing the lower leg motors, they stay unpowered in reduced mode...")
        for index in KNEES:
            release_servo(index)

# fn to stand up on the folded legs
def stand_up_reduced():
    global robot_standing, servos_used
    print("Running stand up in reduced mode...")
    servos_used = True

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

# fn to initialise ranges, prepare knees and stand the robot up (call this first when importing)
def setup_robot():
    global servos_used
    if not reduced_mode:
        print("reduced_mode is False - normal mode is not added to this script yet, nothing will move.")
        return False
    if servo_driver_imp.kit is None:
        print("Servo driver is not ready, nothing will move.")
        return False
    servos_used = True
    init_servo_ranges()
    prepare_knees()
    stand_up_reduced()
    return True

# fn to ease the thighs and shoulders back to the reduced stand angles
def return_to_stand():
    print("Returning to the reduced stand...")
    targets = {}
    for index, angle in REDUCED_THIGH_STAND.items():
        targets[index] = angle
    for index, angle in REDUCED_SHOULDER_ANGLES.items():
        targets[index] = angle
    move_servos_smooth(targets, return_move_time)

# fn to go back to the rest pose
def rest_down():
    global robot_standing
    print("Running rest down...")

    # make sure shoulders are at reduced stand before folding thighs
    move_servos_smooth(REDUCED_SHOULDER_ANGLES, 0.5)

    # back thighs first
    move_servos_smooth({BL_MOTOR_2: REST_THIGH_ANGLES[BL_MOTOR_2], BR_MOTOR_2: REST_THIGH_ANGLES[BR_MOTOR_2]}, rest_move_time)
    time.sleep(0.5)

    # front thighs
    move_servos_smooth({FL_MOTOR_2: REST_THIGH_ANGLES[FL_MOTOR_2], FR_MOTOR_2: REST_THIGH_ANGLES[FR_MOTOR_2]}, rest_move_time)
    time.sleep(0.5)

    move_servos_smooth(REST_SHOULDER_ANGLES, shoulder_move_time)
    robot_standing = False
    print("Finished resting.")

# fn to safely fold the legs and release all servos on exit (call this in finally when importing)
def shutdown_robot():
    global servos_used
    print("Shutting down robot safely...")
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
        servos_used = False
    print("Robot shutdown complete.")

# fn to swing each thigh a little toward its forward direction, one at a time
def direction_test():
    print("Running direction test, watch each foot...")
    for index in THIGHS:
        stand_angle = REDUCED_THIGH_STAND[index]
        forward_angle = clamp_value(stand_angle + THIGH_FORWARD_SIGN[index] * test_swing, THIGH_MIN[index], THIGH_MAX[index])
        print("Thigh " + str(index) + " (" + THIGH_NAMES[index] + ") moving to " + str(forward_angle) + ", the foot should move FORWARD")
        move_servos_smooth({index: forward_angle}, 1.5)
        time.sleep(1.0)
        move_servos_smooth({index: stand_angle}, 1.5)
        time.sleep(0.5)
    print("Direction test finished.")

# fn to test the shoulder lift direction for Option B (strictly_thighs = False), one leg at a time
def shoulder_lift_test():
    print("Running shoulder lift test for Option B, watch each leg tilt outward...")
    for shoulder_index in SHOULDERS:
        stand_angle = REDUCED_SHOULDER_ANGLES[shoulder_index]
        lifted_angle = clamp_value(
            stand_angle + SHOULDER_LIFT_SIGN[shoulder_index] * shoulder_lift_angle,
            SHOULDER_MIN[shoulder_index],
            SHOULDER_MAX[shoulder_index],
        )
        print("Shoulder " + str(shoulder_index) + " (" + SHOULDER_NAMES[shoulder_index] + ") moving to " + str(lifted_angle) + ", leg should tilt OUTWARD / UP")
        move_servos_smooth({shoulder_index: lifted_angle}, 1.2)
        time.sleep(1.0)
        move_servos_smooth({shoulder_index: stand_angle}, 1.2)
        time.sleep(0.5)
    print("If a leg tilted inward instead of outward, press 'i' in WASD or flip SHOULDER_LIFT_SIGN.")

# fn for the shape of one leg cycle in Option A
def gait_wave(phase):
    phase = phase % 1.0
    if phase < power_stroke_fraction:
        t = phase / power_stroke_fraction
        return 1.0 - 2.0 * smooth(t)
    return_t = (phase - power_stroke_fraction) / (1.0 - power_stroke_fraction)
    return -1.0 + 2.0 * smooth(return_t)

# fn to compute and write one single frame of the walking gait
# left_direction and right_direction : 1 = move that side forward, -1 = move that side backward
def update_gait_step(elapsed, left_direction, right_direction, ramp=1.0):
    phases = get_leg_phases()

    for index in THIGHS:
        direction = left_direction if THIGH_SIDE[index] == "left" else right_direction

        if strictly_thighs:
            # Option A : 4-servo phase-shifted wave centered at REDUCED_THIGH_STAND + swing_bias
            # With swing_bias = +16 deg, all 4 legs oscillate in front of 90 deg so the backward stroke wedges and pushes forward
            phase = wave_time_sign * direction * (elapsed / step_time) + phases[index]
            wave = gait_wave(phase)
            offset = (swing_bias + swing_amplitude * wave) * ramp
            thigh_angle = REDUCED_THIGH_STAND[index] + THIGH_FORWARD_SIGN[index] * offset
            write_servo(index, clamp_value(thigh_angle, THIGH_MIN[index], THIGH_MAX[index]))
        else:
            # Option B : Online 8-servo CPG gait (cos for thigh sweep, sin for shoulder lift/press)
            # When sin(rad) > 0, shoulder lifts leg UP while -cos(rad) moves thigh FORWARD.
            # When sin(rad) < 0, shoulder presses leg DOWN while -cos(rad) moves thigh BACKWARD.
            rad = ((wave_time_sign * elapsed / step_time) + phases[index]) * 2.0 * math.pi
            thigh_offset = (swing_bias - direction * swing_amplitude * math.cos(rad)) * ramp
            thigh_angle = REDUCED_THIGH_STAND[index] + THIGH_FORWARD_SIGN[index] * thigh_offset
            write_servo(index, clamp_value(thigh_angle, THIGH_MIN[index], THIGH_MAX[index]))

            shoulder_index = THIGH_TO_SHOULDER[index]
            shoulder_offset = SHOULDER_LIFT_SIGN[shoulder_index] * shoulder_lift_angle * math.sin(rad) * ramp
            shoulder_angle = REDUCED_SHOULDER_ANGLES[shoulder_index] + shoulder_offset
            write_servo(shoulder_index, clamp_value(shoulder_angle, SHOULDER_MIN[shoulder_index], SHOULDER_MAX[shoulder_index]))

# fn to check whether current swing settings exceed thigh limits
def check_swing_limits():
    for index in THIGHS:
        low = REDUCED_THIGH_STAND[index] + THIGH_FORWARD_SIGN[index] * (swing_bias - swing_amplitude)
        high = REDUCED_THIGH_STAND[index] + THIGH_FORWARD_SIGN[index] * (swing_bias + swing_amplitude)
        min_reach = min(low, high)
        max_reach = max(low, high)
        if min_reach < THIGH_MIN[index] or max_reach > THIGH_MAX[index]:
            print("Warning : thigh " + str(index) + " (" + THIGH_NAMES[index] + ") will be clamped by its limit, lower swing_amplitude or swing_bias")

# fn to run the gait for a given number of cycles
# left_direction and right_direction : 1 = push the body forward, -1 = push it backward
def run_gait(cycles, left_direction, right_direction):
    mode_name = "Option A (strictly thighs)" if strictly_thighs else "Option B (8-servo sin/cos oval CPG)"
    print("Running gait (" + gait_pattern + ") for " + str(cycles) + " cycles using " + mode_name + "...")
    check_swing_limits()

    total_time = cycles * step_time
    start_time = time.time()
    last_log = 0.0
    while True:
        elapsed = time.time() - start_time
        if elapsed >= total_time:
            break

        ramp = smooth(min(1.0, elapsed / (step_time * 0.5))) # eases the swing in over the first half cycle
        update_gait_step(elapsed, left_direction, right_direction, ramp)

        if log_tilt and elapsed - last_log >= tilt_log_interval:
            last_log = elapsed
            tilt = gyro_imp.get_pitch_roll()
            print("Tilt > " + str(tilt))

        time.sleep(loop_dt)

    return_to_stand()
    print("Gait finished.")
    rgb_led_imp.blink_oke()

# fn to walk forward for a number of cycles
def walk_forward(cycles=None):
    if cycles is None:
        cycles = walk_step_count
    print("Walk forward (" + str(cycles) + " cycles)")
    run_gait(cycles, 1, 1)

# fn to walk backward for a number of cycles
def walk_backward(cycles=None):
    if cycles is None:
        cycles = walk_step_count
    print("Walk backward (" + str(cycles) + " cycles)")
    run_gait(cycles, -1, -1)

# fn to turn left, right legs push forward and left legs push backward
def turn_left(cycles=None):
    if cycles is None:
        cycles = walk_step_count
    print("Turn left (" + str(cycles) + " cycles)")
    run_gait(cycles, -1, 1)

# fn to turn right, left legs push forward and right legs push backward
def turn_right(cycles=None):
    if cycles is None:
        cycles = walk_step_count
    print("Turn right (" + str(cycles) + " cycles)")
    run_gait(cycles, 1, -1)

# fn to run the script directly
def main():
    try:
        print("Reduced walking script started...")
        input("Put the robot in the rest pose, then press Enter to start > ")

        if not setup_robot():
            return

        if run_mode == "direction_test":
            direction_test()
        elif run_mode == "shoulder_test":
            shoulder_lift_test()
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
        shutdown_robot()
        print("Finished")

if __name__ == "__main__":
    main()
