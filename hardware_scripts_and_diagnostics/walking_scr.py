# imports
import servo_driver_imp
import rgb_led_imp
import gyro_imp
import time
import threading
import sys
import tty
import termios
import select

# define servo indexes
CAM_X = 0
CAM_Y = 1

FL_MOTOR_1 = 2  # front left shoulder
FL_MOTOR_2 = 3  # front left upper leg
FL_MOTOR_3 = 4  # front left lower leg

FR_MOTOR_1 = 5  # front right shoulder
FR_MOTOR_2 = 6  # front right upper leg
FR_MOTOR_3 = 7  # front right lower leg

BL_MOTOR_1 = 8  # back left shoulder
BL_MOTOR_2 = 9  # back left upper leg
BL_MOTOR_3 = 10 # back left lower leg

BR_MOTOR_1 = 11 # back right shoulder
BR_MOTOR_2 = 12 # back right upper leg
BR_MOTOR_3 = 13 # back right lower leg

# define stand angles - first calibration
STAND_ANGLES = {
    CAM_X:      90,
    CAM_Y:      90,
    FL_MOTOR_1: 110,
    FL_MOTOR_2: 50,
    FL_MOTOR_3: 90,
    FR_MOTOR_1: 110,
    FR_MOTOR_2: 130,
    FR_MOTOR_3: 90,
    BL_MOTOR_1: 130,
    BL_MOTOR_2: 50,
    BL_MOTOR_3: 90,
    BR_MOTOR_1: 80,
    BR_MOTOR_2: 130,
    BR_MOTOR_3: 90,
}

# define stand angles - second calibration, better stability
BETTER_STAND_ANGLES = {
    CAM_X:      90,
    CAM_Y:      90,
    FL_MOTOR_1: 110,
    FL_MOTOR_2: 70,
    FL_MOTOR_3: 50,
    FR_MOTOR_1: 110,
    FR_MOTOR_2: 110,
    FR_MOTOR_3: 150,
    BL_MOTOR_1: 130,
    BL_MOTOR_2: 70,
    BL_MOTOR_3: 40,
    BR_MOTOR_1: 90,
    BR_MOTOR_2: 90,
    BR_MOTOR_3: 160,
}

# define resting angles
resting_angles = {
    CAM_X:      90,
    CAM_Y:      90,
    FL_MOTOR_1: 110,
    FL_MOTOR_2: 180,
    FL_MOTOR_3: 180,
    FR_MOTOR_1: 110,
    FR_MOTOR_2: 0,
    FR_MOTOR_3: 0,
    BL_MOTOR_1: 130,
    BL_MOTOR_2: 180,
    BL_MOTOR_3: 180,
    BR_MOTOR_1: 110,
    BR_MOTOR_2: 0,
    BR_MOTOR_3: 0,
}

# set to True to use the better standing angles
better_standing = True

# gyro offset calibration
GYRO_PITCH_OFFSET = 0.0
GYRO_ROLL_OFFSET = 0.0

GYRO_CALIBRATION_SAMPLES = 30
GYRO_CALIBRATION_INTERVAL = 0.1

# define leveling vars
level_gain = 1.2
level_deadband = 1.0
level_max_correction_shoulders = 15.0
level_max_correction_legs = 20.0
level_interval = 0.15

roll_gain = level_gain
pitch_gain = level_gain * 1.5

leg_roll_gain = level_gain * 0.6
leg_pitch_gain = level_gain * 0.6

full_leveling = False

# flag to control the leveling thread
leveling_active = False

# walking settings
walking_active = False

STEP_FORWARD = 12
STEP_BACKWARD = 12
STEP_LIFT = 15

WALK_MOVE_TIME = 0.25
WALK_STEP_DELAY = 0.08

# turning settings
TURN_STEP = 10

# fn to get active stand angles
def get_active_stand_angles():
    if better_standing:
        return BETTER_STAND_ANGLES

    return STAND_ANGLES

# fn to get a key without pressing enter
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)

        if select.select([sys.stdin], [], [], 0.05)[0]:
            return sys.stdin.read(1)

        return None

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# fn to init all servo ranges
def init_servo_ranges():
    print("Initialising servo ranges...")

    for index in get_active_stand_angles().keys():
        servo_driver_imp.set_servo_range(index, 180)
        time.sleep(0.05)

    print("All servo ranges set.")

# helper fn to clamp a value
def clamp_value(value, min_val, max_val):
    return max(min_val, min(max_val, value))

# fn to calibrate gyro level position
def calibrate_level_offset():
    global GYRO_PITCH_OFFSET
    global GYRO_ROLL_OFFSET

    print("Calibrating gyro level position...")
    print("Keep the robot completely still.")

    pitch_total = 0.0
    roll_total = 0.0
    valid_samples = 0

    for i in range(GYRO_CALIBRATION_SAMPLES):
        data = gyro_imp.get_pitch_roll()

        if data is not None:
            pitch_total += data["pitch"]
            roll_total += data["roll"]
            valid_samples += 1

            print(
                "Calibration "
                + str(i + 1)
                + "/"
                + str(GYRO_CALIBRATION_SAMPLES)
                + " > pitch: "
                + str(round(data["pitch"], 2))
                + ", roll: "
                + str(round(data["roll"], 2))
            )
        else:
            print("Could not read gyro during calibration.")

        time.sleep(GYRO_CALIBRATION_INTERVAL)

    if valid_samples == 0:
        print("Gyro calibration failed - no valid readings.")
        return False

    GYRO_PITCH_OFFSET = pitch_total / valid_samples
    GYRO_ROLL_OFFSET = roll_total / valid_samples

    print("Gyro calibration complete.")
    print("Pitch offset: " + str(round(GYRO_PITCH_OFFSET, 3)))
    print("Roll offset: " + str(round(GYRO_ROLL_OFFSET, 3)))

    return True

# fn to apply shoulder correction
def apply_shoulder_correction(roll_corr, pitch_corr):
    angles = get_active_stand_angles()

    fl_new_angle = clamp_value(
        angles[FL_MOTOR_1] - roll_corr + pitch_corr,
        angles[FL_MOTOR_1] - level_max_correction_shoulders,
        angles[FL_MOTOR_1] + level_max_correction_shoulders
    )

    fr_new_angle = clamp_value(
        angles[FR_MOTOR_1] + roll_corr + pitch_corr,
        angles[FR_MOTOR_1] - level_max_correction_shoulders,
        angles[FR_MOTOR_1] + level_max_correction_shoulders
    )

    bl_new_angle = clamp_value(
        angles[BL_MOTOR_1] - roll_corr - pitch_corr,
        angles[BL_MOTOR_1] - level_max_correction_shoulders,
        angles[BL_MOTOR_1] + level_max_correction_shoulders
    )

    br_new_angle = clamp_value(
        angles[BR_MOTOR_1] + roll_corr - pitch_corr,
        angles[BR_MOTOR_1] - level_max_correction_shoulders,
        angles[BR_MOTOR_1] + level_max_correction_shoulders
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_1,
        int(fl_new_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_1,
        int(fr_new_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_1,
        int(bl_new_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_1,
        int(br_new_angle)
    )

# fn to apply upper leg correction
def apply_leg_correction(roll_corr, pitch_corr):
    angles = get_active_stand_angles()

    fl_leg_angle = clamp_value(
        angles[FL_MOTOR_2] - roll_corr + pitch_corr,
        angles[FL_MOTOR_2] - level_max_correction_legs,
        angles[FL_MOTOR_2] + level_max_correction_legs
    )

    bl_leg_angle = clamp_value(
        angles[BL_MOTOR_2] - roll_corr - pitch_corr,
        angles[BL_MOTOR_2] - level_max_correction_legs,
        angles[BL_MOTOR_2] + level_max_correction_legs
    )

    fr_leg_angle = clamp_value(
        angles[FR_MOTOR_2] + roll_corr + pitch_corr,
        angles[FR_MOTOR_2] - level_max_correction_legs,
        angles[FR_MOTOR_2] + level_max_correction_legs
    )

    br_leg_angle = clamp_value(
        angles[BR_MOTOR_2] + roll_corr - pitch_corr,
        angles[BR_MOTOR_2] - level_max_correction_legs,
        angles[BR_MOTOR_2] + level_max_correction_legs
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_2,
        int(fl_leg_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_2,
        int(fr_leg_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_2,
        int(bl_leg_angle)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_2,
        int(br_leg_angle)
    )

# fn to read gyro and apply correction
def apply_level_correction():
    if walking_active:
        return

    data = gyro_imp.get_pitch_roll()

    if data is None:
        print("Could not read gyro, skipping correction...")
        return

    pitch = data["pitch"] - GYRO_PITCH_OFFSET
    roll = data["roll"] - GYRO_ROLL_OFFSET

    effective_roll = roll if abs(roll) > level_deadband else 0.0
    effective_pitch = pitch if abs(pitch) > level_deadband else 0.0

    if effective_roll == 0.0 and effective_pitch == 0.0:
        return

    shoulder_roll_corr = roll_gain * effective_roll
    shoulder_pitch_corr = pitch_gain * effective_pitch

    apply_shoulder_correction(
        shoulder_roll_corr,
        shoulder_pitch_corr
    )

    if full_leveling:
        leg_roll_corr = leg_roll_gain * effective_roll
        leg_pitch_corr = leg_pitch_gain * effective_pitch

        apply_leg_correction(
            leg_roll_corr,
            leg_pitch_corr
        )

# fn that runs leveling in background
def leveling_thread_fn():
    print("Leveling thread started...")

    while leveling_active:
        apply_level_correction()
        time.sleep(level_interval)

    print("Leveling thread stopped.")

# fn to start leveling
def start_leveling():
    global leveling_active

    leveling_active = True

    t = threading.Thread(
        target=leveling_thread_fn,
        daemon=True
    )

    t.start()

    if full_leveling:
        print("Continuous full leveling started.")
    else:
        print("Continuous shoulder leveling started.")

    rgb_led_imp.blink_warning()

    return t

# fn to stop leveling
def stop_leveling():
    global leveling_active

    leveling_active = False

# fn to set the sholders
def set_sholders():
    print("Running set sholders fn...")

    angles = get_active_stand_angles()

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_1,
        angles[FL_MOTOR_1]
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_1,
        angles[FR_MOTOR_1]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_1,
        angles[BL_MOTOR_1]
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_1,
        angles[BR_MOTOR_1]
    )

    print("Finished setting sholders.")
    rgb_led_imp.blink_oke()

# fn to set main legs
def set_main_legs():
    print("Running setting main legs..")

    angles = get_active_stand_angles()

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_2,
        angles[FL_MOTOR_2]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_2,
        angles[FR_MOTOR_2]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_2,
        angles[BL_MOTOR_2]
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_2,
        angles[BR_MOTOR_2]
    )

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to set lower legs
def set_lower_legs():
    print("Running setting lower legs..")

    angles = get_active_stand_angles()

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_3,
        angles[FL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_3,
        angles[FR_MOTOR_3]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_3,
        angles[BL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_3,
        angles[BR_MOTOR_3]
    )

    print("Finished setting lower legs.")
    rgb_led_imp.blink_oke()

# fn to set one leg position
def set_leg_position(motor_2, motor_3, angle_2, angle_3):
    servo_driver_imp.driver_set_servo_angle_with_index(
        motor_2,
        int(angle_2)
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        motor_3,
        int(angle_3)
    )

# fn to move one leg smoothly
def move_leg_smooth(
    motor_2,
    motor_3,
    start_2,
    start_3,
    end_2,
    end_3
):
    steps = max(1, int(WALK_MOVE_TIME / 0.03))

    for i in range(steps + 1):
        t = i / steps

        angle_2 = start_2 + ((end_2 - start_2) * t)
        angle_3 = start_3 + ((end_3 - start_3) * t)

        servo_driver_imp.driver_set_servo_angle_with_index(
            motor_2,
            int(angle_2)
        )

        servo_driver_imp.driver_set_servo_angle_with_index(
            motor_3,
            int(angle_3)
        )

        time.sleep(WALK_MOVE_TIME / steps)

# fn to return all legs to standing position
def return_to_stand():
    angles = get_active_stand_angles()

    print("Returning to standing position...")

    for motor in [
        FL_MOTOR_2,
        FR_MOTOR_2,
        BL_MOTOR_2,
        BR_MOTOR_2
    ]:
        servo_driver_imp.driver_set_servo_angle_with_index(
            motor,
            angles[motor]
        )

    for motor in [
        FL_MOTOR_3,
        FR_MOTOR_3,
        BL_MOTOR_3,
        BR_MOTOR_3
    ]:
        servo_driver_imp.driver_set_servo_angle_with_index(
            motor,
            angles[motor]
        )

    time.sleep(0.5)

# fn to lift diagonal pair
def lift_diagonal_pair(pair):
    angles = get_active_stand_angles()

    if pair == 1:
        # FL + BR
        set_leg_position(
            FL_MOTOR_2,
            FL_MOTOR_3,
            angles[FL_MOTOR_2],
            angles[FL_MOTOR_3] + STEP_LIFT
        )

        set_leg_position(
            BR_MOTOR_2,
            BR_MOTOR_3,
            angles[BR_MOTOR_2],
            angles[BR_MOTOR_3] - STEP_LIFT
        )

    else:
        # FR + BL
        set_leg_position(
            FR_MOTOR_2,
            FR_MOTOR_3,
            angles[FR_MOTOR_2],
            angles[FR_MOTOR_3] - STEP_LIFT
        )

        set_leg_position(
            BL_MOTOR_2,
            BL_MOTOR_3,
            angles[BL_MOTOR_2],
            angles[BL_MOTOR_3] + STEP_LIFT
        )

    time.sleep(WALK_STEP_DELAY)

# fn to lower diagonal pair
def lower_diagonal_pair(pair):
    angles = get_active_stand_angles()

    if pair == 1:
        set_leg_position(
            FL_MOTOR_2,
            FL_MOTOR_3,
            angles[FL_MOTOR_2],
            angles[FL_MOTOR_3]
        )

        set_leg_position(
            BR_MOTOR_2,
            BR_MOTOR_3,
            angles[BR_MOTOR_2],
            angles[BR_MOTOR_3]
        )

    else:
        set_leg_position(
            FR_MOTOR_2,
            FR_MOTOR_3,
            angles[FR_MOTOR_2],
            angles[FR_MOTOR_3]
        )

        set_leg_position(
            BL_MOTOR_2,
            BL_MOTOR_3,
            angles[BL_MOTOR_2],
            angles[BL_MOTOR_3]
        )

    time.sleep(WALK_STEP_DELAY)

# fn to move diagonal pair forward
def move_pair_forward(pair, amount):
    angles = get_active_stand_angles()

    if pair == 1:
        move_leg_smooth(
            FL_MOTOR_2,
            FL_MOTOR_3,
            angles[FL_MOTOR_2],
            angles[FL_MOTOR_3] + STEP_LIFT,
            angles[FL_MOTOR_2] + amount,
            angles[FL_MOTOR_3] + STEP_LIFT
        )

        move_leg_smooth(
            BR_MOTOR_2,
            BR_MOTOR_3,
            angles[BR_MOTOR_2],
            angles[BR_MOTOR_3] - STEP_LIFT,
            angles[BR_MOTOR_2] - amount,
            angles[BR_MOTOR_3] - STEP_LIFT
        )

    else:
        move_leg_smooth(
            FR_MOTOR_2,
            FR_MOTOR_3,
            angles[FR_MOTOR_2],
            angles[FR_MOTOR_3] - STEP_LIFT,
            angles[FR_MOTOR_2] - amount,
            angles[FR_MOTOR_3] - STEP_LIFT
        )

        move_leg_smooth(
            BL_MOTOR_2,
            BL_MOTOR_3,
            angles[BL_MOTOR_2],
            angles[BL_MOTOR_3] + STEP_LIFT,
            angles[BL_MOTOR_2] + amount,
            angles[BL_MOTOR_3] + STEP_LIFT
        )

# fn to move diagonal pair backward
def move_pair_backward(pair, amount):
    angles = get_active_stand_angles()

    if pair == 1:
        move_leg_smooth(
            FL_MOTOR_2,
            FL_MOTOR_3,
            angles[FL_MOTOR_2],
            angles[FL_MOTOR_3] + STEP_LIFT,
            angles[FL_MOTOR_2] - amount,
            angles[FL_MOTOR_3] + STEP_LIFT
        )

        move_leg_smooth(
            BR_MOTOR_2,
            BR_MOTOR_3,
            angles[BR_MOTOR_2],
            angles[BR_MOTOR_3] - STEP_LIFT,
            angles[BR_MOTOR_2] + amount,
            angles[BR_MOTOR_3] - STEP_LIFT
        )

    else:
        move_leg_smooth(
            FR_MOTOR_2,
            FR_MOTOR_3,
            angles[FR_MOTOR_2],
            angles[FR_MOTOR_3] - STEP_LIFT,
            angles[FR_MOTOR_2] + amount,
            angles[FR_MOTOR_3] - STEP_LIFT
        )

        move_leg_smooth(
            BL_MOTOR_2,
            BL_MOTOR_3,
            angles[BL_MOTOR_2],
            angles[BL_MOTOR_3] + STEP_LIFT,
            angles[BL_MOTOR_2] - amount,
            angles[BL_MOTOR_3] + STEP_LIFT
        )

# fn to walk forward one cycle
def walk_forward():
    global walking_active

    walking_active = True

    print("Walking forward...")

    # FL + BR lift
    lift_diagonal_pair(1)

    # move lifted pair forward
    move_pair_forward(1, STEP_FORWARD)

    # lower FL + BR
    lower_diagonal_pair(1)

    # move support pair backward
    move_pair_backward(2, STEP_FORWARD)

    # FR + BL lift
    lift_diagonal_pair(2)

    # move lifted pair forward
    move_pair_forward(2, STEP_FORWARD)

    # lower FR + BL
    lower_diagonal_pair(2)

    # move support pair backward
    move_pair_backward(1, STEP_FORWARD)

    return_to_stand()

    walking_active = False

# fn to walk backward one cycle
def walk_backward():
    global walking_active

    walking_active = True

    print("Walking backward...")

    # FL + BR lift
    lift_diagonal_pair(1)

    # move lifted pair backward
    move_pair_backward(1, STEP_BACKWARD)

    # lower FL + BR
    lower_diagonal_pair(1)

    # move support pair forward
    move_pair_forward(2, STEP_BACKWARD)

    # FR + BL lift
    lift_diagonal_pair(2)

    # move lifted pair backward
    move_pair_backward(2, STEP_BACKWARD)

    # lower FR + BL
    lower_diagonal_pair(2)

    # move support pair forward
    move_pair_forward(1, STEP_BACKWARD)

    return_to_stand()

    walking_active = False

# fn to turn left one cycle
def turn_left():
    global walking_active

    walking_active = True

    angles = get_active_stand_angles()

    print("Turning left...")

    # FL + BR lift
    lift_diagonal_pair(1)

    # FL backward
    move_leg_smooth(
        FL_MOTOR_2,
        FL_MOTOR_3,
        angles[FL_MOTOR_2],
        angles[FL_MOTOR_3] + STEP_LIFT,
        angles[FL_MOTOR_2] - TURN_STEP,
        angles[FL_MOTOR_3] + STEP_LIFT
    )

    # BR backward
    move_leg_smooth(
        BR_MOTOR_2,
        BR_MOTOR_3,
        angles[BR_MOTOR_2],
        angles[BR_MOTOR_3] - STEP_LIFT,
        angles[BR_MOTOR_2] - TURN_STEP,
        angles[BR_MOTOR_3] - STEP_LIFT
    )

    lower_diagonal_pair(1)

    # FR + BL lift
    lift_diagonal_pair(2)

    # FR backward
    move_leg_smooth(
        FR_MOTOR_2,
        FR_MOTOR_3,
        angles[FR_MOTOR_2],
        angles[FR_MOTOR_3] - STEP_LIFT,
        angles[FR_MOTOR_2] - TURN_STEP,
        angles[FR_MOTOR_3] - STEP_LIFT
    )

    # BL backward
    move_leg_smooth(
        BL_MOTOR_2,
        BL_MOTOR_3,
        angles[BL_MOTOR_2],
        angles[BL_MOTOR_3] + STEP_LIFT,
        angles[BL_MOTOR_2] - TURN_STEP,
        angles[BL_MOTOR_3] + STEP_LIFT
    )

    lower_diagonal_pair(2)

    return_to_stand()

    walking_active = False

# fn to turn right one cycle
def turn_right():
    global walking_active

    walking_active = True

    angles = get_active_stand_angles()

    print("Turning right...")

    # FL + BR lift
    lift_diagonal_pair(1)

    # FL forward
    move_leg_smooth(
        FL_MOTOR_2,
        FL_MOTOR_3,
        angles[FL_MOTOR_2],
        angles[FL_MOTOR_3] + STEP_LIFT,
        angles[FL_MOTOR_2] + TURN_STEP,
        angles[FL_MOTOR_3] + STEP_LIFT
    )

    # BR forward
    move_leg_smooth(
        BR_MOTOR_2,
        BR_MOTOR_3,
        angles[BR_MOTOR_2],
        angles[BR_MOTOR_3] - STEP_LIFT,
        angles[BR_MOTOR_2] + TURN_STEP,
        angles[BR_MOTOR_3] - STEP_LIFT
    )

    lower_diagonal_pair(1)

    # FR + BL lift
    lift_diagonal_pair(2)

    # FR forward
    move_leg_smooth(
        FR_MOTOR_2,
        FR_MOTOR_3,
        angles[FR_MOTOR_2],
        angles[FR_MOTOR_3] - STEP_LIFT,
        angles[FR_MOTOR_2] + TURN_STEP,
        angles[FR_MOTOR_3] - STEP_LIFT
    )

    # BL forward
    move_leg_smooth(
        BL_MOTOR_2,
        BL_MOTOR_3,
        angles[BL_MOTOR_2],
        angles[BL_MOTOR_3] + STEP_LIFT,
        angles[BL_MOTOR_2] + TURN_STEP,
        angles[BL_MOTOR_3] + STEP_LIFT
    )

    lower_diagonal_pair(2)

    return_to_stand()

    walking_active = False

# fn to stop walking
def stop_walking():
    global walking_active

    walking_active = False

    print("Stopping walking...")
    return_to_stand()

# fn to rest main legs
def rest_main_legs():
    print("Running rest main legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_2,
        resting_angles[FL_MOTOR_2]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_2,
        resting_angles[FR_MOTOR_2]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_2,
        resting_angles[BL_MOTOR_2]
    )

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_2,
        resting_angles[BR_MOTOR_2]
    )

    print("Finished.")
    rgb_led_imp.blink_oke()

# fn to rest lower legs
def resting_lower_legs():
    print("Running setting lower legs..")

    time.sleep(0.2)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FL_MOTOR_3,
        resting_angles[FL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        FR_MOTOR_3,
        resting_angles[FR_MOTOR_3]
    )

    time.sleep(0.9)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BL_MOTOR_3,
        resting_angles[BL_MOTOR_3]
    )

    time.sleep(0.5)

    servo_driver_imp.driver_set_servo_angle_with_index(
        BR_MOTOR_3,
        resting_angles[BR_MOTOR_3]
    )

    print("Finished resting lower legs.")
    rgb_led_imp.blink_oke()

# loop
run_loop = True
level_thread = None

try:
    init_servo_ranges()
    time.sleep(0.5)

    while run_loop:
        print("[=======[New cycle]=========]")

        if better_standing:
            print("Using better standing angles.")
        else:
            print("Using default standing angles.")

        # stand up
        set_sholders()
        set_main_legs()
        set_lower_legs()

        print("Robot standing, waiting to settle...")
        time.sleep(1.0)

        # calibrate gyro using current standing position
        calibrate_level_offset()

        # start leveling
        level_thread = start_leveling()

        rgb_led_imp.animate_running_process()

        print("")
        print("Robot controls:")
        print("W = walk forward")
        print("S = walk backward")
        print("A = turn left")
        print("D = turn right")
        print("X = stop")
        print("R = return to standing")
        print("Q = quit")
        print("")
        print("Press a key without pressing Enter.")

        while run_loop:
            command = get_key()

            if command is None:
                continue

            command = command.lower()

            if command == "w":
                stop_leveling()
                walk_forward()
                level_thread = start_leveling()

            elif command == "s":
                stop_leveling()
                walk_backward()
                level_thread = start_leveling()

            elif command == "a":
                stop_leveling()
                turn_left()
                level_thread = start_leveling()

            elif command == "d":
                stop_leveling()
                turn_right()
                level_thread = start_leveling()

            elif command == "x":
                stop_walking()
                level_thread = start_leveling()

            elif command == "r":
                stop_leveling()
                return_to_stand()
                calibrate_level_offset()
                level_thread = start_leveling()

            elif command == "q":
                print("Ending script...")
                run_loop = False

except KeyboardInterrupt:
    print("Keyboard interrupt detected - stopping script...")

except Exception as e:
    print("Error happened:", e)

finally:
    print("Script ended - stopping")

    stop_leveling()
    time.sleep(0.5)

    resting_lower_legs()
    rest_main_legs()

    print("Legs resting now release the motors...")
    time.sleep(1)

    servo_driver_imp.cleanup_servos()
    time.sleep(1)

    rgb_led_imp.turn_off_strip()

    print("Finished")
