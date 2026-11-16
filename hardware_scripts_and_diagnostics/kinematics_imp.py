# kinematics module for the robot dog
# handles forward and inverse kinematics for all 4 legs
import math
import servo_driver_imp

# body dimensions in cm
BODY_LENGTH = 26.5
BODY_WIDTH  = 11.0
BODY_HEIGHT = 7.0

# leg segment lengths in cm
UPPER_LEG_LENGTH = 11.5
LOWER_LEG_LENGTH = 14.0

# shoulder joint positions relative to body center
# x = forward, y = left, z = down
SHOULDER_POSITIONS = {
    "fl": (BODY_LENGTH / 2, BODY_WIDTH / 2, 0),
    "fr": (BODY_LENGTH / 2, -BODY_WIDTH / 2, 0),
    "bl": (-BODY_LENGTH / 2, BODY_WIDTH / 2, 0),
    "br": (-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0),
}

# default standing foot position relative to each shoulder
# x = forward/back, y = outward from body, z = down
DEFAULT_FOOT_X = 0.0
DEFAULT_FOOT_Y = 4.0
DEFAULT_FOOT_Z = -20.0

# servo index map
FL_MOTOR_1 = 2
FL_MOTOR_2 = 3
FL_MOTOR_3 = 4
FR_MOTOR_1 = 5
FR_MOTOR_2 = 6
FR_MOTOR_3 = 7
BL_MOTOR_1 = 8
BL_MOTOR_2 = 9
BL_MOTOR_3 = 10
BR_MOTOR_1 = 11
BR_MOTOR_2 = 12
BR_MOTOR_3 = 13

# per leg servo offsets
# left side (fl, bl) : servo = offset + direction * ik_angle
# right side (fr, br) : servo = offset + direction * ik_angle
LEG_OFFSETS = {
    "fl": {"s_off": 99,  "u_off": 111, "l_off": 124, "s_dir": +1, "u_dir": -1, "l_dir": -1},
    "fr": {"s_off": 121, "u_off": 69,  "l_off": 76, "s_dir": -1, "u_dir": +1, "l_dir": +1},
    "bl": {"s_off": 119, "u_off": 111, "l_off": 114, "s_dir": +1, "u_dir": -1, "l_dir": -1},
    "br": {"s_off": 101, "u_off": 49,  "l_off": 86, "s_dir": -1, "u_dir": +1, "l_dir": +1},
}

# fn to solve 3 dof leg ik given target foot position relative to shoulder
# x = forward/back, y = lateral outward, z = vertical
def solve_leg_ik(x, y, z):
    print("Solving IK for x : " + str(x) + ", y : " + str(y) + ", z : " + str(z))

    shoulder_deg = math.degrees(math.atan2(y, -z))

    lateral_dist = math.sqrt(y ** 2 + z ** 2)
    dist = math.sqrt(x ** 2 + lateral_dist ** 2)

    max_reach = UPPER_LEG_LENGTH + LOWER_LEG_LENGTH
    min_reach = abs(UPPER_LEG_LENGTH - LOWER_LEG_LENGTH)

    if dist > max_reach:
        print("Target out of reach, too far : " + str(round(dist, 2)) + " > " + str(max_reach))
        return None

    if dist < min_reach:
        print("Target out of reach, too close : " + str(round(dist, 2)) + " < " + str(min_reach))
        return None

    cos_knee = (UPPER_LEG_LENGTH ** 2 + LOWER_LEG_LENGTH ** 2 - dist ** 2) / (2 * UPPER_LEG_LENGTH * LOWER_LEG_LENGTH)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_angle = math.acos(cos_knee)

    angle_to_foot = math.atan2(x, lateral_dist)

    cos_upper = (UPPER_LEG_LENGTH ** 2 + dist ** 2 - LOWER_LEG_LENGTH ** 2) / (2 * UPPER_LEG_LENGTH * dist)
    cos_upper = max(-1.0, min(1.0, cos_upper))
    upper_offset = math.acos(cos_upper)

    upper_deg = math.degrees(angle_to_foot + upper_offset)
    lower_deg = math.degrees(math.pi - knee_angle)

    print("IK result > shoulder : " + str(round(shoulder_deg, 1)) + ", upper : " + str(round(upper_deg, 1)) + ", lower : " + str(round(lower_deg, 1)))

    return {
        "shoulder_deg": shoulder_deg,
        "upper_deg": upper_deg,
        "lower_deg": lower_deg,
    }

# fn to convert ik geometric angles to servo values
def ik_to_servo_angles(leg, ik_result):
    if leg not in LEG_OFFSETS:
        print("Unknown leg : " + str(leg))
        return None

    o = LEG_OFFSETS[leg]

    shoulder_servo = o["s_off"] + o["s_dir"] * ik_result["shoulder_deg"]
    upper_servo = o["u_off"] + o["u_dir"] * ik_result["upper_deg"]
    lower_servo = o["l_off"] + o["l_dir"] * ik_result["lower_deg"]

    shoulder_servo = max(0, min(180, shoulder_servo))
    upper_servo = max(0, min(180, upper_servo))
    lower_servo = max(0, min(180, lower_servo))

    return {
        "shoulder": int(shoulder_servo),
        "upper": int(upper_servo),
        "lower": int(lower_servo),
    }

# fn to get the shoulder servo angles for the normal standing position
def get_stand_servo_angles():
    ik_result = solve_leg_ik(DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z)

    if ik_result is None:
        print("Could not calculate IK stand angles.")
        return None

    fl = ik_to_servo_angles("fl", ik_result)
    fr = ik_to_servo_angles("fr", ik_result)
    bl = ik_to_servo_angles("bl", ik_result)
    br = ik_to_servo_angles("br", ik_result)

    return {
        FL_MOTOR_1: fl["shoulder"],
        FR_MOTOR_1: fr["shoulder"],
        BL_MOTOR_1: bl["shoulder"],
        BR_MOTOR_1: br["shoulder"],
    }

# fn to move a single leg to a foot position using ik
def move_leg_to(leg, x, y, z):
    print("Moving leg " + leg + " to position...")

    ik_result = solve_leg_ik(x, y, z)

    if ik_result is None:
        print("Could not solve IK for leg " + leg + ", skipping...")
        return False

    servo_angles = ik_to_servo_angles(leg, ik_result)

    if servo_angles is None:
        return False

    print("Servo angles for " + leg + " > shoulder : " + str(servo_angles["shoulder"]) + ", upper : " + str(servo_angles["upper"]) + ", lower : " + str(servo_angles["lower"]))

    index_map = {
        "fl": (FL_MOTOR_1, FL_MOTOR_2, FL_MOTOR_3),
        "fr": (FR_MOTOR_1, FR_MOTOR_2, FR_MOTOR_3),
        "bl": (BL_MOTOR_1, BL_MOTOR_2, BL_MOTOR_3),
        "br": (BR_MOTOR_1, BR_MOTOR_2, BR_MOTOR_3),
    }

    s1, s2, s3 = index_map[leg]

    servo_driver_imp.driver_set_servo_angle_with_index(s1, servo_angles["shoulder"])
    servo_driver_imp.driver_set_servo_angle_with_index(s2, servo_angles["upper"])
    servo_driver_imp.driver_set_servo_angle_with_index(s3, servo_angles["lower"])

    return True

# fn to move all 4 legs to the same foot position
def move_all_legs_to(x, y, z):
    print("Moving all legs to x : " + str(x) + ", y : " + str(y) + ", z : " + str(z))

    move_leg_to("fl", x, y, z)
    move_leg_to("fr", x, y, z)
    move_leg_to("bl", x, y, z)
    move_leg_to("br", x, y, z)

    print("All legs moved.")

# fn to stand using ik
def ik_stand():
    print("Standing using IK...")

    move_all_legs_to(DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z)

    print("IK stand finished.")

# fn to verify ik output matches manual calibration
def verify_ik_calibration():
    print("Verifying IK against manual calibration...")
    print("Expected (from manual calibration):")
    print("fl > shoulder 110, upper 70,  lower 50")
    print("fr > shoulder 110, upper 110, lower 150")
    print("bl > shoulder 130, upper 70,  lower 40")
    print("br > shoulder 90,  upper 90,  lower 160")
    print("")
    print("IK calculated:")

    ik_result = solve_leg_ik(DEFAULT_FOOT_X, DEFAULT_FOOT_Y, DEFAULT_FOOT_Z)

    if ik_result is None:
        print("IK failed for default foot position")
        return

    for leg in ["fl", "fr", "bl", "br"]:
        servo_angles = ik_to_servo_angles(leg, ik_result)

        print(leg + " > shoulder : " + str(servo_angles["shoulder"]) + ", upper : " + str(servo_angles["upper"]) + ", lower : " + str(servo_angles["lower"]))