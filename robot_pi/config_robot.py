# robot_pi/config_robot.py
"""Configuration parameters for the Quadruped Robot (Raspberry Pi 4)."""

import os

# Networking Configuration
LAPTOP_SERVER_IP = os.getenv("PERF_SERVER_IP", "10.80.166.248")
ZMQ_STREAM_PORT = int(os.getenv("PERF_STREAM_PORT", "5555"))   # Pi -> Laptop (Frames, IMU, Servos)
ZMQ_COMMAND_PORT = int(os.getenv("PERF_CMD_PORT", "5556"))     # Laptop -> Pi (Nav commands, Map, Target update)
UDP_DISCOVERY_PORT = int(os.getenv("PERF_DISCOVERY_PORT", "5557"))  # Auto-discovery broadcast port
DCP_WEBSOCKET_PORT = 8765                                      # MachineMake Mobile App

# Camera Configuration
CAMERA_INDEX = int(os.getenv("CAMERA_INDEX", "0"))
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
JPEG_QUALITY = 75

# Camera Pan/Tilt Servos on PCA9685
# X-axis (Pan):  90 = centered, -> 180 = left,  -> 0 = right
# Y-axis (Tilt): 52 = centered, -> 0   = up,    -> 180 = down
CAM_X_INDEX = 0  # Horizontal Pan
CAM_Y_INDEX = 1  # Vertical Tilt
CAM_X_CENTER = 90
CAM_Y_CENTER = 52
CAM_X_MIN = 0
CAM_X_MAX = 180
CAM_Y_MIN = 0
CAM_Y_MAX = 180

# Servos Index Map
FL_MOTOR_1 = 2   # front left shoulder
FL_MOTOR_2 = 3   # front left upper leg
FL_MOTOR_3 = 4   # front left lower leg

FR_MOTOR_1 = 5   # front right shoulder
FR_MOTOR_2 = 6   # front right upper leg
FR_MOTOR_3 = 7   # front right lower leg

BL_MOTOR_1 = 8   # back left shoulder
BL_MOTOR_2 = 9   # back left upper leg
BL_MOTOR_3 = 10  # back left lower leg

BR_MOTOR_1 = 11  # back right shoulder
BR_MOTOR_2 = 12  # back right upper leg
BR_MOTOR_3 = 13  # back right lower leg

# Calibrated Standing Angles (Best stability from calibration)
BETTER_STAND_ANGLES = {
    CAM_X_INDEX: 90,
    CAM_Y_INDEX: 52,
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

# Resting / Sit Angles
RESTING_ANGLES = {
    CAM_X_INDEX: 90,
    CAM_Y_INDEX: 52,
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

# Local Fallback Mapping Config
LOCAL_MAP_SIZE = 60          # 60x60 grid cells
LOCAL_MAP_RESOLUTION = 0.10  # 10 cm per cell -> 6m x 6m local window
ROBOT_HEIGHT_M = 0.18        # Ground-to-camera standing height
