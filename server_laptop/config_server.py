# server_laptop/config_server.py
"""Configuration settings for the Offboard Laptop Performance Server (Ryzen 7 + RTX 3050 Ti)."""

import os

# Network bindings
ZMQ_INGEST_PORT = int(os.getenv("PERF_STREAM_PORT", "5555"))   # Receives Pi frames and IMU (also binds 5565)
ZMQ_COMMAND_PORT = int(os.getenv("PERF_CMD_PORT", "5556"))     # Broadcasts commands and map to Pi (also binds 5566)
UDP_DISCOVERY_PORT = int(os.getenv("PERF_DISCOVERY_PORT", "5557"))  # Broadcasts Laptop IP on LAN
ROBOT_DCP_PORT = int(os.getenv("ROBOT_DCP_PORT", "8765"))      # Robot MachineMake DCP Server WebSocket port
ROBOT_IP = os.getenv("ROBOT_IP", "")                           # Optional explicit IP of Raspberry Pi (auto-discovered if empty)

# Deep Learning / YOLO26 Model
YOLO_MODEL_NAME = os.getenv("YOLO_MODEL", "yolo26n.pt")         # "yolo26n.pt", "yolo26s.pt", or "yolo11n.pt"
CONFIDENCE_THRESHOLD = 0.45
IOU_THRESHOLD = 0.45
USE_CUDA = True

# Tracking & Persistence
MAX_TRACK_AGE_FRAMES = 60          # ~2 seconds persistence after occlusion
PERSISTENCE_TIME_WINDOW_S = 10.0   # Persist spatial object location in world map for 10s

# Occupancy Grid Map Settings (High Resolution)
MAP_GRID_SIZE = 120                # 120x120 cells
MAP_RESOLUTION_M = 0.05            # 5 cm per cell -> 6m x 6m local environment
ROBOT_HEIGHT_M = 0.18              # Robot standing camera height

# Navigation / Obstacle Avoidance
SAFE_STOP_DISTANCE_M = 0.45        # Stop walking if obstacle is closer than 45 cm
CLEAR_PATH_THRESHOLD_M = 0.80      # Forward path is considered clear if unobstructed up to 80 cm
WALK_SPEED_DEFAULT = 1.0
