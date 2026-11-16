# robot_pi/mapping/local_mapper.py
"""Stripped-Down Local Fallback World Mapping Engine.

Runs directly on the Raspberry Pi 4 CPU when the Laptop Performance Server is unavailable.
Constructs a coarse 2D occupancy grid map ($60 \\times 60$ cells at 10 cm/cell)
by combining camera edge/color floor contrast with MPU-6050 heading and pan/tilt angles.
"""

import logging
import math
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

logger = logging.getLogger("robot.local_mapper")


class LocalFallbackMapper:
    def __init__(self, grid_size: int = 60, resolution_m: float = 0.10, robot_height_m: float = 0.18):
        self.grid_size = grid_size
        self.resolution = resolution_m
        self.robot_height = robot_height_m

        # Center index of the robot within the local map
        self.origin = (grid_size // 2, grid_size // 2)

        # 0 = Unknown, 1 = Free, 100 = Occupied
        self.grid = np.zeros((grid_size, grid_size), dtype=np.int8)

        # Camera Intrinsics estimation (640x480 standard webcam FOV ~ 60 deg)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0

    def clear(self):
        """Clears the local map to unknown state."""
        self.grid.fill(0)

    def process_frame(
        self,
        frame: np.ndarray,
        pan_angle_deg: float,
        tilt_angle_deg: float,
        heading_yaw_deg: float,
    ) -> Dict[str, Any]:
        """Analyzes a camera frame and updates the local 2D occupancy grid.

        Returns:
            Dict containing obstacle proximity warnings and sector clearances.
        """
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Focus analysis on lower half of the frame (ground plane area)
        lower_region = gray[int(h * 0.45) : h, :]

        # Detect high-contrast edges and texture changes corresponding to obstacles
        blurred = cv2.GaussianBlur(lower_region, (5, 5), 0)
        edges = cv2.Canny(blurred, 40, 120)

        # Effective total camera angles in world coordinate frame
        # Pan: 90 is forward. Angle > 90 is left, < 90 is right.
        rel_pan = (pan_angle_deg - 90.0)
        total_yaw_rad = math.radians(heading_yaw_deg + rel_pan)

        # Tilt: 52 is horizontal center. Angle > 52 is looking down (toward 180).
        pitch_rad = math.radians(max(5.0, tilt_angle_deg - 52.0))

        # Sample vertical columns to find floor obstacle contact points
        num_cols = 12
        col_step = w // num_cols

        front_clear_dist_m = 3.0
        left_clear = True
        right_clear = True
        center_clear = True

        ox, oy = self.origin

        for col_idx in range(num_cols):
            col_x = col_idx * col_step + (col_step // 2)
            edge_column = edges[:, col_x]
            edge_ys = np.where(edge_column > 0)[0]

            if len(edge_ys) > 0:
                # Lowest edge in image = closest contact with ground
                bottom_edge_y = edge_ys[-1] + int(h * 0.45)

                # Monocular ground raycast distance estimate
                v_err = bottom_edge_y - self.cy
                angle_elevation = pitch_rad + math.atan2(v_err, self.fy)

                if angle_elevation > 0.05:
                    dist_m = self.robot_height / math.tan(angle_elevation)
                    dist_m = max(0.2, min(4.0, dist_m))

                    # Calculate relative bearing angle of this column
                    u_err = col_x - self.cx
                    bearing_rad = total_yaw_rad + math.atan2(u_err, self.fx)

                    # Project into 2D grid coordinates
                    gx = int(ox + (dist_m * math.cos(bearing_rad)) / self.resolution)
                    gy = int(oy + (dist_m * math.sin(bearing_rad)) / self.resolution)

                    if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                        self.grid[gy, gx] = 100  # Mark occupied

                    # Track sector warnings
                    if dist_m < 0.6:
                        if col_x < w * 0.35:
                            left_clear = False
                        elif col_x > w * 0.65:
                            right_clear = False
                        else:
                            center_clear = False
                            front_clear_dist_m = min(front_clear_dist_m, dist_m)

        # Formulate guidance recommendation
        directive = "walk_forward"
        if not center_clear:
            if left_clear and not right_clear:
                directive = "turn_left"
            elif right_clear and not left_clear:
                directive = "turn_right"
            elif left_clear and right_clear:
                directive = "turn_right"
            else:
                directive = "stop"

        return {
            "front_clear_distance_m": round(front_clear_dist_m, 2),
            "center_clear": center_clear,
            "left_clear": left_clear,
            "right_clear": right_clear,
            "local_directive": directive,
        }

    def get_map_snapshot(self) -> Dict[str, Any]:
        """Returns metadata and serialized occupancy grid for telemetry."""
        return {
            "size": self.grid_size,
            "resolution": self.resolution,
            "origin": self.origin,
            "grid_base64": self.grid.tobytes().hex(),
        }
