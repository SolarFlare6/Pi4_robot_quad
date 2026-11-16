# server_laptop/mapping/ground_projector.py
"""Monocular Ground Plane Raycasting & 3D-to-2D Coordinate Projector.

Translates pixel locations in the camera image into real-world $(X, Y)$ ground coordinates
relative to the robot, without requiring a physical LiDAR sensor.
Uses camera mounting height, IMU pitch/roll tilt, and Pan/Tilt servo angles.
"""

import math
from typing import Dict, Optional, Tuple


class GroundProjector:
    def __init__(
        self,
        robot_height_m: float = 0.18,
        fx: float = 500.0,
        fy: float = 500.0,
        cx: float = 320.0,
        cy: float = 240.0,
    ):
        self.robot_height = robot_height_m
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def pixel_to_ground(
        self,
        u: float,
        v: float,
        robot_pose: Dict[str, float],
    ) -> Optional[Tuple[float, float]]:
        """Projects image pixel $(u, v)$ onto the floor plane relative to robot world coordinates.

        Args:
            u, v: Image coordinates.
            robot_pose: Dict with 'yaw', 'cam_x' (pan), 'cam_y' (tilt), 'pitch', 'roll', 'robot_x', 'robot_y'.

        Returns:
            Optional[Tuple[float, float]]: Real-world coordinates $(X, Y)$ in meters, or None.
        """
        cam_x_deg = robot_pose.get("cam_x", 90.0)
        cam_y_deg = robot_pose.get("cam_y", 52.0)
        heading_deg = robot_pose.get("yaw", 0.0)
        imu_pitch = robot_pose.get("pitch", 0.0)

        # Pan angle: 90 is straight forward (>90 is left, <90 is right)
        rel_pan_deg = cam_x_deg - 90.0
        total_yaw_rad = math.radians(heading_deg + rel_pan_deg)

        # Tilt angle: 52 is horizontal center. Higher angle (>52, toward 180) points downward toward floor.
        tilt_pitch_deg = max(4.0, (cam_y_deg - 52.0) + imu_pitch)
        pitch_rad = math.radians(tilt_pitch_deg)

        # Elevation angle of pixel (v) relative to camera principal point
        v_err = v - self.cy
        pixel_elevation = pitch_rad + math.atan2(v_err, self.fy)

        if pixel_elevation <= 0.04:  # Looking at or above horizon, no ground intersection
            return None

        # Distance along ground plane
        ground_distance_m = self.robot_height / math.tan(pixel_elevation)
        if ground_distance_m > 6.0 or ground_distance_m < 0.15:
            return None

        # Azimuth angle of pixel (u)
        u_err = u - self.cx
        azimuth_rad = total_yaw_rad + math.atan2(u_err, self.fx)

        rx = robot_pose.get("robot_x", 0.0)
        ry = robot_pose.get("robot_y", 0.0)

        world_x = rx + ground_distance_m * math.cos(azimuth_rad)
        world_y = ry + ground_distance_m * math.sin(azimuth_rad)

        return world_x, world_y

    def project_bbox_to_world(
        self,
        bbox: Tuple[int, int, int, int],
        robot_pose: Dict[str, float],
    ) -> Optional[Tuple[float, float]]:
        """Projects the bottom-center contact point of an obstacle's bounding box onto the ground."""
        x, y, w, h = bbox
        bottom_u = x + (w / 2.0)
        bottom_v = y + h
        return self.pixel_to_ground(bottom_u, bottom_v, robot_pose)
