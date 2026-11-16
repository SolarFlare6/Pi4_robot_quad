# server_laptop/mapping/occupancy_grid_map.py
"""High-Resolution 2D Bayesian Occupancy Grid Map.

Maintains a 2D probabilistic representation of the robot's physical surroundings.
Updates cells using Bresenham raycasting:
- Cells traversed by camera visual rays are updated toward 'Free'
- Cells containing ground obstacle contacts are updated toward 'Occupied'
"""

import math
from typing import Any, Dict, List, Tuple
import numpy as np


class OccupancyGridMap:
    def __init__(self, size: int = 120, resolution_m: float = 0.05):
        self.size = size
        self.resolution = resolution_m
        self.origin = (size // 2, size // 2)

        # Log-odds probability representation (0 = unknown, <0 = free, >0 = occupied)
        self.log_odds = np.zeros((size, size), dtype=np.float32)

        # Log-odds update constants
        self.l_occ = 0.85
        self.l_free = -0.40
        self.l_max = 5.0
        self.l_min = -5.0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        """Converts world $(X, Y)$ meters to 2D matrix indices $(gx, gy)$."""
        ox, oy = self.origin
        gx = int(ox + (wx / self.resolution))
        gy = int(oy + (wy / self.resolution))
        return gx, gy

    def update_ray(self, start_w: Tuple[float, float], end_w: Tuple[float, float], is_occupied: bool = True):
        """Traces a ray using Bresenham algorithm, clearing free cells and marking target cell."""
        x0, y0 = self.world_to_grid(*start_w)
        x1, y1 = self.world_to_grid(*end_w)

        # Clip start point
        if not (0 <= x0 < self.size and 0 <= y0 < self.size):
            return

        # Bresenham ray traversal
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        curr_x, curr_y = x0, y0
        while True:
            # Reached endpoint
            if curr_x == x1 and curr_y == y1:
                if is_occupied and (0 <= curr_x < self.size and 0 <= curr_y < self.size):
                    self.log_odds[curr_y, curr_x] = min(self.l_max, self.log_odds[curr_y, curr_x] + self.l_occ)
                break

            # Intermediate cell is free space
            if 0 <= curr_x < self.size and 0 <= curr_y < self.size:
                self.log_odds[curr_y, curr_x] = max(self.l_min, self.log_odds[curr_y, curr_x] + self.l_free)

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                curr_x += sx
            if e2 < dx:
                err += dx
                curr_y += sy

            # Boundary break
            if not (0 <= curr_x < self.size and 0 <= curr_y < self.size):
                break

    def get_probability_grid(self) -> np.ndarray:
        """Converts log-odds grid to 0-100 probability values for visualization."""
        # p = 1 / (1 + exp(-log_odds))
        probs = 1.0 / (1.0 + np.exp(-self.log_odds))
        return (probs * 100.0).astype(np.uint8)

    def evaluate_sector_clearance(self, robot_x: float, robot_y: float, heading_deg: float) -> Dict[str, Any]:
        """Evaluates forward, left, and right sectors ahead of the robot.

        Returns sector clearance distances and obstacle detection warnings.
        """
        front_min_dist = 5.0
        left_min_dist = 5.0
        right_min_dist = 5.0

        # Sample angular arcs from -60 deg to +60 deg
        for angle_offset in range(-60, 65, 5):
            ray_angle = math.radians(heading_deg + angle_offset)
            for dist_m in np.arange(0.2, 2.5, 0.05):
                sample_x = robot_x + dist_m * math.cos(ray_angle)
                sample_y = robot_y + dist_m * math.sin(ray_angle)
                gx, gy = self.world_to_grid(sample_x, sample_y)

                if 0 <= gx < self.size and 0 <= gy < self.size:
                    if self.log_odds[gy, gx] > 1.0:  # Confirmed obstacle cell
                        if -20 <= angle_offset <= 20:
                            front_min_dist = min(front_min_dist, dist_m)
                        elif angle_offset < -20:
                            right_min_dist = min(right_min_dist, dist_m)
                        else:
                            left_min_dist = min(left_min_dist, dist_m)
                        break

        return {
            "front_clear_dist_m": round(front_min_dist, 2),
            "left_clear_dist_m": round(left_min_dist, 2),
            "right_clear_dist_m": round(right_min_dist, 2),
        }
