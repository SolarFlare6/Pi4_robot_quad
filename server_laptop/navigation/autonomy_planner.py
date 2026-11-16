# server_laptop/navigation/autonomy_planner.py
"""Autonomy Planner & Obstacle Avoidance Decision Engine.

Analyzes the 2D occupancy grid and tracked obstacles to determine high-level
locomotion directives ('walk_forward', 'turn_left', 'turn_right', 'stop').
"""

import logging
from typing import Any, Dict

logger = logging.getLogger("server.planner")


class AutonomyPlanner:
    def __init__(self, safe_stop_dist: float = 0.45, clear_threshold: float = 0.80):
        self.safe_stop_dist = safe_stop_dist
        self.clear_threshold = clear_threshold

    def plan_next_move(self, sector_clearance: Dict[str, Any]) -> Dict[str, Any]:
        """Calculates optimal locomotion directive based on obstacle clearances."""
        front_d = sector_clearance.get("front_clear_dist_m", 5.0)
        left_d = sector_clearance.get("left_clear_dist_m", 5.0)
        right_d = sector_clearance.get("right_clear_dist_m", 5.0)

        command = "walk_forward"
        speed = 1.0
        duration = 0.5

        # Immediate danger of collision
        if front_d < self.safe_stop_dist:
            # Need to turn or stop
            if left_d > right_d and left_d > self.safe_stop_dist:
                command = "turn_left"
                duration = 0.6
            elif right_d >= left_d and right_d > self.safe_stop_dist:
                command = "turn_right"
                duration = 0.6
            else:
                command = "stop"
                duration = 0.5
        elif front_d < self.clear_threshold:
            # Slow down and begin veering toward the more open side
            if left_d > right_d:
                command = "turn_left"
                duration = 0.4
            else:
                command = "turn_right"
                duration = 0.4
        else:
            # Clear path ahead
            command = "walk_forward"
            speed = 1.0
            duration = 0.5

        return {
            "command": command,
            "speed": speed,
            "duration": duration,
            "front_clear_m": front_d,
        }
