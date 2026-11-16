# server_laptop/vision/object_persistence.py
"""Spatial Object Persistence Engine.

Registers recognized objects with persistent world-coordinate estimates $(X, Y)$ relative
to the robot. Even when the robot pans its camera away or turns around, objects remain
anchored in the world map memory for a configured time window.
"""

import logging
import math
import time
from typing import Any, Dict, List, Optional

logger = logging.getLogger("server.persistence")


class PersistentObject:
    def __init__(self, obj_id: int, label: str, world_x: float, world_y: float):
        self.obj_id = obj_id
        self.label = label
        self.world_x = world_x
        self.world_y = world_y
        self.first_seen = time.time()
        self.last_seen = time.time()
        self.confidence = 1.0

    def update_position(self, world_x: float, world_y: float):
        # Exponential moving average filter
        self.world_x = 0.7 * world_x + 0.3 * self.world_x
        self.world_y = 0.7 * world_y + 0.3 * self.world_y
        self.last_seen = time.time()

    def is_expired(self, expiry_seconds: float = 10.0) -> bool:
        return (time.time() - self.last_seen) > expiry_seconds


class ObjectPersistenceDatabase:
    def __init__(self, expiry_time_s: float = 10.0, match_dist_threshold_m: float = 0.8):
        self.expiry_time_s = expiry_time_s
        self.match_dist = match_dist_threshold_m
        self.objects: Dict[int, PersistentObject] = {}
        self._next_id = 100

    def update_from_detections(
        self,
        tracked_objects: List[Dict[str, Any]],
        robot_pose: Dict[str, float],
        ground_projector,
    ):
        """Projects detected boxes to world coordinates and updates persistent register."""
        now = time.time()

        for obj in tracked_objects:
            bbox = obj.get("bbox")
            label = obj.get("label", "object")
            if not bbox:
                continue

            # Project bottom center of bounding box to world plane (X, Y)
            coords = ground_projector.project_bbox_to_world(bbox, robot_pose)
            if coords is None:
                continue

            wx, wy = coords

            # Check if this object matches an existing persistent anchor nearby
            matched = False
            for p_obj in self.objects.values():
                dist = math.hypot(p_obj.world_x - wx, p_obj.world_y - wy)
                if dist < self.match_dist and p_obj.label == label:
                    p_obj.update_position(wx, wy)
                    obj["persistent_id"] = p_obj.obj_id
                    obj["world_pos"] = (round(wx, 2), round(wy, 2))
                    matched = True
                    break

            if not matched:
                new_p = PersistentObject(self._next_id, label, wx, wy)
                self.objects[self._next_id] = new_p
                obj["persistent_id"] = self._next_id
                obj["world_pos"] = (round(wx, 2), round(wy, 2))
                self._next_id += 1

        # Prune expired objects that haven't been seen recently
        expired = [oid for oid, p in self.objects.items() if p.is_expired(self.expiry_time_s)]
        for oid in expired:
            del self.objects[oid]

    def get_all_persistent_objects(self) -> List[Dict[str, Any]]:
        """Returns all currently maintained world objects."""
        return [
            {
                "id": p.obj_id,
                "label": p.label,
                "world_x": round(p.world_x, 2),
                "world_y": round(p.world_y, 2),
                "age_s": round(time.time() - p.first_seen, 1),
                "last_seen_s_ago": round(time.time() - p.last_seen, 1),
            }
            for p in self.objects.values()
        ]
