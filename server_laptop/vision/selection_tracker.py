# server_laptop/vision/selection_tracker.py
"""ONNX NanoTrack Selection Object Tracker for the Laptop Performance Server.

When `robot_pi` is connected to `server_laptop`, selection object tracking is offloaded
from the Raspberry Pi to this module on the Laptop:
- Uses the exact `onnx_tracking.py` implementation (`cv2.TrackerNano_create` with
  `nanotrack_backbone_sim.onnx` and `nanotrack_head_sim.onnx`) to track any selected
  bounding box or object in the camera view.
- Combines NanoTrack's smooth 30 FPS bounding-box tracking with YOLO26 re-identification
  when the selected object also matches a YOLO26 `track_id` or `"person"`.
"""

import logging
import os
from typing import Any, Dict, List, Optional, Tuple
import cv2

logger = logging.getLogger("server.selection_tracker")

VISION_DIR = os.path.dirname(os.path.abspath(__file__))
CANDIDATE_MODEL_DIRS = [
    os.path.join(VISION_DIR, "models"),
    os.path.abspath(os.path.join(VISION_DIR, "..", "..", "robot_pi", "vision", "models")),
    os.path.abspath(os.path.join(VISION_DIR, "..", "..", "..", "hardware_scripts_and_diagnostics", "openCV_work")),
]


def _resolve_onnx_model(filename: str) -> str:
    for d in CANDIDATE_MODEL_DIRS:
        candidate = os.path.join(d, filename)
        if os.path.isfile(candidate):
            return candidate
    return os.path.join(CANDIDATE_MODEL_DIRS[0], filename)


class LaptopSelectionTracker:
    """Runs `onnx_tracking.py` (NanoTrack ONNX) on `server_laptop` for offloaded selection tracking."""

    def __init__(self, score_threshold: float = 0.42):
        self.score_threshold = score_threshold
        self.backbone_path = _resolve_onnx_model("nanotrack_backbone_sim.onnx")
        self.head_path = _resolve_onnx_model("nanotrack_head_sim.onnx")

        self.tracker = None
        self.is_tracking: bool = False
        self.bbox: Optional[Tuple[int, int, int, int]] = None
        self.target_label: Optional[str] = None
        self.selected_track_id: Optional[int] = None
        self.last_score: float = 0.0
        self._just_lost: bool = False

    def _create_nanotrack(self):
        """Creates `cv2.TrackerNano` using the `onnx_tracking.py` ONNX models."""
        if (
            hasattr(cv2, "TrackerNano_create")
            and hasattr(cv2, "TrackerNano_Params")
            and os.path.isfile(self.backbone_path)
            and os.path.isfile(self.head_path)
        ):
            try:
                params = cv2.TrackerNano_Params()
                params.backbone = self.backbone_path
                params.neckhead = self.head_path
                return cv2.TrackerNano_create(params)
            except Exception as exc:
                logger.warning("TrackerNano creation failed on laptop (%s); falling back to CSRT.", exc)

        for creator_path in ("TrackerCSRT_create", "legacy.TrackerCSRT_create", "TrackerKCF_create"):
            obj = cv2
            for attr in creator_path.split("."):
                obj = getattr(obj, attr, None)
            if callable(obj):
                return obj()
        return None

    def start_track(
        self,
        frame,
        bbox: List[int],
        label: Optional[str] = None,
        track_id: Optional[int] = None,
    ) -> bool:
        """Initializes `onnx_tracking.py` NanoTrack on the given frame and bounding box."""
        if frame is None or bbox is None or len(bbox) != 4:
            return False

        fh, fw = frame.shape[:2]
        x, y, w, h = [int(v) for v in bbox]
        x = max(0, min(x, fw - 10))
        y = max(0, min(y, fh - 10))
        w = max(12, min(w, fw - x))
        h = max(12, min(h, fh - y))

        self.tracker = self._create_nanotrack()
        if self.tracker is None:
            return False

        try:
            self.tracker.init(frame, (x, y, w, h))
            self.is_tracking = True
            self.bbox = (x, y, w, h)
            self.target_label = label or "selected"
            self.selected_track_id = track_id
            self.last_score = 1.0
            self._just_lost = False
            logger.info(
                "[Laptop ONNX NanoTrack] Initialized on bbox=(%d, %d, %d, %d) label=%s track_id=%s",
                x, y, w, h, self.target_label, self.selected_track_id,
            )
            return True
        except Exception as exc:
            logger.error("Failed to initialize Laptop ONNX NanoTrack: %s", exc)
            self.is_tracking = False
            self.bbox = None
            return False

    def cancel(self):
        """Cancels active selection tracking on the laptop."""
        self.tracker = None
        self.is_tracking = False
        self.bbox = None
        self.target_label = None
        self.selected_track_id = None
        self.last_score = 0.0
        self._just_lost = False

    def update(self, frame, tracked_objects: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
        """Updates the ONNX NanoTrack tracker on the current laptop frame and returns `server_track` dict."""
        if frame is None:
            return {"active": False, "bbox": None, "label": self.target_label, "score": 0.0}

        # 1. If searching for 'person' by label and not yet initialized on a bounding box
        if not self.is_tracking and self.target_label and self.target_label.lower() in ("person", "people", "human"):
            if tracked_objects:
                matching = [o for o in tracked_objects if str(o.get("label", "")).lower() == "person"]
                if matching:
                    best = max(matching, key=lambda o: float(o.get("conf", 0.0)))
                    self.start_track(
                        frame,
                        list(best["bbox"]),
                        label="person",
                        track_id=best.get("track_id"),
                    )

        if not self.is_tracking or self.tracker is None:
            lost = self._just_lost
            self._just_lost = False
            return {
                "active": False,
                "bbox": None,
                "label": self.target_label,
                "score": 0.0,
                "lost": lost,
            }

        # 2. Run `onnx_tracking.py` NanoTrack update on the laptop
        try:
            ok, new_box = self.tracker.update(frame)
            score = 1.0
            if hasattr(self.tracker, "getTrackingScore"):
                try:
                    score = float(self.tracker.getTrackingScore())
                except Exception:
                    score = 1.0
            self.last_score = score

            if ok and score >= self.score_threshold:
                x, y, w, h = [int(v) for v in new_box]
                if w > 8 and h > 8:
                    self.bbox = (x, y, w, h)
                    return {
                        "active": True,
                        "bbox": [x, y, w, h],
                        "label": self.target_label or "selected",
                        "score": round(score, 3),
                    }
        except Exception as exc:
            logger.debug("Laptop NanoTrack update error: %s", exc)

        # 3. If NanoTrack confidence dropped, check if YOLO26 still sees the locked `selected_track_id` (or person)
        if tracked_objects:
            reacquired_obj = None
            if self.selected_track_id is not None:
                reacquired_obj = next(
                    (o for o in tracked_objects if o.get("track_id") == self.selected_track_id),
                    None,
                )
            elif self.target_label and self.target_label.lower() in ("person", "people", "human"):
                matching = [o for o in tracked_objects if str(o.get("label", "")).lower() == "person"]
                if matching:
                    reacquired_obj = max(matching, key=lambda o: float(o.get("conf", 0.0)))

            if reacquired_obj is not None and reacquired_obj.get("bbox"):
                self.start_track(
                    frame,
                    list(reacquired_obj["bbox"]),
                    label=reacquired_obj.get("label", self.target_label or "selected"),
                    track_id=reacquired_obj.get("track_id"),
                )
                if self.is_tracking and self.bbox is not None:
                    return {
                        "active": True,
                        "bbox": list(self.bbox),
                        "label": self.target_label or "selected",
                        "score": round(self.last_score, 3),
                    }

        # Target left the camera view
        logger.info("[Laptop ONNX NanoTrack] Target '%s' lost (score=%.2f)", self.target_label, self.last_score)
        self.is_tracking = False
        self.bbox = None
        self.tracker = None
        self._just_lost = True
        return {
            "active": False,
            "bbox": None,
            "label": self.target_label,
            "score": round(self.last_score, 3),
            "lost": True,
        }
