# robot_pi/vision/local_tracker.py
"""Hybrid Selection & Person Object Tracker for `robot_pi`.

1. **When Connected to `server_laptop` (`server_connected=True`)**:
   - Selection tracking and person tracking are processed on `server_laptop`
     (using the laptop's GPU/CPU with YOLO26 + `onnx_tracking.py` NanoTrack),
     and `robot_pi` directly consumes the laptop's tracked bounding box without
     burdening the Raspberry Pi CPU.
2. **When Disconnected from `server_laptop` (`server_connected=False`)**:
   - Runs the exact `onnx_tracking.py` implementation locally on the Raspberry Pi
     (`cv2.TrackerNano_create(params)` with `nanotrack_backbone_sim.onnx` and
     `nanotrack_head_sim.onnx`, plus `tracker.getTrackingScore()`).
"""

import logging
import os
import threading
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

logger = logging.getLogger("robot.local_tracker")

_VISION_DIR = os.path.dirname(os.path.abspath(__file__))
_ROBOT_PI_DIR = os.path.dirname(_VISION_DIR)
_REPO_ROOT_DIR = os.path.dirname(_ROBOT_PI_DIR)
_ALT_OPENCV_DIR = os.path.join(os.path.dirname(_REPO_ROOT_DIR), "hardware_scripts_and_diagnostics", "openCV_work")

PERSON_LABELS = {"person", "people", "human", "face", "me", "user"}


def _find_model_file(filename: str) -> Optional[str]:
    """Locates a NanoTrack ONNX model file across standard project directories."""
    for path in (
        os.path.join(_VISION_DIR, "models", filename),
        os.path.join(_VISION_DIR, filename),
        os.path.join(_ROBOT_PI_DIR, filename),
        os.path.join(_ALT_OPENCV_DIR, filename),
        filename,
    ):
        if os.path.isfile(path):
            return os.path.abspath(path)
    return None


class LocalTracker:
    def __init__(self, tracker_type: str = "NANOTRACK"):
        self.tracker_type = tracker_type.upper()
        self._tracker = None
        self._using_nanotrack: bool = False
        self._local_initialized: bool = False
        self._lock = threading.Lock()

        self.is_tracking: bool = False
        self.mode: str = "idle"  # "idle", "selected", or "person"
        self.target_bbox: Optional[Tuple[int, int, int, int]] = None  # (x, y, w, h)
        self.target_label: Optional[str] = None
        self.confidence: float = 0.0

        # Queued selection request to forward to server_laptop when connected
        self._pending_server_request: Optional[Dict[str, Any]] = None

        self._backbone_path = _find_model_file("nanotrack_backbone_sim.onnx")
        self._neckhead_path = _find_model_file("nanotrack_head_sim.onnx")
        if self._backbone_path and self._neckhead_path:
            logger.info(
                "ONNX NanoTrack models ready: %s, %s",
                os.path.basename(self._backbone_path),
                os.path.basename(self._neckhead_path),
            )

        # Built-in OpenCV frontal face cascade for standalone person lookup on the Pi
        self._face_cascade = None
        try:
            cascade_path = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
            if os.path.isfile(cascade_path):
                self._face_cascade = cv2.CascadeClassifier(cascade_path)
        except Exception:
            self._face_cascade = None

    def _create_onnx_tracker(self):
        """Creates the NanoTrack ONNX tracker exactly as in `onnx_tracking.py`."""
        self._using_nanotrack = False
        if (
            hasattr(cv2, "TrackerNano_create")
            and hasattr(cv2, "TrackerNano_Params")
            and self._backbone_path
            and self._neckhead_path
        ):
            try:
                params = cv2.TrackerNano_Params()
                params.backbone = self._backbone_path
                params.neckhead = self._neckhead_path
                params.backend = cv2.dnn.DNN_BACKEND_OPENCV
                params.target = cv2.dnn.DNN_TARGET_CPU
                tracker = cv2.TrackerNano_create(params)
                self._using_nanotrack = True
                return tracker
            except Exception as e:
                logger.warning("TrackerNano_create failed (%s); falling back to TrackerMIL.", e)

        if hasattr(cv2, "TrackerMIL_create"):
            return cv2.TrackerMIL_create()
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerMIL_create"):
            return cv2.legacy.TrackerMIL_create()
        return None

    def consume_server_request(self) -> Optional[Dict[str, Any]]:
        """Returns and clears any pending selection/cancel request to send to `server_laptop`."""
        with self._lock:
            req = self._pending_server_request
            self._pending_server_request = None
            return req

    def start_track(
        self,
        frame: np.ndarray,
        bbox: Tuple[float, float, float, float],
        label: Optional[str] = None,
        from_server: bool = False,
    ) -> bool:
        """Locks onto an object that was selected inside the camera's current view `[x, y, w, h]`."""
        if frame is None or frame.size == 0:
            return False

        fh, fw = frame.shape[:2]
        x, y, w, h = [float(v) for v in bbox]

        # Support normalized coordinates (0.0..1.0)
        if 0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 < w <= 1.0 and 0.0 < h <= 1.0:
            x, y, w, h = x * fw, y * fh, w * fw, h * fh

        xi = max(0, min(fw - 10, int(round(x))))
        yi = max(0, min(fh - 10, int(round(y))))
        wi = max(10, min(fw - xi, int(round(w))))
        hi = max(10, min(fh - yi, int(round(h))))
        if wi <= 0 or hi <= 0:
            return False

        rect = (xi, yi, wi, hi)
        clean_label = (str(label).strip().lower() if label else "selected") or "selected"
        if clean_label in PERSON_LABELS:
            clean_label = "person"

        with self._lock:
            self._tracker = self._create_onnx_tracker()
            self._local_initialized = False
            if self._tracker is not None:
                try:
                    self._tracker.init(frame, rect)
                    self._local_initialized = True
                except Exception as exc:
                    logger.error("Failed to initialize ONNX tracker: %s", exc)

            self.is_tracking = True
            self.mode = "person" if clean_label == "person" else "selected"
            self.target_bbox = rect
            self.target_label = clean_label
            self.confidence = 1.0

            # If this selection originated on the robot/mobile app (not from server_laptop),
            # queue it so server_laptop can take over tracking when connected!
            if not from_server:
                self._pending_server_request = {
                    "action": "start",
                    "bbox": list(rect),
                    "label": clean_label,
                }

            logger.info(
                "Selection tracking started (%s): bbox=%s, label=%s",
                "ONNX-NanoTrack" if self._using_nanotrack else "MIL",
                rect,
                self.target_label,
            )
            return True

    def start_track_point(
        self,
        frame: np.ndarray,
        cx: float,
        cy: float,
        box_size: int = 84,
        label: Optional[str] = None,
    ) -> bool:
        """Locks onto an object at a tapped/clicked point `(cx, cy)` in the camera's view."""
        if frame is None or frame.size == 0:
            return False
        fh, fw = frame.shape[:2]
        px = float(cx) * fw if 0.0 <= float(cx) <= 1.0 else float(cx)
        py = float(cy) * fh if 0.0 <= float(cy) <= 1.0 else float(cy)
        if not (0 <= px <= fw and 0 <= py <= fh):
            return False

        half = max(20, int(box_size) // 2)
        x = max(0, min(fw - half * 2, int(round(px - half))))
        y = max(0, min(fh - half * 2, int(round(py - half))))
        return self.start_track(frame, (x, y, half * 2, half * 2), label=label or "selected")

    def set_target_label(self, label: Optional[str]) -> bool:
        """Enables looking for a person in the camera's view."""
        clean = (str(label).strip().lower() if label else "")
        with self._lock:
            if not clean:
                self.mode = "idle"
                self.target_label = None
                return False
            if clean in PERSON_LABELS:
                self.mode = "person"
                self.target_label = "person"
                self._pending_server_request = {"action": "person", "label": "person"}
                logger.info("Tracker set to look for people in camera view.")
                return True
            logger.info("Ignoring label '%s' (tracker only looks for people or a selected object in view).", clean)
            return False

    def _find_person_in_view(
        self,
        frame: np.ndarray,
        detections: Optional[List[Dict[str, Any]]] = None,
    ) -> Optional[Tuple[int, int, int, int]]:
        """Finds a person currently in the camera's view using YOLO26 detections or OpenCV face cascade."""
        fh, fw = frame.shape[:2]

        if detections:
            people = [
                d for d in detections
                if str(d.get("label", "")).strip().lower() == "person"
                and isinstance(d.get("bbox"), (list, tuple))
                and len(d["bbox"]) == 4
            ]
            if people:
                best = max(people, key=lambda d: float(d.get("conf", 0.0)))
                bx, by, bw, bh = [int(round(float(v))) for v in best["bbox"]]
                bx = max(0, min(fw - 10, bx))
                by = max(0, min(fh - 10, by))
                bw = max(10, min(fw - bx, bw))
                bh = max(10, min(fh - by, bh))
                return (bx, by, bw, bh)

        if self._face_cascade is not None:
            try:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = self._face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.2,
                    minNeighbors=5,
                    minSize=(40, 40),
                )
                if len(faces) > 0:
                    fx, fy, fw_box, fh_box = max(faces, key=lambda b: int(b[2]) * int(b[3]))
                    return (int(fx), int(fy), int(fw_box), int(fh_box))
            except Exception:
                pass

        return None

    def update(
        self,
        frame: np.ndarray,
        detections: Optional[List[Dict[str, Any]]] = None,
        server_connected: bool = False,
        server_track: Optional[Dict[str, Any]] = None,
    ) -> Tuple[bool, Optional[Tuple[int, int, int, int]], Tuple[float, float]]:
        """Updates tracking.

        - If `server_connected=True`: Uses the tracking result computed on `server_laptop`
          (`server_track` or YOLO26 person detection) instead of running ONNX on the Pi.
        - If `server_connected=False`: Runs `onnx_tracking.py` (`TrackerNano`) locally on the Pi!
        """
        with self._lock:
            if frame is None or frame.size == 0:
                return False, None, (0.0, 0.0)

            fh, fw = frame.shape[:2]

            # =========================================================================
            # PATH 1: CONNECTED TO `server_laptop` -> Offload tracking to the Laptop!
            # =========================================================================
            if server_connected:
                # 1a. Check if server_laptop sent an active selection/person track result
                if isinstance(server_track, dict) and server_track.get("active") and server_track.get("bbox"):
                    bx, by, bw, bh = [int(round(float(v))) for v in server_track["bbox"]]
                    bx = max(0, min(fw - 10, bx))
                    by = max(0, min(fh - 10, by))
                    bw = max(10, min(fw - bx, bw))
                    bh = max(10, min(fh - by, bh))

                    self.is_tracking = True
                    self.target_bbox = (bx, by, bw, bh)
                    self.target_label = str(server_track.get("label") or self.target_label or "selected")
                    self.mode = "person" if self.target_label in PERSON_LABELS else "selected"
                    self.confidence = float(server_track.get("score", 1.0))
                    # Mark local tracker uninitialized so if laptop disconnects later, Pi re-inits from latest bbox
                    self._local_initialized = False

                    center_x = bx + bw // 2
                    center_y = (by + int(bh * 0.35)) if (self.mode == "person" and bh > fh * 0.35) else (by + bh // 2)
                    err_x = float(center_x - (fw // 2))
                    err_y = float(center_y - (fh // 2))
                    return True, self.target_bbox, (err_x, err_y)

                # 1b. If in person mode while connected to laptop, also check YOLO26 person detections
                if self.mode == "person":
                    person_bbox = self._find_person_in_view(frame, detections)
                    if person_bbox is not None:
                        self.is_tracking = True
                        self.target_bbox = person_bbox
                        self.target_label = "person"
                        self.confidence = 1.0
                        self._local_initialized = False

                        bx, by, bw, bh = person_bbox
                        center_x = bx + bw // 2
                        center_y = by + (int(bh * 0.35) if bh > fh * 0.35 else bh // 2)
                        err_x = float(center_x - (fw // 2))
                        err_y = float(center_y - (fh // 2))
                        return True, self.target_bbox, (err_x, err_y)

                # 1c. If server_laptop explicitly reported track lost, clear state
                if isinstance(server_track, dict) and server_track.get("lost"):
                    self.is_tracking = False
                    self.target_bbox = None
                    self.confidence = 0.0
                    if self.mode == "selected":
                        self.mode = "idle"
                        self.target_label = None
                    return False, None, (0.0, 0.0)

                # If a track was just started on the Pi and we're waiting 1-2 frames for server_laptop's reply,
                # return current target_bbox without running heavy ONNX on the Pi
                if self.is_tracking and self.target_bbox is not None:
                    bx, by, bw, bh = self.target_bbox
                    center_x = bx + bw // 2
                    center_y = (by + int(bh * 0.35)) if (self.mode == "person" and bh > fh * 0.35) else (by + bh // 2)
                    return True, self.target_bbox, (float(center_x - fw // 2), float(center_y - fh // 2))

                return False, None, (0.0, 0.0)

            # =========================================================================
            # PATH 2: DISCONNECTED FROM `server_laptop` -> Run `onnx_tracking.py` on Pi!
            # =========================================================================
            if self.mode == "idle":
                return False, None, (0.0, 0.0)

            # If server_laptop just disconnected while tracking, seamlessly initialize local ONNX tracker
            if self.is_tracking and not self._local_initialized and self.target_bbox is not None:
                self._tracker = self._create_onnx_tracker()
                if self._tracker is not None:
                    try:
                        self._tracker.init(frame, self.target_bbox)
                        self._local_initialized = True
                        logger.info("Laptop disconnected -> seamlessly handed off to local ONNX NanoTrack on Pi!")
                    except Exception:
                        self._local_initialized = False

            # Standalone Person Mode fallback when not yet locked with ONNX tracker
            if self.mode == "person" and not self._local_initialized:
                person_bbox = self._find_person_in_view(frame, detections=None)
                if person_bbox is not None:
                    self._tracker = self._create_onnx_tracker()
                    if self._tracker is not None:
                        try:
                            self._tracker.init(frame, person_bbox)
                            self._local_initialized = True
                        except Exception:
                            pass
                    self.is_tracking = True
                    self.target_bbox = person_bbox
                    self.target_label = "person"
                    self.confidence = 1.0
                    bx, by, bw, bh = person_bbox
                    center_x = bx + bw // 2
                    center_y = by + (int(bh * 0.35) if bh > fh * 0.35 else bh // 2)
                    return True, self.target_bbox, (float(center_x - fw // 2), float(center_y - fh // 2))
                return False, None, (0.0, 0.0)

            # Run `onnx_tracking.py` update loop (`success, bbox = tracker.update(frame)`)
            if self.is_tracking and self._tracker is not None and self._local_initialized:
                try:
                    success, bbox = self._tracker.update(frame)
                    score = 1.0
                    if success and self._using_nanotrack and hasattr(self._tracker, "getTrackingScore"):
                        score = float(self._tracker.getTrackingScore())
                        self.confidence = round(score, 2)
                        if score < 0.42:
                            success = False
                except Exception as exc:
                    logger.debug("ONNX tracker update error: %s", exc)
                    success, bbox = False, None

                if success and bbox is not None:
                    x, y, w, h = [int(v) for v in bbox]
                    x = max(0, min(fw - 10, x))
                    y = max(0, min(fh - 10, y))
                    w = max(10, min(fw - x, w))
                    h = max(10, min(fh - y, h))
                    self.target_bbox = (x, y, w, h)

                    center_x = x + w // 2
                    center_y = (y + int(h * 0.35)) if (self.mode == "person" and h > fh * 0.35) else (y + h // 2)
                    err_x = float(center_x - (fw // 2))
                    err_y = float(center_y - (fh // 2))
                    return True, self.target_bbox, (err_x, err_y)

                # Object lost in standalone mode
                self.is_tracking = False
                self._local_initialized = False
                self.target_bbox = None
                self.confidence = 0.0
                if self.mode == "selected":
                    self.mode = "idle"
                    self.target_label = None
                logger.info("Object lost (ONNX tracker).")

            return False, None, (0.0, 0.0)

    def cancel(self, from_server: bool = False):
        """Cancels current object tracking."""
        with self._lock:
            self.is_tracking = False
            self.mode = "idle"
            self.target_bbox = None
            self.target_label = None
            self._tracker = None
            self._local_initialized = False
            self.confidence = 0.0
            if not from_server:
                self._pending_server_request = {"action": "cancel"}
        logger.info("Tracker canceled.")

    def stop(self):
        """Alias for cancel()."""
        self.cancel()
