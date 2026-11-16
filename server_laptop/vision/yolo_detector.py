# server_laptop/vision/yolo_detector.py
"""CUDA-accelerated YOLO26 Object Detection & Multi-Object Tracking on the RTX 3050 Ti.

Executes inference and persistent tracking (`model.track(persist=True)`) on incoming video
frames from the robot, returning labeled bounding boxes, confidence scores, class categories,
and persistent tracker IDs.
"""

import logging
import os
from typing import Any, Dict, List
import numpy as np

logger = logging.getLogger("server.detector")

_VISION_DIR = os.path.dirname(os.path.abspath(__file__))
_SERVER_DIR = os.path.dirname(_VISION_DIR)
_ROOT_DIR = os.path.dirname(_SERVER_DIR)


def _resolve_model_path(model_name: str) -> str:
    """Resolves local model weight path across server_laptop/ and workspace root before fallback download."""
    for candidate in (
        model_name,
        os.path.join(_SERVER_DIR, model_name),
        os.path.join(_ROOT_DIR, model_name),
    ):
        if os.path.isfile(candidate):
            return os.path.abspath(candidate)
    return model_name


class YOLODetector:
    def __init__(self, model_name: str = "yolo26n.pt", conf_thresh: float = 0.45, use_cuda: bool = True):
        self.model_name = _resolve_model_path(model_name)
        self.conf_thresh = conf_thresh
        self.use_cuda = use_cuda
        self.device = "cpu"

        self.model = None
        self._load_model()

    def _load_model(self):
        try:
            from ultralytics import YOLO
            import torch

            self.device = "cuda" if (self.use_cuda and torch.cuda.is_available()) else "cpu"
            logger.info("Loading YOLO26 model '%s' on device: %s", self.model_name, self.device)
            self.model = YOLO(self.model_name)
            self.model.to(self.device)
            logger.info("YOLO26 initialized successfully on %s.", self.device)
        except Exception as exc:
            logger.warning("Could not initialize YOLO26 via ultralytics: %s. Using simulated detector.", exc)
            self.model = None

    def detect(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Runs YOLO26 detection + native persistent tracking (`model.track(persist=True)`) on a BGR frame.

        Returns:
            List[Dict[str, Any]]: List of detections:
                [ { "track_id": Optional[int], "label": str, "conf": float, "bbox": (x, y, w, h), "cls_id": int }, ... ]
        """
        if self.model is None or frame is None:
            return []

        detections: List[Dict[str, Any]] = []
        try:
            results = self.model.track(
                source=frame,
                conf=self.conf_thresh,
                persist=True,
                verbose=False,
            )

            for r in results:
                boxes = r.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0].cpu().numpy())
                    cls_id = int(box.cls[0].cpu().numpy())
                    label = self.model.names.get(cls_id, str(cls_id))
                    track_id = int(box.id[0].cpu().numpy()) if box.id is not None else None

                    w = max(1, int(x2 - x1))
                    h = max(1, int(y2 - y1))

                    det: Dict[str, Any] = {
                        "label": label,
                        "conf": round(conf, 3),
                        "bbox": (int(x1), int(y1), w, h),
                        "cls_id": cls_id,
                    }
                    if track_id is not None:
                        det["track_id"] = track_id
                    detections.append(det)

        except Exception as e:
            logger.debug("YOLO26 tracking/detection error: %s", e)

        return detections
