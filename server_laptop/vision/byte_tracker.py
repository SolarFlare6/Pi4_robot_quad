# server_laptop/vision/byte_tracker.py
"""Multi-Object Tracker integrating native YOLO26 track IDs with ByteTrack IoU association fallback.

Maintains track identities across successive frames, preserving native `model.track(persist=True)`
IDs when provided by YOLO26 and falling back to IoU association when needed.
"""

import logging
from typing import Any, Dict, List, Tuple
import numpy as np

logger = logging.getLogger("server.tracker")


def iou_batch(bboxes1: np.ndarray, bboxes2: np.ndarray) -> np.ndarray:
    """Computes Intersection-over-Union (IoU) matrix between two sets of bounding boxes.
    Boxes formatted as (x, y, w, h).
    """
    if len(bboxes1) == 0 or len(bboxes2) == 0:
        return np.zeros((len(bboxes1), len(bboxes2)))

    b1_x1, b1_y1, b1_x2, b1_y2 = bboxes1[:, 0], bboxes1[:, 1], bboxes1[:, 0] + bboxes1[:, 2], bboxes1[:, 1] + bboxes1[:, 3]
    b2_x1, b2_y1, b2_x2, b2_y2 = bboxes2[:, 0], bboxes2[:, 1], bboxes2[:, 0] + bboxes2[:, 2], bboxes2[:, 1] + bboxes2[:, 3]

    inter_x1 = np.maximum(b1_x1[:, None], b2_x1[None, :])
    inter_y1 = np.maximum(b1_y1[:, None], b2_y1[None, :])
    inter_x2 = np.minimum(b1_x2[:, None], b2_x2[None, :])
    inter_y2 = np.minimum(b1_y2[:, None], b2_y2[None, :])

    inter_w = np.maximum(0, inter_x2 - inter_x1)
    inter_h = np.maximum(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    area1 = bboxes1[:, 2] * bboxes1[:, 3]
    area2 = bboxes2[:, 2] * bboxes2[:, 3]
    union_area = area1[:, None] + area2[None, :] - inter_area

    return np.where(union_area > 0, inter_area / union_area, 0.0)


class TrackedTarget:
    def __init__(self, track_id: int, bbox: Tuple[int, int, int, int], label: str, conf: float):
        self.track_id = track_id
        self.bbox = bbox  # (x, y, w, h)
        self.label = label
        self.conf = conf
        self.age = 0
        self.time_since_update = 0
        self.hits = 1

    def update(self, bbox: Tuple[int, int, int, int], conf: float, label: str = ""):
        bx, by, bw, bh = self.bbox
        nx, ny, nw, nh = bbox
        self.bbox = (
            int(0.7 * nx + 0.3 * bx),
            int(0.7 * ny + 0.3 * by),
            int(0.7 * nw + 0.3 * bw),
            int(0.7 * nh + 0.3 * bh),
        )
        self.conf = conf
        if label:
            self.label = label
        self.time_since_update = 0
        self.hits += 1


class ByteTracker:
    def __init__(self, max_age: int = 30, min_hits: int = 1, iou_thresh: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_thresh = iou_thresh

        self.tracks: List[TrackedTarget] = []
        self._next_id = 1000

    def update(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Updates tracks with newly detected items from YOLO26."""
        for t in self.tracks:
            t.age += 1
            t.time_since_update += 1

        if not detections:
            self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]
            return [self._format_track(t) for t in self.tracks if t.hits >= self.min_hits]

        track_by_id = {t.track_id: t for t in self.tracks}
        unmatched_dets: List[Dict[str, Any]] = []

        # 1. First match detections that already carry a native YOLO26 track_id
        for d in detections:
            native_tid = d.get("track_id")
            if native_tid is not None:
                tid = int(native_tid)
                if tid in track_by_id:
                    track_by_id[tid].update(d["bbox"], d["conf"], d.get("label", ""))
                else:
                    new_t = TrackedTarget(track_id=tid, bbox=d["bbox"], label=d["label"], conf=d["conf"])
                    self.tracks.append(new_t)
                    track_by_id[tid] = new_t
            else:
                unmatched_dets.append(d)

        # 2. Fallback IoU matching for any detections without native track_id
        if unmatched_dets:
            unupdated_tracks = [t for t in self.tracks if t.time_since_update > 0]
            det_boxes = np.array([d["bbox"] for d in unmatched_dets], dtype=float)
            trk_boxes = np.array([t.bbox for t in unupdated_tracks], dtype=float) if unupdated_tracks else np.empty((0, 4))

            matched_dets = set()
            if len(trk_boxes) > 0 and len(det_boxes) > 0:
                iou_mat = iou_batch(trk_boxes, det_boxes)
                for _ in range(min(len(unupdated_tracks), len(unmatched_dets))):
                    t_idx, d_idx = np.unravel_index(np.argmax(iou_mat), iou_mat.shape)
                    if iou_mat[t_idx, d_idx] < self.iou_thresh:
                        break
                    unupdated_tracks[t_idx].update(
                        unmatched_dets[d_idx]["bbox"],
                        unmatched_dets[d_idx]["conf"],
                        unmatched_dets[d_idx].get("label", ""),
                    )
                    matched_dets.add(d_idx)
                    iou_mat[t_idx, :] = -1.0
                    iou_mat[:, d_idx] = -1.0

            for d_idx, d in enumerate(unmatched_dets):
                if d_idx not in matched_dets:
                    while self._next_id in track_by_id:
                        self._next_id += 1
                    new_track = TrackedTarget(
                        track_id=self._next_id,
                        bbox=d["bbox"],
                        label=d["label"],
                        conf=d["conf"],
                    )
                    self._next_id += 1
                    self.tracks.append(new_track)

        # Remove expired tracks
        self.tracks = [t for t in self.tracks if t.time_since_update <= self.max_age]

        # Return currently visible or recently confirmed tracks
        return [self._format_track(t) for t in self.tracks if t.time_since_update == 0]

    def _format_track(self, t: TrackedTarget) -> Dict[str, Any]:
        return {
            "track_id": t.track_id,
            "label": t.label,
            "conf": round(t.conf, 2),
            "bbox": t.bbox,
            "age": t.age,
        }
