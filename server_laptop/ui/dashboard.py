# server_laptop/ui/dashboard.py
"""Visual Debug Dashboard for the Performance Server with Interactive Mouse Target Selection.

Renders an OpenCV desktop window displaying:
- Left pane: Robot's live camera feed with YOLO detection bboxes, active tracking IDs,
  and interactive Mouse Click / Drag-Box target selection for camera visual servoing.
- Right pane: 2D Top-down Occupancy Grid Map showing robot position, camera view cone, and persistent objects.

Interactive Controls:
- Left-Click on a detected object (or anywhere in the video): Locks robot camera onto that target.
- Left-Click + Drag on the video: Draws a custom bounding box for the robot camera to track.
- Right-Click (or press 'C'): Cancels active object tracking.
- Press 'Q' or 'ESC': Exits dashboard.
"""

import logging
import math
from typing import Any, Dict, List, Optional, Tuple
import cv2
import numpy as np

logger = logging.getLogger("server.dashboard")


class ServerDashboard:
    # Left camera pane bounds on the 1100x600 canvas
    PANE_X1, PANE_Y1 = 20, 30
    PANE_W, PANE_H = 540, 420
    PANE_X2, PANE_Y2 = PANE_X1 + PANE_W, PANE_Y1 + PANE_H

    def __init__(self, window_name: str = "Robot Vision & World Map (Laptop GPU Server)"):
        self.window_name = window_name
        self._mouse_registered = False

        # Frame dimensions (updated dynamically when frames arrive)
        self._frame_w: int = 640
        self._frame_h: int = 480

        # Cached tracked objects from latest render for hit-testing clicks
        self._last_tracked_objects: List[Dict[str, Any]] = []

        # Interactive selection state
        self.selected_track_id: Optional[int] = None
        self.selected_label: Optional[str] = None
        self.pending_reinit_bbox: Optional[List[int]] = None
        self.pending_cancel: bool = False

        # Mouse drag state (in canvas coordinates)
        self._dragging: bool = False
        self._drag_start: Optional[Tuple[int, int]] = None
        self._drag_current: Optional[Tuple[int, int]] = None

    def _ensure_mouse_callback(self):
        if not self._mouse_registered:
            try:
                cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(self.window_name, self._on_mouse)
                self._mouse_registered = True
            except Exception as exc:
                logger.debug("Could not register mouse callback yet: %s", exc)

    def _canvas_to_frame_coords(self, mx: int, my: int) -> Tuple[int, int]:
        """Maps canvas pixel (mx, my) inside the left video pane to original camera frame coordinates."""
        rel_x = max(0, min(self.PANE_W - 1, mx - self.PANE_X1))
        rel_y = max(0, min(self.PANE_H - 1, my - self.PANE_Y1))
        fx = int(round(rel_x * (self._frame_w / float(self.PANE_W))))
        fy = int(round(rel_y * (self._frame_h / float(self.PANE_H))))
        return fx, fy

    def _on_mouse(self, event: int, x: int, y: int, flags: int, param: Any):
        """Handles mouse clicks and drags inside the camera video pane to select/cancel tracking targets."""
        # Right-click anywhere cancels tracking
        if event == cv2.EVENT_RBUTTONDOWN:
            self.selected_track_id = None
            self.selected_label = None
            self.pending_reinit_bbox = None
            self.pending_cancel = True
            self._dragging = False
            logger.info("[Dashboard] Right-click -> Canceling robot object tracking.")
            return

        in_video_pane = (self.PANE_X1 <= x <= self.PANE_X2) and (self.PANE_Y1 <= y <= self.PANE_Y2)

        if event == cv2.EVENT_LBUTTONDOWN and in_video_pane:
            self._dragging = True
            self._drag_start = (x, y)
            self._drag_current = (x, y)

        elif event == cv2.EVENT_MOUSEMOVE and self._dragging:
            cx = max(self.PANE_X1, min(self.PANE_X2, x))
            cy = max(self.PANE_Y1, min(self.PANE_Y2, y))
            self._drag_current = (cx, cy)

        elif event == cv2.EVENT_LBUTTONUP and self._dragging:
            self._dragging = False
            sx, sy = self._drag_start or (x, y)
            ex = max(self.PANE_X1, min(self.PANE_X2, x))
            ey = max(self.PANE_Y1, min(self.PANE_Y2, y))

            drag_dist = math.hypot(ex - sx, ey - sy)
            if drag_dist >= 14.0:
                # User dragged a custom rectangle
                fx1, fy1 = self._canvas_to_frame_coords(min(sx, ex), min(sy, ey))
                fx2, fy2 = self._canvas_to_frame_coords(max(sx, ex), max(sy, ey))
                bw = max(20, fx2 - fx1)
                bh = max(20, fy2 - fy1)
                self.pending_reinit_bbox = [fx1, fy1, bw, bh]
                self.pending_cancel = False

                # Check if a YOLO tracked object overlaps this dragged box
                self.selected_track_id = None
                self.selected_label = "custom"
                cx_f = fx1 + bw / 2.0
                cy_f = fy1 + bh / 2.0
                for obj in self._last_tracked_objects:
                    ox, oy, ow, oh = obj["bbox"]
                    if ox <= cx_f <= ox + ow and oy <= cy_f <= oy + oh:
                        self.selected_track_id = obj.get("track_id")
                        self.selected_label = obj.get("label", "object")
                        break

                logger.info(
                    "[Dashboard] Drag-selected target bbox=%s (track_id=%s, label=%s)",
                    self.pending_reinit_bbox,
                    self.selected_track_id,
                    self.selected_label,
                )
            else:
                # Single Left-Click: Check if user clicked inside a detected YOLO object box
                fx, fy = self._canvas_to_frame_coords(ex, ey)
                matched_obj = None
                best_area = float("inf")
                for obj in self._last_tracked_objects:
                    ox, oy, ow, oh = obj["bbox"]
                    if ox <= fx <= ox + ow and oy <= fy <= oy + oh:
                        area = ow * oh
                        if area < best_area:
                            best_area = area
                            matched_obj = obj

                if matched_obj is not None:
                    self.selected_track_id = matched_obj.get("track_id")
                    self.selected_label = matched_obj.get("label", "object")
                    self.pending_reinit_bbox = list(matched_obj["bbox"])
                    self.pending_cancel = False
                    logger.info(
                        "[Dashboard] Clicked YOLO object -> Locked ID:%s (%s) bbox=%s",
                        self.selected_track_id,
                        self.selected_label,
                        self.pending_reinit_bbox,
                    )
                else:
                    # Clicked on an arbitrary point not covered by a YOLO box -> create 84x84 tracker box
                    box_sz = 84
                    bx = max(0, min(self._frame_w - box_sz, fx - box_sz // 2))
                    by = max(0, min(self._frame_h - box_sz, fy - box_sz // 2))
                    self.selected_track_id = None
                    self.selected_label = "point_target"
                    self.pending_reinit_bbox = [bx, by, box_sz, box_sz]
                    self.pending_cancel = False
                    logger.info("[Dashboard] Clicked point (%d, %d) -> Tracking custom box %s", fx, fy, self.pending_reinit_bbox)

            self._drag_start = None
            self._drag_current = None

    def consume_tracking_action(self) -> Tuple[Optional[List[int]], bool]:
        """Returns (reinit_bbox, cancel_flag) queued by user mouse/keyboard interaction and clears the queue."""
        bbox = self.pending_reinit_bbox
        cancel = self.pending_cancel
        self.pending_reinit_bbox = None
        self.pending_cancel = False
        return bbox, cancel

    def render(
        self,
        frame: Optional[np.ndarray],
        tracked_objects: List[Dict[str, Any]],
        occupancy_grid_map,
        robot_pose: Dict[str, float],
        persistent_objects: List[Dict[str, Any]],
        navigation_directive: Dict[str, Any],
        is_robot_connected: bool = False,
        connection_detail: str = "",
        robot_tracker_info: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Renders combined perception and world map HUD view.

        Returns False if user pressed 'q' or 'ESC' to exit.
        """
        self._ensure_mouse_callback()
        self._last_tracked_objects = list(tracked_objects or [])

        canvas_h, canvas_w = 600, 1100
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        # Status badge colors
        if is_robot_connected:
            badge_color = (0, 200, 0)      # Green
            badge_text = f"ROBOT: CONNECTED ({connection_detail})" if connection_detail else "ROBOT: CONNECTED"
        else:
            badge_color = (0, 0, 220)      # Red
            badge_text = "ROBOT: DISCONNECTED (AUTO-DISCOVERING)"

        # Draw connection status pill at top right
        cv2.rectangle(canvas, (710, 6), (1085, 28), (40, 40, 40), -1)
        cv2.rectangle(canvas, (710, 6), (1085, 28), badge_color, 1)
        cv2.circle(canvas, (725, 17), 5, badge_color, -1)
        cv2.putText(canvas, badge_text[:44], (738, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.42, badge_color, 1)

        # 1. Left View: Camera Frame with Detections & Interactive Target Lock
        if frame is not None:
            self._frame_h, self._frame_w = frame.shape[:2]
            vis_frame = frame.copy()

            # Draw detected / tracked bounding boxes from YOLOv8 + ByteTrack
            for obj in tracked_objects:
                x, y, w, h = obj["bbox"]
                tid = obj.get("track_id", "?")
                lbl = obj.get("label", "object")
                conf = obj.get("conf", 0.0)
                is_selected = (self.selected_track_id is not None and tid == self.selected_track_id)

                if is_selected:
                    color = (0, 165, 255)  # Bright Orange for selected target
                    cv2.rectangle(vis_frame, (x, y), (x + w, y + h), color, 3)
                    cx, cy = x + w // 2, y + h // 2
                    cv2.drawMarker(vis_frame, (cx, cy), color, cv2.MARKER_CROSS, 18, 2)
                    tag = f"[LOCKED] ID:{tid} {lbl} {conf:.2f}"
                    cv2.putText(vis_frame, tag, (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
                else:
                    color = (0, 230, 0)    # Green for normal detections
                    cv2.rectangle(vis_frame, (x, y), (x + w, y + h), color, 2)
                    tag = f"ID:{tid} {lbl} {conf:.2f}"
                    cv2.putText(vis_frame, tag, (x, max(20, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Also draw the Robot's onboard LocalTracker bounding box (cyan dashed/corner overlay)
            if robot_tracker_info and robot_tracker_info.get("active") and robot_tracker_info.get("bbox"):
                rx, ry, rw, rh = [int(v) for v in robot_tracker_info["bbox"]]
                rcx, rcy = rx + rw // 2, ry + rh // 2
                cv2.rectangle(vis_frame, (rx, ry), (rx + rw, ry + rh), (255, 255, 0), 2)
                cv2.circle(vis_frame, (rcx, rcy), 4, (255, 255, 0), -1)
                cv2.putText(
                    vis_frame,
                    "PI CAM TRACKING",
                    (rx, min(self._frame_h - 8, ry + rh + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (255, 255, 0),
                    1,
                )

            # Resize to fit left pane
            vis_frame = cv2.resize(vis_frame, (self.PANE_W, self.PANE_H))
            canvas[self.PANE_Y1 : self.PANE_Y2, self.PANE_X1 : self.PANE_X2] = vis_frame

            # If user is currently dragging a selection box with the mouse, draw it live
            if self._dragging and self._drag_start and self._drag_current:
                cv2.rectangle(canvas, self._drag_start, self._drag_current, (0, 255, 255), 2)
        else:
            cv2.putText(canvas, "Awaiting Camera Stream...", (100, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)

        cv2.putText(
            canvas,
            "ROBOT CAMERA FEED (Click/Drag on object to Track | Right-Click to Cancel)",
            (20, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (255, 255, 255),
            1,
        )

        # 2. Right View: 2D Occupancy Grid Map
        if occupancy_grid_map is not None:
            probs = occupancy_grid_map.get_probability_grid()
            # Invert: white = free, black = occupied
            map_img = 255 - probs
            map_color = cv2.applyColorMap(map_img, cv2.COLORMAP_BONE)
            map_resized = cv2.resize(map_color, (420, 420), interpolation=cv2.INTER_NEAREST)

            # Draw robot position at center
            cx, cy = 210, 210
            cv2.circle(map_resized, (cx, cy), 6, (0, 0, 255), -1)

            # Draw camera view cone
            cam_x = robot_pose.get("cam_x", 90.0)
            yaw = robot_pose.get("yaw", 0.0)
            total_heading_rad = math.radians(yaw + (cam_x - 90.0))
            cone_len = 45
            end_x = int(cx + cone_len * math.cos(total_heading_rad))
            end_y = int(cy + cone_len * math.sin(total_heading_rad))
            cv2.line(map_resized, (cx, cy), (end_x, end_y), (0, 255, 255), 2)

            canvas[30:450, 580:1000] = map_resized

        cv2.putText(canvas, "2D OCCUPANCY GRID WORLD MAP (LiDAR-free)", (580, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)

        # 3. Bottom Telemetry, Tracking & Navigation Status Bar
        nav_cmd = navigation_directive.get("command", "N/A")
        front_d = navigation_directive.get("front_clear_m", 0.0)
        num_p_objs = len(persistent_objects)

        pi_trk_active = bool(robot_tracker_info and robot_tracker_info.get("active"))
        if self.selected_track_id is not None:
            trk_status = f"LOCKED ID:{self.selected_track_id} ({self.selected_label})"
        elif pi_trk_active:
            trk_status = f"ACTIVE ({robot_tracker_info.get('target_label') or 'custom box'})"
        elif robot_tracker_info and robot_tracker_info.get("target_label"):
            trk_status = f"SEARCHING FOR '{robot_tracker_info.get('target_label')}'"
        else:
            trk_status = "IDLE (Click video to lock)"

        status_text = (
            f"Directive: {nav_cmd.upper()} | Front Clear: {front_d:.2f}m | "
            f"Objects: {num_p_objs} | Yaw: {robot_pose.get('yaw', 0.0):.1f} deg | "
            f"Pan: {robot_pose.get('cam_x', 90):.0f} deg | Tilt: {robot_pose.get('cam_y', 90):.0f} deg"
        )
        cv2.putText(canvas, status_text, (20, 485), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 200), 1)
        cv2.putText(
            canvas,
            f"Camera Tracker: {trk_status}",
            (20, 515),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 200, 255) if (self.selected_track_id is not None or pi_trk_active) else (180, 180, 180),
            1,
        )
        cv2.putText(
            canvas,
            "Controls: Left-Click/Drag video = Track Object | Right-Click or 'C' = Cancel Track | 'Q'/ESC = Quit",
            (20, 548),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (150, 150, 150),
            1,
        )

        cv2.imshow(self.window_name, canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            return False
        if key in (ord("c"), ord("C")):
            self.selected_track_id = None
            self.selected_label = None
            self.pending_reinit_bbox = None
            self.pending_cancel = True
            logger.info("[Dashboard] Pressed 'C' -> Canceling robot object tracking.")
        return True

    def close(self):
        cv2.destroyAllWindows()
