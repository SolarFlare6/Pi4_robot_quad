# server_laptop/main_server.py
"""Main Application Entrypoint for the Laptop Performance Server (Ryzen 7 + RTX 3050 Ti).

Executes:
1. ZeroMQ ingest of robot video frames and IMU/Servo data
2. YOLOv8 Deep Object Detection & ByteTrack multi-object tracking
3. Re-ID spatial object persistence database
4. Monocular ground plane projection and 2D occupancy grid mapping
5. High-level autonomy planning (obstacle avoidance directives sent back to robot)
6. Real-time visual dashboard
"""

import argparse
import logging
import os
import sys
import time
import cv2
import numpy as np

# Ensure local directories are in python search path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from config_server import (
    CLEAR_PATH_THRESHOLD_M,
    CONFIDENCE_THRESHOLD,
    MAP_GRID_SIZE,
    MAP_RESOLUTION_M,
    PERSISTENCE_TIME_WINDOW_S,
    ROBOT_DCP_PORT,
    ROBOT_HEIGHT_M,
    ROBOT_IP,
    SAFE_STOP_DISTANCE_M,
    UDP_DISCOVERY_PORT,
    USE_CUDA,
    YOLO_MODEL_NAME,
    ZMQ_COMMAND_PORT,
    ZMQ_INGEST_PORT,
)
from comms.network_server import NetworkServer
from vision.yolo_detector import YOLODetector
from vision.byte_tracker import ByteTracker
from vision.selection_tracker import LaptopSelectionTracker
from vision.object_persistence import ObjectPersistenceDatabase
from mapping.ground_projector import GroundProjector
from mapping.occupancy_grid_map import OccupancyGridMap
from navigation.autonomy_planner import AutonomyPlanner
from ui.dashboard import ServerDashboard

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("server.main")


class PerformanceServerApp:
    def __init__(self, model_name: str = YOLO_MODEL_NAME, headless: bool = False, robot_ip: str = ROBOT_IP):
        self.headless = headless

        logger.info("Initializing Laptop Performance Server Engine (RTX 3050 Ti)...")

        # 1. Vision & Tracking Subsystems
        self.detector = YOLODetector(model_name=model_name, conf_thresh=CONFIDENCE_THRESHOLD, use_cuda=USE_CUDA)
        self.tracker = ByteTracker(max_age=30, min_hits=2)
        self.selection_tracker = LaptopSelectionTracker()
        self.persistence_db = ObjectPersistenceDatabase(expiry_time_s=PERSISTENCE_TIME_WINDOW_S)

        # 2. Mapping & Spatial Projection
        self.ground_projector = GroundProjector(robot_height_m=ROBOT_HEIGHT_M)
        self.occupancy_grid = OccupancyGridMap(size=MAP_GRID_SIZE, resolution_m=MAP_RESOLUTION_M)

        # 3. Autonomy Navigation Planner
        self.planner = AutonomyPlanner(safe_stop_dist=SAFE_STOP_DISTANCE_M, clear_threshold=CLEAR_PATH_THRESHOLD_M)

        # 4. Dashboard Visualizer
        self.dashboard = ServerDashboard() if not self.headless else None

        # State Variables
        self.latest_payload = None
        self.latest_frame = None
        self.latest_robot_pose = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0, "cam_x": 90.0, "cam_y": 90.0}
        self.latest_robot_tracker: dict = {"active": False, "bbox": None, "target_label": None}
        self.pending_client_track_request: dict | None = None
        self.latest_directive = {"command": "stop", "front_clear_m": 5.0}
        self.last_robot_contact_time: float = 0.0
        self.last_reinit_sent_time: float = 0.0
        self.total_frames_received: int = 0

        # 5. Network Server (ZeroMQ + Active Robot DCP Discovery/Bridge)
        self.net_server = NetworkServer(
            ingest_port=ZMQ_INGEST_PORT,
            command_port=ZMQ_COMMAND_PORT,
            discovery_port=UDP_DISCOVERY_PORT,
            dcp_port=ROBOT_DCP_PORT,
            robot_ip=robot_ip or None,
            frame_callback=self._on_robot_data,
        )

    def _on_robot_data(self, data: dict):
        """Callback invoked whenever a new frame & telemetry packet arrives from the robot."""
        self.latest_payload = data
        self.last_robot_contact_time = time.time()
        self.total_frames_received += 1

        if self.total_frames_received == 1:
            logger.info(">>> FIRST CONTACT: Receiving telemetry stream from Robot! <<<")

        # Decode JPEG frame
        if data.get("has_frame") and data.get("frame"):
            try:
                np_arr = np.frombuffer(data["frame"], dtype=np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                self.latest_frame = frame
            except Exception as e:
                logger.debug("Failed to decode incoming JPEG frame: %s", e)

        # Update pose & robot tracker status
        imu = data.get("imu", {})
        self.latest_robot_pose = {
            "yaw": imu.get("yaw", 0.0),
            "pitch": imu.get("pitch", 0.0),
            "roll": imu.get("roll", 0.0),
            "cam_x": data.get("cam_x", 90.0),
            "cam_y": data.get("cam_y", 90.0),
            "robot_x": 0.0,
            "robot_y": 0.0,
        }
        trk = data.get("tracker")
        if isinstance(trk, dict):
            self.latest_robot_tracker = trk
            client_req = trk.get("client_request")
            if isinstance(client_req, dict):
                self.pending_client_track_request = client_req

    def run(self):
        self.net_server.start()
        logger.info("Performance Server running. Press Ctrl+C or 'Q' to quit.")

        try:
            while True:
                t0 = time.time()

                frame = self.latest_frame
                pose = self.latest_robot_pose

                tracked_objects = []
                if frame is not None:
                    # A. Deep Object Detection (YOLO26 on CUDA)
                    detections = self.detector.detect(frame)

                    # B. Multi-Object Tracking (YOLO26 Native + ByteTrack)
                    tracked_objects = self.tracker.update(detections)

                    # C. Update Persistent World Objects Register
                    self.persistence_db.update_from_detections(tracked_objects, pose, self.ground_projector)

                    # D. Update 2D Occupancy Grid Map
                    # For each detected object touching floor, trace ray and mark obstacle
                    for obj in tracked_objects:
                        bbox = obj.get("bbox")
                        if bbox:
                            world_pt = self.ground_projector.project_bbox_to_world(bbox, pose)
                            if world_pt:
                                self.occupancy_grid.update_ray((0.0, 0.0), world_pt, is_occupied=True)

                    # E. Evaluate Sector Clearances & Plan Next Move
                    clearances = self.occupancy_grid.evaluate_sector_clearance(0.0, 0.0, pose["yaw"])
                    directive = self.planner.plan_next_move(clearances)
                    self.latest_directive = directive

                    # F. Send Directives, Detections & Selected Track Target back to Robot
                    compact_detections = [
                        {
                            "track_id": obj.get("track_id"),
                            "label": obj.get("label", "object"),
                            "conf": round(float(obj.get("conf", 0.0)), 2),
                            "bbox": list(obj.get("bbox", [0, 0, 0, 0])),
                        }
                        for obj in tracked_objects
                    ]
                    feedback_msg = {
                        "command": directive["command"],
                        "speed": directive["speed"],
                        "duration": directive["duration"],
                        "front_clear_m": directive["front_clear_m"],
                        "detections": compact_detections,
                    }

                    # F1. Check if user clicked/dragged a target on Laptop Dashboard OR Mobile App (`client_request`)
                    clicked_bbox, cancel_requested = (
                        self.dashboard.consume_tracking_action() if self.dashboard is not None else (None, False)
                    )
                    client_req = self.pending_client_track_request
                    self.pending_client_track_request = None

                    if cancel_requested or (client_req and client_req.get("action") == "cancel"):
                        self.selection_tracker.cancel()
                        if self.dashboard is not None:
                            self.dashboard.selected_track_id = None
                            self.dashboard.selected_label = None
                        if cancel_requested:
                            feedback_msg["cancel_track"] = True
                        self.last_reinit_sent_time = time.time()

                    elif clicked_bbox is not None:
                        sel_label = self.dashboard.selected_label if self.dashboard else "selected"
                        sel_tid = self.dashboard.selected_track_id if self.dashboard else None
                        self.selection_tracker.start_track(
                            frame,
                            clicked_bbox,
                            label=sel_label,
                            track_id=sel_tid,
                        )
                        feedback_msg["reinit_track"] = clicked_bbox
                        feedback_msg["track_label"] = sel_label
                        self.last_reinit_sent_time = time.time()

                    elif client_req and client_req.get("action") == "init_bbox" and client_req.get("bbox"):
                        # Mobile app / robot_pi requested selection tracking -> offload to Laptop ONNX NanoTrack!
                        req_bbox = list(client_req["bbox"])
                        req_label = client_req.get("label") or "selected"
                        matched_tid = None
                        cx_f = req_bbox[0] + req_bbox[2] / 2.0
                        cy_f = req_bbox[1] + req_bbox[3] / 2.0
                        for obj in tracked_objects:
                            ox, oy, ow, oh = obj["bbox"]
                            if ox <= cx_f <= ox + ow and oy <= cy_f <= oy + oh:
                                matched_tid = obj.get("track_id")
                                if req_label in ("selected", "custom", "point_target", ""):
                                    req_label = obj.get("label", req_label)
                                break
                        if self.dashboard is not None:
                            self.dashboard.selected_track_id = matched_tid
                            self.dashboard.selected_label = req_label
                        self.selection_tracker.start_track(
                            frame,
                            req_bbox,
                            label=req_label,
                            track_id=matched_tid,
                        )

                    elif client_req and client_req.get("action") == "find_person":
                        self.selection_tracker.cancel()
                        self.selection_tracker.target_label = "person"
                        if self.dashboard is not None:
                            self.dashboard.selected_label = "person"

                    # F2. Also check if robot_pi is passively requesting 'person' search
                    requested_label = (self.latest_robot_tracker.get("target_label") or "").strip().lower()
                    if (
                        not self.selection_tracker.is_tracking
                        and requested_label in ("person", "people", "human")
                    ):
                        self.selection_tracker.target_label = "person"

                    # F3. Run Selection Object Tracking (`onnx_tracking.py` NanoTrack + YOLO26) on the Laptop
                    server_track = self.selection_tracker.update(frame, tracked_objects)
                    feedback_msg["server_track"] = server_track
                    if server_track.get("active") and server_track.get("bbox"):
                        self.latest_robot_tracker = {
                            "active": True,
                            "bbox": server_track["bbox"],
                            "target_label": server_track.get("label") or "selected",
                        }

                    self.net_server.send_command(feedback_msg)

                # Heartbeat command to robot so it knows laptop is listening
                if frame is None and (time.time() - self.last_robot_contact_time) > 1.0:
                    self.net_server.send_command({"command": "stop", "speed": 0.0, "duration": 0.5, "front_clear_m": 5.0})

                # G. Render Desktop Visualizer Dashboard
                if not self.headless and self.dashboard is not None:
                    persistent_list = self.persistence_db.get_all_persistent_objects()
                    is_connected = (time.time() - self.last_robot_contact_time) < 2.5 and self.total_frames_received > 0
                    conn_label = self.net_server.active_transport
                    if self.net_server.discovered_robot_ip and self.net_server.discovered_robot_ip not in conn_label:
                        conn_label = f"{self.net_server.discovered_robot_ip} | {conn_label}"
                    keep_running = self.dashboard.render(
                        frame=frame,
                        tracked_objects=tracked_objects,
                        occupancy_grid_map=self.occupancy_grid,
                        robot_pose=pose,
                        persistent_objects=persistent_list,
                        navigation_directive=self.latest_directive,
                        is_robot_connected=is_connected,
                        connection_detail=conn_label,
                        robot_tracker_info=self.latest_robot_tracker,
                    )
                    if not keep_running:
                        break

                elapsed = time.time() - t0
                time.sleep(max(0.005, 0.033 - elapsed))  # target ~30 FPS

        except KeyboardInterrupt:
            pass
        finally:
            logger.info("Stopping Performance Server...")
            self.net_server.stop()
            if self.dashboard:
                self.dashboard.close()
            logger.info("Performance Server stopped.")


def main():
    parser = argparse.ArgumentParser(description="Laptop Performance Server for Robot Vision & Mapping")
    parser.add_argument("--model", default=YOLO_MODEL_NAME, help="YOLO model path or name (e.g. yolov8n.pt)")
    parser.add_argument("--robot-ip", default=ROBOT_IP, help="Optional IP address of Raspberry Pi robot (auto-discovered if omitted)")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode without visual dashboard")
    args = parser.parse_args()

    app = PerformanceServerApp(model_name=args.model, headless=args.headless, robot_ip=args.robot_ip)
    app.run()


if __name__ == "__main__":
    main()
