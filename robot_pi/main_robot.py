# robot_pi/main_robot.py
"""Main Application Entrypoint for the Quadruped Robot (Raspberry Pi 4).

Orchestrates:
1. One-time startup hook (initialize_robot)
2. MachineMake DCP Device Server on port 8765
3. Camera ingestion, Pan/Tilt servoing, and lightweight target tracking
4. IMU telemetry and fallback local 2D obstacle mapping
5. ZeroMQ uplink/downlink to Laptop Performance Server
6. Autonomous mode and navigation execution
"""

import argparse
import asyncio
import logging
import os
import signal
import sys
import time

# Ensure local directories are in python search path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from config_robot import (
    BETTER_STAND_ANGLES,
    CAM_X_INDEX,
    CAM_Y_INDEX,
    CAMERA_FPS,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_WIDTH,
    DCP_WEBSOCKET_PORT,
    JPEG_QUALITY,
    LAPTOP_SERVER_IP,
    RESTING_ANGLES,
    UDP_DISCOVERY_PORT,
    ZMQ_COMMAND_PORT,
    ZMQ_STREAM_PORT,
)
from init_hooks import initialize_robot
from voice import voice_synth
from navigation import locomotion_placeholders
from navigation.autonomy_controller import AutonomyController
from vision.pan_tilt_controller import PanTiltController
from vision.camera_worker import CameraWorker
from vision.local_tracker import LocalTracker
from vision.scan_routine import LookAroundScanner
from sensors.imu_handler import IMUHandler
from mapping.local_mapper import LocalFallbackMapper
from comms.network_client import NetworkClient
from comms.dcp_server_integration import DCPServerBridge

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("robot.main")


async def main_loop(server_ip: str, enable_autonomy_on_start: bool = False):
    logger.info("==================================================")
    logger.info("   STARTING QUADRUPED ROBOT CORE APPLICATION      ")
    logger.info("==================================================")

    # 1. Initialize Subsystems
    pan_tilt = PanTiltController(pan_channel=CAM_X_INDEX, tilt_channel=CAM_Y_INDEX)
    scanner = LookAroundScanner(pan_tilt)
    camera = CameraWorker(camera_index=CAMERA_INDEX, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS)
    tracker = LocalTracker(tracker_type="NANOTRACK")
    imu = IMUHandler(sample_rate_hz=50.0)
    local_mapper = LocalFallbackMapper()

    # 2. Run one-time startup initialization hook
    init_context = {
        "pan_tilt": pan_tilt,
        "imu": imu,
        "locomotion": locomotion_placeholders,
        "voice": voice_synth,
    }
    initialize_robot(init_context)

    # 3. Start Camera Capture Worker
    camera.start()

    # 4. ZeroMQ Network Client (Communication with Laptop Performance Server)
    net_client = NetworkClient(
        server_ip=server_ip,
        stream_port=ZMQ_STREAM_PORT,
        command_port=ZMQ_COMMAND_PORT,
        discovery_port=UDP_DISCOVERY_PORT,
    )

    # 5. Autonomy Controller
    autonomy = AutonomyController(
        scanner=scanner,
        local_mapper=local_mapper,
        network_client=net_client,
        locomotion_placeholders=locomotion_placeholders,
        voice_synth=voice_synth,
    )

    # 6. MachineMake DCP Device Server Bridge (Mobile App WebSocket)
    dcp_bridge = DCPServerBridge(
        port=DCP_WEBSOCKET_PORT,
        autonomy_callback=lambda enabled: autonomy.set_autonomy(enabled),
        locomotion_handler=locomotion_placeholders,
    )
    dcp_bridge.camera_worker = camera
    dcp_bridge.tracker = tracker
    dcp_bridge.pan_tilt = pan_tilt

    latest_server_track = None

    # Connect network commands to autonomy controller & tracker
    def _on_server_command(cmd_data):
        nonlocal latest_server_track
        if "command" in cmd_data:
            autonomy.receive_server_command(cmd_data)
        if "detections" in cmd_data and isinstance(cmd_data["detections"], list):
            dcp_bridge.latest_detections = cmd_data["detections"]
        if "server_track" in cmd_data:
            latest_server_track = cmd_data.get("server_track")
        if cmd_data.get("cancel_track"):
            latest_server_track = None
            tracker.cancel(from_server=True)
        elif "reinit_track" in cmd_data:
            # Laptop sent selected/updated bounding box
            bbox = cmd_data.get("reinit_track")
            label = cmd_data.get("track_label")
            success, frame, _ = camera.read()
            if success and frame is not None and bbox:
                tracker.start_track(frame, bbox, label=label, from_server=True)

    net_client.command_callback = _on_server_command
    net_client.start()

    await dcp_bridge.start()

    if enable_autonomy_on_start:
        autonomy.set_autonomy(True)

    # 7. Main Real-time Superloop (30 Hz)
    logger.info("Entering 30 Hz real-time superloop...")
    stop_event = asyncio.Event()

    async def _console_reader():
        if not sys.stdin.isatty():
            return
        loop_inner = asyncio.get_running_loop()
        while not stop_event.is_set():
            try:
                line = await loop_inner.run_in_executor(None, sys.stdin.readline)
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd == "qr":
                    dcp_bridge.generate_qr("Mobile App")
                elif cmd in ("quit", "exit", "q"):
                    stop_event.set()
                    break
                elif cmd == "autonomy":
                    autonomy.set_autonomy(not autonomy.is_autonomous)
            except Exception:
                break

    console_task = asyncio.create_task(_console_reader())

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except NotImplementedError:
            pass  # Windows fallback

    last_dcp_telemetry = time.time()
    last_dcp_frame = 0.0
    frame_interval = 1.0 / CAMERA_FPS

    try:
        while not stop_event.is_set():
            t_start = time.time()

            # A. Update IMU Readings
            imu_data = imu.update()

            # B. Read latest Camera Frame
            has_frame, frame, frame_ts = camera.read()

            # C. Object Tracking & Visual Servoing
            #    - Connected to `server_laptop` -> Offloaded to Laptop (YOLO26 + ONNX NanoTrack on Laptop)
            #    - Disconnected from `server_laptop` -> Local `onnx_tracking.py` (ONNX NanoTrack on Pi)
            server_connected = net_client.is_server_alive(timeout_s=1.8)
            tracking_info = {
                "active": False,
                "bbox": None,
                "err_x": 0.0,
                "err_y": 0.0,
                "target_label": tracker.target_label,
                "processed_on": "server_laptop" if server_connected else "robot_pi_onnx",
            }
            if has_frame and frame is not None:
                if tracker.is_tracking or tracker.mode != "idle" or (server_connected and latest_server_track):
                    locked, bbox, (err_x, err_y) = tracker.update(
                        frame,
                        detections=dcp_bridge.latest_detections,
                        server_connected=server_connected,
                        server_track=latest_server_track,
                    )
                    if locked and bbox is not None:
                        # Pause autonomous look-around sweep while actively tracking a target
                        if getattr(scanner, "_running", False):
                            scanner.stop()
                        tracking_info = {
                            "active": True,
                            "bbox": bbox,
                            "err_x": round(err_x, 1),
                            "err_y": round(err_y, 1),
                            "target_label": tracker.target_label,
                            "processed_on": "server_laptop" if server_connected else "robot_pi_onnx",
                        }
                        # Closed-loop Pan/Tilt visual servoing (only moves servo when object moves toward edge of view)
                        pan_tilt.track_error(
                            err_x,
                            err_y,
                            bbox=bbox,
                            frame_w=CAMERA_WIDTH,
                            frame_h=CAMERA_HEIGHT,
                        )
                    else:
                        if autonomy.is_autonomous and not getattr(scanner, "_running", False):
                            scanner.start()
                elif autonomy.is_autonomous and not getattr(scanner, "_running", False):
                    scanner.start()

                # D. Update Local Fallback Mapper
                cur_pan, cur_tilt = pan_tilt.get_angles()
                heading = imu_data.get("yaw", 0.0)
                map_res = local_mapper.process_frame(frame, cur_pan, cur_tilt, heading)
                local_mapper.current_directive = map_res.get("local_directive", "walk_forward")

            # Attach any mobile-app selection request so `server_laptop` can take over selection tracking
            client_req = tracker.consume_server_request()
            if client_req is not None:
                tracking_info["client_request"] = client_req

            # E. Stream Frame & Telemetry to Performance Server (ZeroMQ)
            jpeg_bytes = camera.encode_jpeg(quality=JPEG_QUALITY) if has_frame else None
            cur_pan, cur_tilt = pan_tilt.get_angles()

            net_client.send_telemetry_frame(
                jpeg_bytes=jpeg_bytes,
                imu_data=imu_data,
                servo_angles=BETTER_STAND_ANGLES,
                cam_x=cur_pan,
                cam_y=cur_tilt,
                local_tracker_data=tracking_info,
                autonomy_enabled=autonomy.is_autonomous,
            )

            # E2. Stream Camera Frame to Mobile App (DCP WebSocket at ~12 FPS when streaming is enabled)
            if (
                jpeg_bytes is not None
                and dcp_bridge.robot_controller is not None
                and getattr(dcp_bridge.robot_controller, "camera_streaming", False)
                and (time.time() - last_dcp_frame) >= 0.08
            ):
                await dcp_bridge.emit_camera_frame(
                    jpeg_bytes=jpeg_bytes,
                    width=CAMERA_WIDTH,
                    height=CAMERA_HEIGHT,
                    fps=12,
                    tracking_info=tracking_info,
                )
                last_dcp_frame = time.time()

            # F. Periodic MachineMake Telemetry Broadcast (2 Hz)
            if (time.time() - last_dcp_telemetry) >= 0.5:
                servo_table = {str(k): float(v) for k, v in BETTER_STAND_ANGLES.items()}
                servo_table[str(CAM_X_INDEX)] = cur_pan
                servo_table[str(CAM_Y_INDEX)] = cur_tilt
                await dcp_bridge.emit_telemetry(imu_data, servo_table, autonomy_enabled=autonomy.is_autonomous)
                last_dcp_telemetry = time.time()

            # Sleep remainder of frame budget
            elapsed = time.time() - t_start
            sleep_time = max(0.002, frame_interval - elapsed)
            await asyncio.sleep(sleep_time)

    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        logger.info("Shutting down robot systems...")
        console_task.cancel()
        autonomy.set_autonomy(False)
        scanner.stop()
        camera.stop()
        net_client.stop()
        await dcp_bridge.stop()
        locomotion_placeholders.emergency_stop()
        try:
            from cleanup import cleanup_all_hardware
            cleanup_all_hardware()
        except Exception as exc:
            logger.debug("Final hardware cleanup failed: %s", exc)
        logger.info("Robot shutdown complete.")


def run():
    parser = argparse.ArgumentParser(description="Quadruped Robot Core Controller")
    parser.add_argument("--server-ip", default=LAPTOP_SERVER_IP, help="IP address of Laptop Performance Server")
    parser.add_argument("--autonomy", action="store_true", help="Start directly in Autonomous Mode")
    args = parser.parse_args()

    try:
        asyncio.run(main_loop(server_ip=args.server_ip, enable_autonomy_on_start=args.autonomy))
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    run()
