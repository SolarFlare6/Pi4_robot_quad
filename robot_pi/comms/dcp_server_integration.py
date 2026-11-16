# robot_pi/comms/dcp_server_integration.py
"""Integration module embedding the MachineMake DCP Device Server into the robot.

Exposes tools to the mobile app (`machmake2`), advertises over mDNS (`_dcp._tcp.local`) and BLE,
provides interactive QR pairing, bridges telemetry to the app's 3D wireframe simulator,
and connects the `set_autonomy` toggle directly to the robot's autonomous navigation engine.
"""

import asyncio
import logging
import os
import sys
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger("robot.dcp_bridge")

# Add bundled device_server directory (and fallback candidate paths) to python import path
candidate_paths = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "device_server")),
    os.getenv("DEVICE_SERVER_DIR"),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "device_server")),
    os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "app", "hardware_layer_for_machine_make", "device_server")),
    "/home/pi/Desktop/machine_make/robot_pi/device_server",
    "/home/pi/Desktop/machine_make/device_server",
    "/home/pi/Desktop/device_server",
]

for p in candidate_paths:
    if p and os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)
        logger.info("Added device_server directory to sys.path: %s", p)


class DCPServerBridge:
    def __init__(
        self,
        port: int = 8765,
        autonomy_callback: Optional[Callable[[bool], None]] = None,
        locomotion_handler: Optional[Any] = None,
    ):
        self.port = port
        self.autonomy_callback = autonomy_callback
        self.locomotion = locomotion_handler
        self.camera_worker = None
        self.tracker = None
        self.pan_tilt = None
        self.latest_detections: list = []

        self.dcp_handler = None
        self.session_manager = None
        self.event_manager = None
        self.wifi_server = None
        self.bluetooth_server = None
        self.advertiser = None
        self.pairing_manager = None
        self.trust_store = None
        self.identity = None
        self.robot_controller = None

        self._running = False
        self._prev_cpu_total = 0
        self._prev_cpu_idle = 0

    async def start(self, robot_controller=None, auto_qr: bool = True):
        """Starts the complete MachineMake DCP Server (Wi-Fi WebSocket, BLE, mDNS Discovery, and Quadruped Profile)."""
        try:
            from auth import Authenticator, TrustStore
            from auto_config import build_config_for_profile
            from capability_mapper import CapabilityMapper
            from config import ConfigurationManager
            from dcp_handler import DCPHandler
            from desktop_controller import DesktopController
            from device_agent import DeviceAgent
            from discovery import DiscoveryAdvertiser
            from event_manager import EventManager
            from hardware_mapper.mapper import HardwareMapper
            from identity import DeviceIdentity
            from pairing import PairingManager
            from robot_controller import RobotController
            from safety_manager import SafetyManager
            from session import SessionManager
            from system_power import SystemPowerManager
            from task_manager import TaskManager
            from tool_registry import ToolRegistry
            from transports.bluetooth_transport import BluetoothDCPServer
            from transports.websocket_transport import WiFiDCPServer

            logger.info("Initializing MachineMake DCP Device Server on port %d...", self.port)

            # 1. Force Quadruped Robot Profile in ConfigurationManager
            config = ConfigurationManager()
            quad_name = "MachineMake Quadruped Dog"
            existing_id = config.identity.get("device_id") or "quadruped-pi-01"
            robot_cfg = build_config_for_profile("quadruped", custom_name=quad_name)
            robot_cfg["identity"]["device_id"] = existing_id
            config._config = robot_cfg
            try:
                config.save()
            except Exception as e:
                logger.debug("Could not persist device_config.json: %s", e)

            self.identity = DeviceIdentity(
                name=quad_name,
                profile="quadruped",
                device_type="robot",
                device_id=existing_id,
            )

            # 2. Scan Hardware & Instantiate RobotController with full physical_configuration
            hardware_mapper = HardwareMapper()
            hardware_map = hardware_mapper.scan()
            physical_config = dict(config.physical_configuration or {})

            if robot_controller is None:
                robot_controller = RobotController(physical_config, hardware_map=hardware_map)
            self.robot_controller = robot_controller

            # Hook autonomy toggle in robot_controller
            original_set_autonomy = robot_controller.set_autonomy

            async def _hooked_set_autonomy(enabled: bool = True):
                res = await original_set_autonomy(enabled=enabled)
                if self.autonomy_callback:
                    self.autonomy_callback(bool(enabled))
                return res

            robot_controller.set_autonomy = _hooked_set_autonomy

            # Hook camera_snapshot, take_picture, start_camera & stop_camera to use the active USB CameraWorker
            async def _hooked_take_picture():
                if self.camera_worker is not None:
                    import base64
                    jpg = self.camera_worker.encode_jpeg(quality=80)
                    if jpg:
                        b64_str = base64.b64encode(jpg).decode("utf-8")
                        if self.event_manager is not None and self.session_manager is not None:
                            for s in self.session_manager.all_sessions():
                                s.subscribed_events.add("camera_frame")
                            await self.event_manager.emit(
                                "camera_frame",
                                {
                                    "image_b64": b64_str,
                                    "format": "jpeg",
                                    "width": self.camera_worker.width,
                                    "height": self.camera_worker.height,
                                    "bytes": len(jpg),
                                },
                            )
                        return {
                            "status": "ok",
                            "format": "jpeg",
                            "bytes": len(jpg),
                            "image_b64": b64_str,
                        }
                return {"status": "ok", "format": "jpeg", "bytes": 0, "note": "camera not ready"}

            original_start_camera = robot_controller.start_camera
            original_stop_camera = robot_controller.stop_camera

            async def _hooked_start_camera(resolution: str = "640x480", fps: int = 30):
                res = await original_start_camera(resolution=resolution, fps=fps)
                if self.camera_worker is not None and not getattr(self.camera_worker, "_running", False):
                    self.camera_worker.start()
                if self.session_manager is not None:
                    for s in self.session_manager.all_sessions():
                        s.subscribed_events.add("camera_frame")
                return res

            async def _hooked_stop_camera():
                return await original_stop_camera()

            robot_controller.take_picture = _hooked_take_picture
            robot_controller.camera_snapshot = _hooked_take_picture
            robot_controller.start_camera = _hooked_start_camera
            robot_controller.camera_stream_start = _hooked_start_camera
            robot_controller.stop_camera = _hooked_stop_camera
            robot_controller.camera_stream_stop = _hooked_stop_camera

            # Hook object selection & visual tracking tools for Android App & Needle AI
            async def _hooked_track_object(
                x: Optional[float] = None,
                y: Optional[float] = None,
                w: Optional[float] = None,
                h: Optional[float] = None,
                u: Optional[float] = None,
                v: Optional[float] = None,
                label: Optional[str] = None,
                target: Optional[str] = None,
                box_size: int = 84,
                **kw,
            ):
                lbl = (label or target or "").strip() or None
                if self.tracker is None:
                    return {"status": "error", "message": "Tracker not initialized"}

                frame = None
                fw, fh = 640, 480
                if self.camera_worker is not None:
                    ok, frm, _ = self.camera_worker.read()
                    if ok and frm is not None:
                        frame = frm
                        fh, fw = frame.shape[:2]

                # 1. Explicit bounding box [x, y, w, h]
                if x is not None and y is not None and w is not None and h is not None and float(w) > 0 and float(h) > 0:
                    if frame is not None:
                        started = self.tracker.start_track(frame, (float(x), float(y), float(w), float(h)), label=lbl)
                        return {
                            "status": "tracking" if started else "failed",
                            "bbox": list(self.tracker.target_bbox) if self.tracker.target_bbox else [x, y, w, h],
                            "label": self.tracker.target_label,
                        }

                # 2. Point tap (x, y) or normalized tap (u, v) on the video stream
                tap_x = u if u is not None else x
                tap_y = v if v is not None else y
                if tap_x is not None and tap_y is not None and frame is not None:
                    px = float(tap_x) * fw if 0.0 <= float(tap_x) <= 1.0 else float(tap_x)
                    py = float(tap_y) * fh if 0.0 <= float(tap_y) <= 1.0 else float(tap_y)

                    # First check if tap landed inside any YOLOv8 detection from Laptop Server
                    for det in self.latest_detections:
                        dbbox = det.get("bbox")
                        if dbbox and len(dbbox) == 4:
                            dx, dy, dw, dh = [float(val) for val in dbbox]
                            if dx <= px <= dx + dw and dy <= py <= dy + dh:
                                det_lbl = det.get("label") or lbl
                                started = self.tracker.start_track(frame, (dx, dy, dw, dh), label=det_lbl)
                                return {
                                    "status": "tracking" if started else "failed",
                                    "bbox": list(self.tracker.target_bbox) if self.tracker.target_bbox else dbbox,
                                    "label": self.tracker.target_label,
                                    "matched_detection": True,
                                }

                    started = self.tracker.start_track_point(frame, px, py, box_size=int(box_size), label=lbl or "tap_target")
                    return {
                        "status": "tracking" if started else "failed",
                        "bbox": list(self.tracker.target_bbox) if self.tracker.target_bbox else None,
                        "label": self.tracker.target_label,
                    }

                # 3. Label selection: only 'person' or an object currently visible in the camera's view
                if lbl:
                    lbl_low = lbl.lower()
                    if lbl_low in ("center", "middle", "front") and frame is not None:
                        started = self.tracker.start_track_point(frame, fw / 2.0, fh / 2.0, box_size=int(box_size), label="selected")
                        return {
                            "status": "tracking" if started else "failed",
                            "bbox": list(self.tracker.target_bbox) if self.tracker.target_bbox else None,
                            "label": "selected",
                        }

                    # If user asked for a person, enable person tracking mode
                    if lbl_low in ("person", "people", "human", "face", "me", "user"):
                        self.tracker.set_target_label("person")
                        if frame is not None:
                            locked, bbox, _ = self.tracker.update(frame, detections=self.latest_detections)
                            if locked and bbox:
                                return {
                                    "status": "tracking",
                                    "bbox": list(bbox),
                                    "label": "person",
                                }
                        return {"status": "searching", "label": "person", "message": "Looking for a person in camera view"}

                    # For any other label, ONLY lock on if that object is currently in the camera's view
                    if frame is not None and self.latest_detections:
                        matches = [
                            d for d in self.latest_detections
                            if lbl_low in str(d.get("label", "")).lower()
                            or str(d.get("label", "")).lower() in lbl_low
                        ]
                        if matches:
                            best = max(matches, key=lambda d: float(d.get("conf", 0.0)))
                            started = self.tracker.start_track(frame, tuple(best["bbox"]), label=best.get("label", lbl))
                            return {
                                "status": "tracking" if started else "failed",
                                "bbox": list(self.tracker.target_bbox) if self.tracker.target_bbox else best["bbox"],
                                "label": self.tracker.target_label,
                            }
                    return {
                        "status": "not_in_view",
                        "label": lbl,
                        "message": f"'{lbl}' is not in the camera's view. Tracker only looks for people or a selected object in view.",
                    }

                # Default when called with no args: look for a person in the camera's view
                self.tracker.set_target_label("person")
                if frame is not None:
                    locked, bbox, _ = self.tracker.update(frame, detections=self.latest_detections)
                    if locked and bbox:
                        return {
                            "status": "tracking",
                            "bbox": list(bbox),
                            "label": "person",
                        }
                return {"status": "searching", "label": "person", "message": "Looking for a person in camera view"}

            async def _hooked_stop_tracking(recenter: bool = False, **kw):
                if self.tracker is not None:
                    self.tracker.cancel()
                if recenter and self.pan_tilt is not None:
                    self.pan_tilt.center()
                return {"status": "stopped", "tracking": False, "recentered": bool(recenter)}

            async def _hooked_get_tracked_objects(**kw):
                trk_state = {
                    "active": bool(self.tracker and self.tracker.is_tracking),
                    "bbox": list(self.tracker.target_bbox) if (self.tracker and self.tracker.target_bbox) else None,
                    "target_label": self.tracker.target_label if self.tracker else None,
                }
                return {"tracking": trk_state, "detections": list(self.latest_detections)}

            robot_controller.track_object = _hooked_track_object
            robot_controller.select_object = _hooked_track_object
            robot_controller.stop_tracking = _hooked_stop_tracking
            robot_controller.cancel_tracking = _hooked_stop_tracking
            robot_controller.get_tracked_objects = _hooked_get_tracked_objects

            # 3. Build Capability Map & Tool Registry
            capability_mapper = CapabilityMapper(robot_controller=robot_controller)
            capabilities = capability_mapper.build(hardware_map, physical_config)

            power_manager = SystemPowerManager(robot_controller=robot_controller)
            desktop_controller = DesktopController(power_manager=power_manager)
            tool_registry = ToolRegistry()
            tool_registry.build_default_tools(
                capabilities,
                robot_controller,
                config.safety_limits,
                power_manager=power_manager,
                desktop_controller=desktop_controller,
                hardware_map=hardware_map,
            )

            # 4. Initialize Auth, TrustStore, Session, Event, Task, and Pairing Managers
            self.trust_store = TrustStore()
            authenticator = Authenticator(self.trust_store)
            self.session_manager = SessionManager()
            safety_manager = SafetyManager()
            self.event_manager = EventManager(self.session_manager, identity=self.identity)
            task_manager = TaskManager(self.event_manager, safety_manager)
            self.pairing_manager = PairingManager(self.trust_store, identity=self.identity)
            device_agent = DeviceAgent(tool_registry, safety_manager, task_manager)

            self.dcp_handler = DCPHandler(
                identity=self.identity,
                hardware_mapper=hardware_mapper,
                config=config,
                tool_registry=tool_registry,
                device_agent=device_agent,
                task_manager=task_manager,
                session_manager=self.session_manager,
                trust_store=self.trust_store,
                authenticator=authenticator,
                capabilities=capabilities,
                pairing_manager=self.pairing_manager,
            )

            # 5. Start Wi-Fi WebSocket Server (port 8765)
            self.wifi_server = WiFiDCPServer(self.dcp_handler, self.session_manager, port=self.port)
            await self.wifi_server.start()

            # 6. Start Bluetooth GATT Server (optional fallback)
            try:
                self.bluetooth_server = BluetoothDCPServer(self.dcp_handler, self.session_manager)
                await self.bluetooth_server.start()
            except Exception as e:
                logger.debug("BLE transport start skipped: %s", e)

            # 7. Start mDNS Discovery Advertiser (_dcp._tcp.local.) so the mobile app auto-discovers the robot!
            self.advertiser = DiscoveryAdvertiser(
                device_id=self.identity.device_id,
                profile=self.identity.profile,
                port=self.port,
            )
            self.advertiser.start()

            self._running = True
            local_ip = DiscoveryAdvertiser._local_ip()
            logger.info("====================================================================")
            logger.info("  MACHINE MAKE DCP SERVER READY")
            logger.info("  - Direct Connect IP: %s  (Port: %d)", local_ip, self.port)
            logger.info("  - Device ID:         %s  (Profile: %s)", self.identity.device_id, self.identity.profile)
            logger.info("  - Type 'qr' + Enter in console at any time to generate QR code")
            logger.info("====================================================================")

            if auto_qr:
                try:
                    self.pairing_manager.generate_qr_pairing(client_name="Mobile App", port=self.port)
                except Exception as e:
                    logger.warning("Could not generate initial QR code: %s", e)

        except Exception as exc:
            logger.error("Failed to start MachineMake DCP Server: %s", exc, exc_info=True)

    def generate_qr(self, client_name: str = "Mobile App"):
        """Generates and prints a fresh QR pairing code in the terminal."""
        if self.pairing_manager:
            self.pairing_manager.generate_qr_pairing(client_name=client_name, port=self.port)

    def _read_host_metrics(self):
        """Reads live CPU %, RAM %, GPU %, and SoC Temperature (°C) on Raspberry Pi / Linux / Windows."""
        cpu_val = None
        ram_val = None
        temp_val = None
        gpu_val = None

        # 1. Try psutil for CPU & RAM
        try:
            import psutil
            cpu_val = float(psutil.cpu_percent(interval=None))
            ram_val = float(psutil.virtual_memory().percent)
        except Exception:
            pass

        # 2. Fallback to /proc/stat and /proc/meminfo on Linux if psutil is not installed
        if cpu_val is None and os.path.isfile("/proc/stat"):
            try:
                with open("/proc/stat", "r") as f:
                    parts = [int(x) for x in f.readline().split()[1:]]
                idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
                total = sum(parts)
                diff_total = total - self._prev_cpu_total
                diff_idle = idle - self._prev_cpu_idle
                self._prev_cpu_total = total
                self._prev_cpu_idle = idle
                if diff_total > 0:
                    cpu_val = max(0.0, min(100.0, 100.0 * (1.0 - diff_idle / float(diff_total))))
            except Exception:
                pass

        if ram_val is None and os.path.isfile("/proc/meminfo"):
            try:
                mem_total = 0
                mem_avail = 0
                with open("/proc/meminfo", "r") as f:
                    for line in f:
                        if line.startswith("MemTotal:"):
                            mem_total = int(line.split()[1])
                        elif line.startswith("MemAvailable:"):
                            mem_avail = int(line.split()[1])
                if mem_total > 0:
                    ram_val = max(0.0, min(100.0, 100.0 * (mem_total - mem_avail) / float(mem_total)))
            except Exception:
                pass

        # 3. Read Raspberry Pi / Linux thermal zone temperature (/sys/class/thermal/thermal_zone0/temp)
        if os.path.isfile("/sys/class/thermal/thermal_zone0/temp"):
            try:
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    raw_t = float(f.read().strip())
                temp_val = raw_t / 1000.0 if raw_t > 200.0 else raw_t
            except Exception:
                pass

        # 4. Also try device_server/main.py helpers for cross-platform Temp & GPU
        try:
            from main import get_system_temperature, get_gpu_utilization
            if temp_val is None:
                temp_val = get_system_temperature(default=42.0)
            gpu_val = get_gpu_utilization()
        except Exception:
            pass

        return (
            round(cpu_val if cpu_val is not None else 15.0, 1),
            round(ram_val if ram_val is not None else 35.0, 1),
            round(temp_val if temp_val is not None else 42.0, 1),
            round(gpu_val, 1) if gpu_val is not None else None,
        )

    async def emit_telemetry(
        self,
        imu_data: Dict[str, Any],
        servo_angles: Dict[str, float],
        autonomy_enabled: bool = False,
    ):
        """Pushes real-time telemetry events to subscribed mobile app sessions."""
        if self.robot_controller is not None and isinstance(imu_data, dict):
            self.robot_controller.last_imu_data = dict(imu_data)
            self.robot_controller.orientation = {
                "pitch": float(imu_data.get("pitch", 0.0)),
                "roll": float(imu_data.get("roll", 0.0)),
                "yaw": float(imu_data.get("yaw", 0.0)),
            }

        if not self._running or self.event_manager is None or self.session_manager is None:
            return

        sessions = self.session_manager.all_sessions()
        if not sessions:
            return

        # Ensure all connected sessions receive live telemetry updates
        for s in sessions:
            s.subscribed_events.add("telemetry")

        cpu_val, ram_val, temp_val, gpu_val = self._read_host_metrics()

        payload = {
            "cpu": cpu_val,
            "ram": ram_val,
            "temp": temp_val,
            "cpu_temp": temp_val,
            "gpu": gpu_val,
            "power_source": "dc_external",
            "battery": 88.0,
            "voltage": None,
            "imu": imu_data,
            "servos": servo_angles,
            "autonomy_enabled": bool(autonomy_enabled),
        }
        await self.event_manager.emit("telemetry", payload)

    async def emit_camera_frame(
        self,
        jpeg_bytes: bytes,
        width: int = 640,
        height: int = 480,
        fps: int = 12,
        tracking_info: Optional[Dict[str, Any]] = None,
    ):
        """Pushes compressed JPEG camera frames over DCP WebSocket when camera_streaming is active."""
        if not self._running or self.event_manager is None or self.session_manager is None:
            return
        if self.robot_controller is None or not getattr(self.robot_controller, "camera_streaming", False):
            return
        if not jpeg_bytes:
            return

        sessions = self.session_manager.all_sessions()
        if not sessions:
            return

        # Ensure all active sessions receive the camera stream once streaming is toggled on
        for s in sessions:
            s.subscribed_events.add("camera_frame")

        import base64
        out_jpeg = jpeg_bytes

        # Overlay active tracking bounding box on mobile app stream so user sees locked target on phone
        if tracking_info and tracking_info.get("active") and tracking_info.get("bbox"):
            try:
                import cv2
                import numpy as np
                arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frm = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frm is not None:
                    bx, by, bw, bh = [int(v) for v in tracking_info["bbox"]]
                    cv2.rectangle(frm, (bx, by), (bx + bw, by + bh), (0, 200, 255), 2)
                    lbl = tracking_info.get("target_label") or "TRACKING"
                    cv2.putText(
                        frm,
                        f"[LOCKED] {lbl}",
                        (bx, max(18, by - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 200, 255),
                        2,
                    )
                    ok_enc, buf = cv2.imencode(".jpg", frm, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                    if ok_enc:
                        out_jpeg = buf.tobytes()
            except Exception:
                pass

        b64_str = base64.b64encode(out_jpeg).decode("utf-8")
        payload: Dict[str, Any] = {
            "image_b64": b64_str,
            "format": "jpeg",
            "width": width,
            "height": height,
            "fps": fps,
            "bytes": len(out_jpeg),
        }
        if tracking_info is not None:
            payload["tracking"] = tracking_info
        if self.latest_detections:
            payload["detections"] = self.latest_detections

        await self.event_manager.emit("camera_frame", payload)

    async def stop(self):
        """Stops the DCP server, mDNS advertiser, and BLE peripheral."""
        if self.advertiser:
            try:
                self.advertiser.stop()
            except Exception:
                pass
        if self.bluetooth_server:
            try:
                await self.bluetooth_server.stop()
            except Exception:
                pass
        if self.wifi_server:
            try:
                await self.wifi_server.stop()
            except Exception:
                pass
        self._running = False
        self._prev_cpu_total = 0
        self._prev_cpu_idle = 0
        logger.info("DCP Server Bridge stopped.")
