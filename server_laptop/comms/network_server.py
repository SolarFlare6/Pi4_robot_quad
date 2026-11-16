# server_laptop/comms/network_server.py
"""ZeroMQ + DCP Co-Processing Network Server on Laptop receiving video & telemetry from the Robot.

Features:
1. Resilient ZeroMQ Binding:
   - Binds Port 5555 (SUB) and 5556 (PUB).
   - Automatically bypasses Windows `127.0.0.1:5555` port conflicts caused by Android Emulator (ADB / qemu)
     by binding directly to the Laptop's LAN IPv4 interfaces AND secondary ports 5565 (SUB) / 5566 (PUB).
2. Active `robot_pi` Server Discovery & Recognition:
   - Discovers `robot_pi`'s MachineMake DCP Server (`port 8765`) via mDNS (`_dcp._tcp.local.`),
     fast LAN subnet probing (prioritizing known Pi addresses like `192.168.1.8`), or `--robot-ip`.
   - Sends unicast UDP discovery beacons directly to `<robot_ip>:5557` so `robot_pi`'s ZeroMQ client
     immediately learns the Laptop's exact IP and port.
   - Connects outbound to `robot_pi`'s DCP WebSocket server (`ws://<robot_ip>:8765`) and reverse ZeroMQ
     endpoints (`5568`/`5569`), automatically activating `start_camera` and ingesting `camera_frame` + `telemetry`
     even if Windows Firewall blocks inbound TCP connections on the Laptop.
"""

import asyncio
import base64
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
import logging
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple
import msgpack
import zmq

logger = logging.getLogger("server.network")


class NetworkServer:
    def __init__(
        self,
        ingest_port: int = 5555,
        command_port: int = 5556,
        discovery_port: int = 5557,
        dcp_port: int = 8765,
        robot_ip: Optional[str] = None,
        frame_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self.ingest_port = ingest_port
        self.command_port = command_port
        self.discovery_port = discovery_port
        self.dcp_port = dcp_port
        self.alt_ingest_port = 5565 if ingest_port == 5555 else ingest_port + 10
        self.alt_command_port = 5566 if command_port == 5556 else command_port + 10
        self.robot_Reverse_pub_port = 5568
        self.robot_reverse_sub_port = 5569

        self.active_ingest_port = ingest_port
        self.active_command_port = command_port

        self.discovered_robot_ip: Optional[str] = robot_ip.strip() if robot_ip else None
        self.discovered_robot_name: Optional[str] = None
        self.active_transport: str = "WAITING"
        self.frame_callback = frame_callback

        self._ctx: Optional[zmq.Context] = None
        self._sub_socket: Optional[zmq.Socket] = None
        self._pub_socket: Optional[zmq.Socket] = None
        self._zmq_lock = threading.Lock()
        self._connected_reverse_ips: set = set()

        self._last_zmq_frame_time: float = 0.0
        self._latest_dcp_imu: Dict[str, Any] = {"yaw": 0.0, "pitch": 0.0, "roll": 0.0}
        self._latest_dcp_servos: Dict[str, float] = {}
        self.robot_autonomy_enabled: bool = False
        self._dcp_cmd_queue: List[Dict[str, Any]] = []
        self._last_dcp_nav_cmd: str = ""
        self._last_dcp_nav_time: float = 0.0

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._beacon_thread: Optional[threading.Thread] = None
        self._discovery_thread: Optional[threading.Thread] = None
        self._dcp_thread: Optional[threading.Thread] = None
        self._zeroconf = None
        self._zc_browser = None

    @staticmethod
    def get_local_ip(target_ip: str = "8.8.8.8") -> str:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((target_ip, 80))
            return s.getsockname()[0]
        except Exception:
            return "127.0.0.1"
        finally:
            s.close()

    @classmethod
    def get_all_local_ips(cls) -> List[str]:
        """Returns all non-loopback IPv4 addresses on this machine."""
        ips: List[str] = []
        primary = cls.get_local_ip()
        if primary and not primary.startswith("127."):
            ips.append(primary)
        try:
            hostname = socket.gethostname()
            for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
                ip = info[4][0]
                if ip and not ip.startswith("127.") and ip not in ips:
                    ips.append(ip)
        except Exception:
            pass
        return ips

    def _bind_socket_resilient(
        self,
        sock: zmq.Socket,
        primary_port: int,
        alt_port: int,
        role: str,
    ) -> int:
        """Binds a ZeroMQ socket while gracefully bypassing Windows loopback port conflicts (e.g. Android Emulator ADB on 127.0.0.1:5555)."""
        bound_primary = False
        try:
            sock.bind(f"tcp://0.0.0.0:{primary_port}")
            bound_primary = True
            logger.info("[%s] Bound wildcard tcp://0.0.0.0:%d", role, primary_port)
        except zmq.ZMQError as exc:
            logger.warning(
                "[%s] tcp://0.0.0.0:%d busy (%s -- likely Android Emulator/ADB on 127.0.0.1:%d). "
                "Binding directly to LAN interface IP(s) and fallback port %d...",
                role,
                primary_port,
                exc,
                primary_port,
                alt_port,
            )
            for ip in self.get_all_local_ips():
                try:
                    sock.bind(f"tcp://{ip}:{primary_port}")
                    bound_primary = True
                    logger.info("[%s] Bound LAN interface tcp://%s:%d", role, ip, primary_port)
                except zmq.ZMQError as ip_exc:
                    logger.debug("[%s] Could not bind tcp://%s:%d: %s", role, ip, primary_port, ip_exc)

        # Always also bind the conflict-free secondary port (e.g. 5565 / 5566) on 0.0.0.0
        bound_alt = False
        if alt_port != primary_port:
            try:
                sock.bind(f"tcp://0.0.0.0:{alt_port}")
                bound_alt = True
                logger.info("[%s] Bound secondary wildcard tcp://0.0.0.0:%d", role, alt_port)
            except zmq.ZMQError as alt_exc:
                for ip in self.get_all_local_ips():
                    try:
                        sock.bind(f"tcp://{ip}:{alt_port}")
                        bound_alt = True
                    except zmq.ZMQError:
                        pass
                if not bound_alt:
                    logger.warning("[%s] Could not bind secondary port %d: %s", role, alt_port, alt_exc)

        if bound_primary:
            return primary_port
        if bound_alt:
            return alt_port

        # Last-resort ephemeral port so startup never crashes
        ephemeral_port = sock.bind_to_random_port("tcp://0.0.0.0")
        logger.warning("[%s] Using fallback ephemeral port %d", role, ephemeral_port)
        return ephemeral_port

    def start(self):
        """Initializes sockets, starts ZMQ receiver, UDP beacon, mDNS/LAN robot finder, and DCP bridge."""
        local_ip = self.get_local_ip()
        logger.info(
            "Starting Laptop Performance Server Network Engine on %s (Primary ZMQ: %d/%d, Alt ZMQ: %d/%d, Robot DCP: %d)...",
            local_ip,
            self.ingest_port,
            self.command_port,
            self.alt_ingest_port,
            self.alt_command_port,
            self.dcp_port,
        )
        self._ctx = zmq.Context()

        # SUB socket to ingest robot frames and telemetry
        self._sub_socket = self._ctx.socket(zmq.SUB)
        self._sub_socket.setsockopt(zmq.RCVHWM, 2)
        self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.active_ingest_port = self._bind_socket_resilient(
            self._sub_socket,
            self.ingest_port,
            self.alt_ingest_port,
            role="ZMQ-INGEST-SUB",
        )

        # PUB socket to broadcast directives to robot
        self._pub_socket = self._ctx.socket(zmq.PUB)
        self._pub_socket.setsockopt(zmq.SNDHWM, 5)
        self.active_command_port = self._bind_socket_resilient(
            self._pub_socket,
            self.command_port,
            self.alt_command_port,
            role="ZMQ-CMD-PUB",
        )

        if self.discovered_robot_ip:
            self._connect_reverse_zmq(self.discovered_robot_ip)

        self._running = True
        self._thread = threading.Thread(target=self._ingest_loop, name="ZMQIngestWorker", daemon=True)
        self._thread.start()

        self._beacon_thread = threading.Thread(target=self._udp_beacon_loop, name="UDPBeaconWorker", daemon=True)
        self._beacon_thread.start()

        self._start_mdns_listener()

        self._discovery_thread = threading.Thread(target=self._robot_discovery_loop, name="RobotDiscoveryWorker", daemon=True)
        self._discovery_thread.start()

        self._dcp_thread = threading.Thread(target=self._dcp_bridge_worker, name="DCPClientBridge", daemon=True)
        self._dcp_thread.start()

        logger.info(
            "Performance Server listening on %s (Ingest: %d & %d, Commands: %d & %d) + searching for robot_pi DCP server (port %d)...",
            local_ip,
            self.active_ingest_port,
            self.alt_ingest_port,
            self.active_command_port,
            self.alt_command_port,
            self.dcp_port,
        )

    def _connect_reverse_zmq(self, robot_ip: str):
        """Connects outbound from Laptop to Robot's reverse ZeroMQ ports (5568/5569) to bypass Laptop inbound firewall."""
        if not robot_ip or robot_ip in self._connected_reverse_ips:
            return
        with self._zmq_lock:
            try:
                if self._sub_socket is not None:
                    self._sub_socket.connect(f"tcp://{robot_ip}:{self.robot_Reverse_pub_port}")
                if self._pub_socket is not None:
                    self._pub_socket.connect(f"tcp://{robot_ip}:{self.robot_reverse_sub_port}")
                self._connected_reverse_ips.add(robot_ip)
                logger.info("Connected outbound reverse ZeroMQ endpoints to robot_pi at %s:%d/%d",
                            robot_ip, self.robot_Reverse_pub_port, self.robot_reverse_sub_port)
            except Exception as exc:
                logger.debug("Reverse ZMQ connect failed for %s: %s", robot_ip, exc)

    def _on_robot_discovered(self, ip: str, name: str = "quadruped-pi-01", port: Optional[int] = None):
        if not ip or ip.startswith("127."):
            return
        if port:
            self.dcp_port = int(port)
        is_new = (ip != self.discovered_robot_ip)
        self.discovered_robot_ip = ip
        self.discovered_robot_name = name
        if is_new:
            logger.info(">>> Discovered robot_pi server '%s' at %s:%d! <<<", name, ip, self.dcp_port)
            self._connect_reverse_zmq(ip)
            self._send_direct_udp_wakeup(ip)

    def _start_mdns_listener(self):
        """Listens for `_dcp._tcp.local.` mDNS advertisements published by `robot_pi`."""
        try:
            from zeroconf import ServiceBrowser, ServiceStateChange, Zeroconf

            def _on_service_state_change(zeroconf: Zeroconf, service_type: str, name: str, state_change: ServiceStateChange):
                if state_change not in (ServiceStateChange.Added, ServiceStateChange.Updated):
                    return
                try:
                    info = zeroconf.get_service_info(service_type, name, timeout=1500)
                    if not info:
                        return
                    addresses = info.parsed_scoped_addresses()
                    if not addresses:
                        return
                    ip = addresses[0]
                    props = {
                        (k.decode("utf-8", errors="ignore") if isinstance(k, bytes) else str(k)):
                        (v.decode("utf-8", errors="ignore") if isinstance(v, bytes) else str(v))
                        for k, v in (info.properties or {}).items()
                    }
                    dev_id = props.get("device_id", name)
                    profile = props.get("profile", "")
                    # Ignore if it's our own local IP
                    if ip in self.get_all_local_ips():
                        return
                    logger.info("mDNS discovered DCP device '%s' (profile=%s) at %s:%s", dev_id, profile, ip, info.port)
                    self._on_robot_discovered(ip, name=dev_id, port=info.port or self.dcp_port)
                except Exception as exc:
                    logger.debug("mDNS service info parse error: %s", exc)

            self._zeroconf = Zeroconf()
            self._zc_browser = ServiceBrowser(self._zeroconf, "_dcp._tcp.local.", handlers=[_on_service_state_change])
        except Exception as exc:
            logger.debug("mDNS listener could not start: %s", exc)

    @staticmethod
    def _probe_dcp_port(ip: str, port: int, timeout: float = 0.35) -> bool:
        """Checks if TCP port 8765 is open on candidate IP."""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(timeout)
        try:
            return s.connect_ex((ip, port)) == 0
        except Exception:
            return False
        finally:
            s.close()

    def _robot_discovery_loop(self):
        """Actively scans local subnet(s) for `robot_pi`'s MachineMake DCP server on port 8765."""
        while self._running:
            if self.discovered_robot_ip and self._probe_dcp_port(self.discovered_robot_ip, self.dcp_port, timeout=0.6):
                time.sleep(5.0)
                continue

            local_ips = set(self.get_all_local_ips())
            # Build priority candidate list (check known Pi IPs like 192.168.1.8 first, then active /24 subnets)
            priority_candidates = ["192.168.1.8", "192.168.1.10", "192.168.1.2", "192.168.0.8", "raspberrypi.local", "pi.local"]
            for host in priority_candidates:
                if not self._running:
                    return
                try:
                    resolved = socket.gethostbyname(host)
                    if resolved and resolved not in local_ips and self._probe_dcp_port(resolved, self.dcp_port, timeout=0.35):
                        self._on_robot_discovered(resolved, name=host, port=self.dcp_port)
                        break
                except Exception:
                    pass

            if self.discovered_robot_ip:
                time.sleep(4.0)
                continue

            # Fast parallel scan of local /24 subnet(s) + 192.168.1.0/24 for port 8765
            prefixes = set()
            for lip in local_ips:
                parts = lip.split(".")
                if len(parts) == 4:
                    prefixes.add(f"{parts[0]}.{parts[1]}.{parts[2]}")
            prefixes.add("192.168.1")

            candidate_ips = [
                f"{prefix}.{host_id}"
                for prefix in prefixes
                for host_id in range(1, 255)
                if f"{prefix}.{host_id}" not in local_ips
            ]

            found_ip = None
            with ThreadPoolExecutor(max_workers=48) as pool:
                future_to_ip = {pool.submit(self._probe_dcp_port, cip, self.dcp_port, 0.25): cip for cip in candidate_ips}
                for fut in as_completed(future_to_ip):
                    if not self._running:
                        break
                    cip = future_to_ip[fut]
                    try:
                        if fut.result():
                            found_ip = cip
                            break
                    except Exception:
                        pass

            if found_ip:
                self._on_robot_discovered(found_ip, name="quadruped-pi-01", port=self.dcp_port)

            for _ in range(25):
                if not self._running:
                    break
                time.sleep(0.2)

    def _send_direct_udp_wakeup(self, target_robot_ip: str):
        """Sends a direct unicast UDP discovery packet to `<robot_ip>:5557`."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            best_local_ip = self.get_local_ip(target_robot_ip)
            beacon = json.dumps({
                "service": "robot_perf_server",
                "ip": best_local_ip,
                "ingest_port": self.active_ingest_port,
                "alt_ingest_port": self.alt_ingest_port,
                "command_port": self.active_command_port,
                "alt_command_port": self.alt_command_port,
            }).encode("utf-8")
            sock.sendto(beacon, (target_robot_ip, self.discovery_port))
            sock.close()
        except Exception:
            pass

    def _udp_beacon_loop(self):
        """Broadcasts UDP discovery packet on LAN (and unicast to discovered robot) so the Pi discovers the Laptop IP."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        except Exception:
            pass

        while self._running:
            try:
                all_ips = self.get_all_local_ips() or ["127.0.0.1"]
                for local_ip in all_ips:
                    beacon = json.dumps({
                        "service": "robot_perf_server",
                        "ip": local_ip,
                        "ingest_port": self.active_ingest_port,
                        "alt_ingest_port": self.alt_ingest_port,
                        "command_port": self.active_command_port,
                        "alt_command_port": self.alt_command_port,
                    }).encode("utf-8")
                    try:
                        sock.sendto(beacon, ("255.255.255.255", self.discovery_port))
                    except Exception:
                        pass
                    parts = local_ip.split(".")
                    if len(parts) == 4:
                        subnet_bcast = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
                        if subnet_bcast != "255.255.255.255":
                            try:
                                sock.sendto(beacon, (subnet_bcast, self.discovery_port))
                            except Exception:
                                pass

                # If we know the robot's IP, send a direct unicast beacon to it as well!
                if self.discovered_robot_ip:
                    self._send_direct_udp_wakeup(self.discovered_robot_ip)
            except Exception:
                pass
            time.sleep(1.0)
        sock.close()

    def _ingest_loop(self):
        poller = zmq.Poller()
        if self._sub_socket is not None:
            poller.register(self._sub_socket, zmq.POLLIN)

        while self._running:
            try:
                socks = dict(poller.poll(timeout=100))
                if self._sub_socket in socks and socks[self._sub_socket] == zmq.POLLIN:
                    with self._zmq_lock:
                        raw = self._sub_socket.recv(flags=zmq.NOBLOCK)
                    data = msgpack.unpackb(raw, raw=False)
                    self._last_zmq_frame_time = time.time()
                    if "autonomy_enabled" in data:
                        self.robot_autonomy_enabled = bool(data["autonomy_enabled"])
                    self.active_transport = f"ZMQ ({self.active_ingest_port})"
                    if self.frame_callback and callable(self.frame_callback):
                        self.frame_callback(data)
            except Exception as e:
                if self._running:
                    logger.debug("Error in ingest loop: %s", e)
                time.sleep(0.01)

    def _dcp_bridge_worker(self):
        """Runs an asyncio loop connecting to `robot_pi`'s MachineMake DCP WebSocket server (port 8765)."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._dcp_client_async_loop())
        except Exception as exc:
            logger.debug("DCP bridge worker exited: %s", exc)
        finally:
            try:
                loop.close()
            except Exception:
                pass

    async def _dcp_client_async_loop(self):
        """Maintains a WebSocket connection to `ws://<robot_ip>:8765` to receive camera frames & telemetry and send commands."""
        try:
            import websockets
        except ImportError:
            logger.warning("`websockets` package not installed; DCP WebSocket bridge disabled.")
            return

        req_id = 1
        while self._running:
            robot_ip = self.discovered_robot_ip
            if not robot_ip:
                await asyncio.sleep(0.5)
                continue

            uri = f"ws://{robot_ip}:{self.dcp_port}"
            try:
                logger.info("Connecting to robot_pi DCP Server at %s...", uri)
                async with websockets.connect(uri, ping_interval=10, ping_timeout=10, max_size=8 * 1024 * 1024) as ws:
                    # 1. Read welcome message
                    raw_welcome = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    welcome = json.loads(raw_welcome)
                    dev_name = welcome.get("name") or welcome.get("device_id") or "quadruped-pi-01"
                    self.discovered_robot_name = dev_name
                    logger.info(">>> Connected to robot_pi DCP Server: %s (%s) <<<", dev_name, uri)

                    # 2. Subscribe to camera_frame and telemetry events
                    req_id += 1
                    await ws.send(json.dumps({
                        "dcp": "1.0",
                        "type": "request",
                        "id": req_id,
                        "command": "subscribe",
                        "arguments": {"events": ["camera_frame", "telemetry"]},
                    }))

                    # 3. Request control & start camera stream on robot_pi so it emits `camera_frame` events
                    req_id += 1
                    await ws.send(json.dumps({
                        "dcp": "1.0",
                        "type": "request",
                        "id": req_id,
                        "command": "request_control",
                        "arguments": {},
                    }))

                    req_id += 1
                    await ws.send(json.dumps({
                        "dcp": "1.0",
                        "type": "request",
                        "id": req_id,
                        "command": "start_camera",
                        "arguments": {"resolution": "640x480", "fps": 20},
                    }))

                    while self._running:
                        # Flush any queued DCP commands if ZMQ is not currently the active stream
                        while self._dcp_cmd_queue:
                            cmd_payload = self._dcp_cmd_queue.pop(0)
                            req_id += 1
                            await ws.send(json.dumps({
                                "dcp": "1.0",
                                "type": "request",
                                "id": req_id,
                                "command": "execute_tool",
                                "arguments": cmd_payload,
                            }))

                        try:
                            msg_raw = await asyncio.wait_for(ws.recv(), timeout=0.1)
                        except asyncio.TimeoutError:
                            continue

                        msg = json.loads(msg_raw)
                        if msg.get("type") != "event":
                            continue

                        event_name = msg.get("event")
                        event_data = msg.get("data") or {}

                        if event_name == "telemetry":
                            imu = event_data.get("imu")
                            if isinstance(imu, dict):
                                self._latest_dcp_imu = imu
                            servos = event_data.get("servos")
                            if isinstance(servos, dict):
                                self._latest_dcp_servos = servos
                            if "autonomy_enabled" in event_data:
                                self.robot_autonomy_enabled = bool(event_data["autonomy_enabled"])

                            # If ZMQ is not active and no camera frame arrived yet, still report robot heartbeat
                            if (time.time() - self._last_zmq_frame_time) > 1.2:
                                self.active_transport = f"DCP-WS ({robot_ip}:{self.dcp_port})"
                                cam_x = float(self._latest_dcp_servos.get("0", 90.0))
                                cam_y = float(self._latest_dcp_servos.get("1", 90.0))
                                if self.frame_callback and callable(self.frame_callback):
                                    self.frame_callback({
                                        "timestamp_ms": int(time.time() * 1000),
                                        "has_frame": False,
                                        "frame": b"",
                                        "imu": self._latest_dcp_imu,
                                        "servos": self._latest_dcp_servos,
                                        "cam_x": cam_x,
                                        "cam_y": cam_y,
                                        "autonomy_enabled": self.robot_autonomy_enabled,
                                        "tracker": {"active": False, "bbox": None, "err_x": 0.0, "err_y": 0.0},
                                    })

                        elif event_name == "camera_frame":
                            # Only use DCP camera frame if ZMQ high-speed stream is not currently delivering frames
                            if (time.time() - self._last_zmq_frame_time) <= 1.0:
                                continue

                            b64_img = event_data.get("image_b64")
                            if not b64_img:
                                continue
                            jpeg_bytes = base64.b64decode(b64_img)
                            self.active_transport = f"DCP-WS ({robot_ip}:{self.dcp_port})"
                            cam_x = float(self._latest_dcp_servos.get("0", 90.0))
                            cam_y = float(self._latest_dcp_servos.get("1", 90.0))
                            tracking = event_data.get("tracking") or {"active": False, "bbox": None, "err_x": 0.0, "err_y": 0.0}
                            if self.frame_callback and callable(self.frame_callback):
                                self.frame_callback({
                                    "timestamp_ms": int(time.time() * 1000),
                                    "has_frame": True,
                                    "frame": jpeg_bytes,
                                    "imu": self._latest_dcp_imu,
                                    "servos": self._latest_dcp_servos,
                                    "cam_x": cam_x,
                                    "cam_y": cam_y,
                                    "autonomy_enabled": self.robot_autonomy_enabled,
                                    "tracker": tracking,
                                })

            except Exception as exc:
                logger.debug("DCP WebSocket connection to %s failed/closed: %s", uri, exc)
                await asyncio.sleep(2.0)

    def send_command(self, command_dict: Dict[str, Any]):
        """Pushes navigation command or tracking re-acquisition to the robot over ZeroMQ (and DCP WebSocket fallback)."""
        if not self._running:
            return
        if self._pub_socket is not None:
            try:
                packed = msgpack.packb(command_dict, use_bin_type=True)
                with self._zmq_lock:
                    self._pub_socket.send(packed, flags=zmq.NOBLOCK)
            except Exception as e:
                logger.debug("Failed to send ZMQ command: %s", e)

        # Forward target tracking selection / cancellation over DCP WebSocket as well
        if self.discovered_robot_ip:
            if command_dict.get("cancel_track"):
                self._dcp_cmd_queue.append({"tool": "stop_tracking", "arguments": {"recenter": False}})
            elif command_dict.get("reinit_track"):
                bbox = command_dict["reinit_track"]
                if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                    self._dcp_cmd_queue.append({
                        "tool": "track_object",
                        "arguments": {
                            "x": int(bbox[0]),
                            "y": int(bbox[1]),
                            "w": int(bbox[2]),
                            "h": int(bbox[3]),
                            "label": str(command_dict.get("track_label") or ""),
                        },
                    })

        # ONLY forward locomotion commands over DCP WebSocket fallback when the robot is actually in Autonomous Mode!
        if (
            self.robot_autonomy_enabled
            and (time.time() - self._last_zmq_frame_time) > 1.2
            and self.discovered_robot_ip
        ):
            nav_cmd = command_dict.get("command")
            now = time.time()
            if nav_cmd and (nav_cmd != self._last_dcp_nav_cmd or (now - self._last_dcp_nav_time) > 1.5):
                self._last_dcp_nav_cmd = nav_cmd
                self._last_dcp_nav_time = now
                tool_map = {
                    "walk_forward": ("walk_forward", {"speed": float(command_dict.get("speed", 1.0))}),
                    "turn_left": ("turn_left", {"speed": float(command_dict.get("speed", 0.8))}),
                    "turn_right": ("turn_right", {"speed": float(command_dict.get("speed", 0.8))}),
                    "stop": ("stand", {}),
                }
                if nav_cmd in tool_map and len(self._dcp_cmd_queue) < 3:
                    t_name, t_args = tool_map[nav_cmd]
                    self._dcp_cmd_queue.append({"tool": t_name, "arguments": t_args})

    def stop(self):
        """Stops server and releases sockets."""
        self._running = False
        if self._zeroconf:
            try:
                self._zeroconf.close()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.5)
        if self._beacon_thread and self._beacon_thread.is_alive():
            self._beacon_thread.join(timeout=0.5)
        if self._discovery_thread and self._discovery_thread.is_alive():
            self._discovery_thread.join(timeout=0.5)
        if self._dcp_thread and self._dcp_thread.is_alive():
            self._dcp_thread.join(timeout=0.5)
        with self._zmq_lock:
            if self._sub_socket:
                self._sub_socket.close()
            if self._pub_socket:
                self._pub_socket.close()
        if self._ctx:
            self._ctx.term()
        logger.info("ZeroMQ + DCP NetworkServer stopped.")
