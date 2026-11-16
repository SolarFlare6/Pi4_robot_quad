# robot_pi/comms/network_client.py
"""High-speed ZeroMQ Client for the Quadruped Robot with Automatic Laptop LAN Discovery.

Streams compressed camera frames and IMU/Servo telemetry to the Laptop Performance Server (port 5555).
Listens for deep vision detections, target updates, and navigation commands from the Laptop (port 5556).
Automatically listens on UDP port 5557 for the Laptop's discovery beacon so it connects even if no IP was passed.
"""

import json
import logging
import socket
import threading
import time
from typing import Any, Callable, Dict, Optional
import msgpack
import zmq

logger = logging.getLogger("robot.network_client")


class NetworkClient:
    def __init__(
        self,
        server_ip: str = "192.168.1.5",
        stream_port: int = 5555,
        command_port: int = 5556,
        discovery_port: int = 5557,
        command_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self.server_ip = server_ip
        self.stream_port = stream_port
        self.command_port = command_port
        self.discovery_port = discovery_port
        self.command_callback = command_callback

        self._ctx: Optional[zmq.Context] = None
        self._pub_socket: Optional[zmq.Socket] = None
        self._sub_socket: Optional[zmq.Socket] = None
        self._socket_lock = threading.Lock()

        self.is_connected = False
        self._running = False
        self._listener_thread: Optional[threading.Thread] = None
        self._discovery_thread: Optional[threading.Thread] = None

        self.last_command_time: float = 0.0

    def start(self):
        """Initializes ZeroMQ sockets, starts command listener, and starts UDP auto-discovery listener."""
        logger.info("Initializing ZeroMQ Client -> %s:%d/%d", self.server_ip, self.stream_port, self.command_port)
        self._ctx = zmq.Context()

        self._connect_sockets(self.server_ip)

        self._running = True
        self._listener_thread = threading.Thread(target=self._command_listener_loop, name="ZMQCommandListener", daemon=True)
        self._listener_thread.start()

        self._discovery_thread = threading.Thread(target=self._udp_discovery_loop, name="UDPDiscoveryListener", daemon=True)
        self._discovery_thread.start()

        self.is_connected = True
        logger.info("ZeroMQ client started (UDP auto-discovery listening on port %d).", self.discovery_port)

    def _connect_sockets(self, target_ip: str, stream_port: Optional[int] = None, command_port: Optional[int] = None):
        if stream_port is not None:
            self.stream_port = int(stream_port)
        if command_port is not None:
            self.command_port = int(command_port)

        with self._socket_lock:
            if self._pub_socket is not None:
                try:
                    self._pub_socket.close()
                except Exception:
                    pass
            if self._sub_socket is not None:
                try:
                    self._sub_socket.close()
                except Exception:
                    pass

            # Publisher socket to stream telemetry & frames to Laptop
            self._pub_socket = self._ctx.socket(zmq.PUB)
            self._pub_socket.setsockopt(zmq.SNDHWM, 2)
            self._pub_socket.connect(f"tcp://{target_ip}:{self.stream_port}")
            # Also bind reverse endpoint (5568) so Laptop can connect outbound to Pi
            try:
                self._pub_socket.bind("tcp://0.0.0.0:5568")
            except Exception:
                pass

            # Subscriber socket to receive navigation commands & detections from Laptop
            self._sub_socket = self._ctx.socket(zmq.SUB)
            self._sub_socket.setsockopt(zmq.RCVHWM, 5)
            self._sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
            self._sub_socket.connect(f"tcp://{target_ip}:{self.command_port}")
            # Also bind reverse endpoint (5569) so Laptop can publish outbound to Pi
            try:
                self._sub_socket.bind("tcp://0.0.0.0:5569")
            except Exception:
                pass

            self.server_ip = target_ip

    def _udp_discovery_loop(self):
        """Listens for UDP broadcast beacons from the Laptop Performance Server and updates target IP/ports automatically."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.discovery_port))
            sock.settimeout(1.0)
        except Exception as e:
            logger.debug("UDP discovery bind skipped: %s", e)
            sock.close()
            return

        while self._running:
            try:
                data, addr = sock.recvfrom(1024)
                msg = json.loads(data.decode("utf-8"))
                if msg.get("service") == "robot_perf_server":
                    discovered_ip = msg.get("ip") or addr[0]
                    new_stream_port = int(msg.get("ingest_port", self.stream_port))
                    new_cmd_port = int(msg.get("command_port", self.command_port))
                    if (
                        discovered_ip
                        and discovered_ip != "127.0.0.1"
                        and (
                            discovered_ip != self.server_ip
                            or new_stream_port != self.stream_port
                            or new_cmd_port != self.command_port
                        )
                    ):
                        logger.info(
                            ">>> Auto-Discovered Laptop Performance Server at %s:%d/%d (was %s:%d/%d)! Reconnecting ZMQ... <<<",
                            discovered_ip,
                            new_stream_port,
                            new_cmd_port,
                            self.server_ip,
                            self.stream_port,
                            self.command_port,
                        )
                        self._connect_sockets(discovered_ip, new_stream_port, new_cmd_port)
            except socket.timeout:
                continue
            except Exception:
                time.sleep(0.2)
        sock.close()

    def send_telemetry_frame(
        self,
        jpeg_bytes: Optional[bytes],
        imu_data: Dict[str, Any],
        servo_angles: Dict[str, float],
        cam_x: float,
        cam_y: float,
        local_tracker_data: Dict[str, Any],
        autonomy_enabled: bool = False,
    ) -> bool:
        """Pushes telemetry payload and compressed camera frame to the Laptop."""
        if not self._running or self._pub_socket is None:
            return False

        try:
            payload = {
                "timestamp_ms": int(time.time() * 1000),
                "has_frame": jpeg_bytes is not None,
                "frame": jpeg_bytes if jpeg_bytes else b"",
                "imu": imu_data,
                "servos": servo_angles,
                "cam_x": float(cam_x),
                "cam_y": float(cam_y),
                "tracker": local_tracker_data,
                "autonomy_enabled": bool(autonomy_enabled),
            }
            packed = msgpack.packb(payload, use_bin_type=True)
            with self._socket_lock:
                if self._pub_socket is not None:
                    self._pub_socket.send(packed, flags=zmq.NOBLOCK)
            return True
        except zmq.Again:
            return False
        except Exception as exc:
            logger.debug("Failed to send telemetry frame: %s", exc)
            return False

    def _command_listener_loop(self):
        """Background loop reading commands and feedback from the Performance Server."""
        first_contact = False
        while self._running:
            try:
                with self._socket_lock:
                    sub = self._sub_socket
                if sub is None:
                    time.sleep(0.05)
                    continue

                if sub.poll(timeout=100) & zmq.POLLIN:
                    with self._socket_lock:
                        raw = sub.recv(flags=zmq.NOBLOCK)
                    data = msgpack.unpackb(raw, raw=False)
                    self.last_command_time = time.time()
                    if not first_contact:
                        logger.info(">>> Handshake confirmed: Connected to Performance Server at %s! <<<", self.server_ip)
                        first_contact = True

                    if self.command_callback and callable(self.command_callback):
                        self.command_callback(data)
            except zmq.Again:
                continue
            except Exception as e:
                if self._running:
                    logger.debug("Error in command listener: %s", e)
                time.sleep(0.02)

    def is_server_alive(self, timeout_s: float = 1.5) -> bool:
        """Checks if recent commands/pings have been received from the Laptop."""
        return (time.time() - self.last_command_time) < timeout_s

    def stop(self):
        """Closes ZeroMQ connections cleanly."""
        self._running = False
        self.is_connected = False
        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=0.5)
        if self._discovery_thread and self._discovery_thread.is_alive():
            self._discovery_thread.join(timeout=0.5)
        with self._socket_lock:
            if self._pub_socket:
                self._pub_socket.close()
            if self._sub_socket:
                self._sub_socket.close()
        if self._ctx:
            self._ctx.term()
        logger.info("ZeroMQ NetworkClient stopped.")
