# robot_pi/vision/camera_worker.py
"""Threaded USB Camera Worker (V4L2 / OpenCV VideoCapture).

Automatically discovers connected USB UVC cameras on Raspberry Pi (including via
`/sys/class/video4linux` and `/dev/v4l/by-id/*` to skip Raspberry Pi's internal
bcm2835/rpivid ISP nodes) and reads frames in a non-blocking background thread.
"""

import glob
import logging
import os
import re
import threading
import time
from typing import List, Optional, Tuple
import cv2
import numpy as np

logger = logging.getLogger("robot.camera")


class CameraWorker:
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480, fps: int = 30):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps

        self._cap: Optional[cv2.VideoCapture] = None
        self._device_label: str = "none"
        self._backend = "none"
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._latest_frame: Optional[np.ndarray] = None
        self._latest_timestamp: float = 0.0

    def _discover_usb_video_candidates(self) -> List[int]:
        """Finds V4L2 video device indices on Linux, prioritizing real USB cameras over Pi ISP nodes."""
        usb_indices: List[int] = []
        other_indices: List[int] = []

        # 1. Check /dev/v4l/by-id/* (exclusively created for USB video devices, index0 is the main stream)
        for by_id in sorted(glob.glob("/dev/v4l/by-id/*-index0")) + sorted(glob.glob("/dev/v4l/by-id/*")):
            try:
                real_dev = os.path.realpath(by_id)
                m = re.search(r"video(\d+)$", real_dev)
                if m:
                    idx = int(m.group(1))
                    if idx not in usb_indices:
                        usb_indices.append(idx)
            except Exception:
                pass

        # 2. Inspect /sys/class/video4linux/video*
        for sys_path in sorted(glob.glob("/sys/class/video4linux/video*")):
            m = re.search(r"video(\d+)$", sys_path)
            if not m:
                continue
            idx = int(m.group(1))
            if idx in usb_indices:
                continue

            name_file = os.path.join(sys_path, "name")
            dev_name = ""
            if os.path.isfile(name_file):
                try:
                    with open(name_file, "r") as f:
                        dev_name = f.read().strip().lower()
                except Exception:
                    pass

            # Skip Pi internal hardware codec/ISP nodes
            if any(skip in dev_name for skip in ("bcm2835", "rpivid", "unicam", "pisp", "codec", "isp")):
                continue

            real_sys = os.path.realpath(sys_path).lower()
            if "usb" in real_sys:
                usb_indices.append(idx)
            else:
                other_indices.append(idx)

        # 3. Always include user-configured camera_index and standard 0..4 fallbacks
        fallback = [self.camera_index, 0, 1, 2, 3, 4]
        combined: List[int] = []
        for i in usb_indices + other_indices + fallback:
            if i not in combined and i >= 0:
                combined.append(i)
        return combined

    def _try_open_usb_camera(self, index: int) -> Optional[cv2.VideoCapture]:
        """Tries opening a USB camera index with V4L2/Default backend, testing both default (YUYV) and MJPG."""
        # Try default pixel format first (safest across all USB UVC webcams), then MJPG
        for use_mjpg in (False, True):
            for api in (cv2.CAP_V4L2, cv2.CAP_ANY):
                cap = None
                try:
                    cap = cv2.VideoCapture(index, api)
                    if not cap.isOpened():
                        cap.release()
                        continue

                    if use_mjpg:
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    cap.set(cv2.CAP_PROP_FPS, self.fps)
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                    # USB webcams can take 300-800ms on first open to auto-expose and deliver frame 1
                    for _ in range(15):
                        ret, frame = cap.read()
                        if ret and frame is not None and frame.size > 0:
                            # Store this valid first frame immediately
                            with self._lock:
                                self._latest_frame = frame
                                self._latest_timestamp = time.time()
                            return cap
                        time.sleep(0.05)

                    cap.release()
                except Exception:
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
        return None

    def start(self) -> bool:
        """Discovers and opens the USB camera, then starts the frame capture thread."""
        candidates = self._discover_usb_video_candidates()
        logger.info("Probing USB camera indices in order: %s", candidates)

        for idx in candidates:
            cap = self._try_open_usb_camera(idx)
            if cap is not None:
                self._cap = cap
                self.camera_index = idx
                self._backend = "usb_opencv"
                self._device_label = f"/dev/video{idx}"
                logger.info(
                    ">>> USB Camera opened on %s (index %d, %dx%d @ %d FPS) <<<",
                    self._device_label,
                    idx,
                    self.width,
                    self.height,
                    self.fps,
                )
                break

        if self._backend == "none":
            logger.warning(
                "No USB camera delivered frames on indices %s! Check USB cable or `ls /dev/video*`. "
                "Using synthetic test stream until camera connects.",
                candidates,
            )

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, name="CameraCaptureWorker", daemon=True)
        self._thread.start()
        return True

    def _capture_loop(self):
        fail_count = 0
        last_reprobe_time = time.time()

        while self._running:
            if self._backend == "usb_opencv" and self._cap is not None:
                ret, frame = self._cap.read()
                if ret and frame is not None and frame.size > 0:
                    fail_count = 0
                    with self._lock:
                        self._latest_frame = frame
                        self._latest_timestamp = time.time()
                else:
                    fail_count += 1
                    if fail_count > 50:
                        logger.warning("USB camera on %s disconnected or stalled; will attempt auto-reconnect.", self._device_label)
                        try:
                            self._cap.release()
                        except Exception:
                            pass
                        self._cap = None
                        self._backend = "none"
                    time.sleep(0.02)
            else:
                # Periodically re-probe every 5 seconds in case USB camera was plugged in or reset
                if (time.time() - last_reprobe_time) > 5.0:
                    last_reprobe_time = time.time()
                    for idx in self._discover_usb_video_candidates():
                        cap = self._try_open_usb_camera(idx)
                        if cap is not None:
                            self._cap = cap
                            self.camera_index = idx
                            self._backend = "usb_opencv"
                            self._device_label = f"/dev/video{idx}"
                            logger.info(">>> Reconnected USB Camera on %s! <<<", self._device_label)
                            break

                # Generate synthetic fallback frame so laptop HUD still shows connection status
                synthetic = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                cv2.putText(
                    synthetic,
                    f"Pi Connected - USB Camera Not Detected ({time.strftime('%H:%M:%S')})",
                    (20, 230),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 180, 255),
                    2,
                )
                cv2.putText(
                    synthetic,
                    "Check USB connection on Pi (ls /dev/video*)",
                    (20, 265),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (180, 180, 180),
                    1,
                )
                with self._lock:
                    self._latest_frame = synthetic
                    self._latest_timestamp = time.time()
                time.sleep(1.0 / self.fps)

    def read(self) -> Tuple[bool, Optional[np.ndarray], float]:
        """Returns the most recent frame and capture timestamp."""
        with self._lock:
            if self._latest_frame is not None:
                return True, self._latest_frame.copy(), self._latest_timestamp
            return False, None, 0.0

    def encode_jpeg(self, quality: int = 75) -> Optional[bytes]:
        """Encodes the latest frame as compressed JPEG bytes."""
        success, frame, _ = self.read()
        if not success or frame is None:
            return None
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)]
        ret, buffer = cv2.imencode(".jpg", frame, encode_params)
        if ret:
            return buffer.tobytes()
        return None

    def stop(self):
        """Stops capture thread and releases video device."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._cap and self._cap.isOpened():
            self._cap.release()
            self._cap = None
        logger.info("CameraWorker stopped.")
