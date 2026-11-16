import argparse
import asyncio
import logging
import os
import signal
import sys
from typing import Optional

from auto_config import run_setup_wizard
from auth import Authenticator, TrustStore
from capability_mapper import CapabilityMapper
from config import ConfigurationManager
from desktop_controller import DesktopController
from device_agent import DeviceAgent
from discovery import DiscoveryAdvertiser
from dcp_handler import DCPHandler
from event_manager import EventManager
from hardware_mapper.mapper import HardwareMapper
from identity import DeviceIdentity
from pairing import PairingManager
from robot_controller import RobotController
from safety_manager import SafetyManager
from session import SessionManager
from task_manager import TaskManager
from tool_registry import ToolRegistry
from transports.bluetooth_transport import BluetoothDCPServer
from transports.websocket_transport import WiFiDCPServer

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
logger = logging.getLogger("main")

WIFI_PORT = 8765

# params
#python main.py --auto-detect (enables auto-detection for this run)
#python main.py --setup (launches the interactive setup wizard)
#python main.py --robot (forces quadruped robot mode)
#python main.py --no-robot (disables forced quadruped mode)


# setup vars
automatic_detection = True # Set to True to run the new hardware auto-detection implementation; False runs the old implementation
is_robot_project = False # Set to True to configure the server for your Quadruped Robot project

# Uppercase aliases for convenience:
AUTOMATIC_DETECTION = automatic_detection
IS_ROBOT_PROJECT = is_robot_project


def configure_robot_project_config(config_file: Optional[str] = None, custom_name: Optional[str] = None) -> ConfigurationManager:
    """Configures and writes device_config.json with the full Quadruped Robot configuration."""
    from auto_config import build_config_for_profile
    mgr = ConfigurationManager(config_file) if config_file else ConfigurationManager()
    quad_name = custom_name or mgr.identity.get("name") or "MachineMake Quadruped Dog"
    robot_cfg = build_config_for_profile("quadruped", custom_name=quad_name)
    existing_id = mgr.identity.get("device_id")
    if existing_id:
        robot_cfg["identity"]["device_id"] = existing_id
    mgr._config = robot_cfg
    mgr.save()
    logger.info("Configured %s for Quadruped Robot.", mgr._config_file)
    return mgr


async def console_loop(
    pairing_manager: PairingManager,
    stop_event: asyncio.Event,
    trust_store: TrustStore,
    session_manager: SessionManager,
    identity: DeviceIdentity,
) -> None:
    """Non-blocking interactive console reader so the user can input commands
    (like 'qr' to generate a pairing QR code) while the server is running."""
    loop = asyncio.get_running_loop()

    # If stdin is not an interactive terminal (e.g. background service), do not block
    if not sys.stdin.isatty():
        return

    banner = (
        "\n" + "=" * 68 + "\n"
        "  MACHINE MAKE DEVICE SERVER CONSOLE\n"
        "  Type 'qr' to generate a pairing QR code for the mobile app.\n"
        "  Commands:\n"
        "    qr [name]    - Generate & display QR pairing code (ASCII + PNG)\n"
        "    devices      - List paired devices and active sessions\n"
        "    status       - Show device status and server info\n"
        "    help         - Show available commands\n"
        "    exit / quit  - Stop the server\n"
        + "=" * 68 + "\n"
    )
    print(banner)

    while not stop_event.is_set():
        try:
            line = await loop.run_in_executor(None, lambda: input("[server]> "))
        except (EOFError, KeyboardInterrupt):
            stop_event.set()
            break
        except Exception:
            break

        if stop_event.is_set():
            break

        cmd_parts = line.strip().split(maxsplit=1)
        if not cmd_parts:
            continue
        cmd = cmd_parts[0].lower()
        arg = cmd_parts[1] if len(cmd_parts) > 1 else ""

        if cmd in ("exit", "quit", "q"):
            stop_event.set()
            break
        elif cmd == "qr":
            client_name = arg or "Mobile App"
            pairing_manager.generate_qr_pairing(client_name=client_name)
        elif cmd in ("devices", "paired"):
            print("\n--- Paired Devices (TrustStore) ---")
            trusted = trust_store._trusted
            if not trusted:
                print("  No devices paired yet. Run 'qr' to pair a mobile app.")
            for cid, data in trusted.items():
                print(f"  - {cid}: {data.get('name')} (permission: {data.get('permission')})")
            print(f"--- Active Sessions: {len(session_manager.all_sessions())} ---")
            for s in session_manager.all_sessions():
                print(f"  - Session {s.session_id}: client={s.client_id} auth={s.authenticated} control={s.has_control}")
            print()
        elif cmd == "status":
            print(f"\nDevice ID: {identity.device_id}")
            print(f"Name:      {identity.name}")
            print(f"Profile:   {identity.profile}")
            print(f"Active sessions: {len(session_manager.all_sessions())}\n")
        elif cmd == "help":
            print("\nAvailable commands:")
            print("  qr [name]    - Generate pairing QR code and display in terminal")
            print("  devices      - List paired devices in TrustStore and active sessions")
            print("  status       - Show device identity & status")
            print("  exit / quit  - Stop the server\n")
        else:
            print(f"Unknown command '{cmd}'. Type 'qr' to generate QR code, or 'help' for options.")


_NVML_INITIALIZED = False
_NVML_FAILED = False

_WIN_PDH_INITIALIZED = False
_WIN_PDH_FAILED = False
_WIN_PDH_QUERY = None
_WIN_PDH_COUNTER_HP = None
_WIN_PDH_COUNTER_STD = None


def _get_windows_pdh_temperature() -> Optional[float]:
    """
    Fast, in-process Windows Thermal Zone temperature query via Win32 PDH (ctypes.windll.pdh).
    Requires no third-party dependencies or admin elevation (~0.2ms per call).
    """
    global _WIN_PDH_INITIALIZED, _WIN_PDH_FAILED, _WIN_PDH_QUERY, _WIN_PDH_COUNTER_HP, _WIN_PDH_COUNTER_STD
    if sys.platform != "win32" or _WIN_PDH_FAILED:
        return None

    try:
        import ctypes
        from ctypes import wintypes

        pdh = ctypes.windll.pdh
        PDH_FMT_DOUBLE = 0x00000200

        class PDH_FMT_COUNTERVALUE_DOUBLE(ctypes.Structure):
            _fields_ = [("CStatus", wintypes.DWORD), ("doubleValue", ctypes.c_double)]

        if not _WIN_PDH_INITIALIZED:
            h_query = wintypes.HANDLE()
            if pdh.PdhOpenQueryW(None, 0, ctypes.byref(h_query)) != 0:
                _WIN_PDH_FAILED = True
                return None
            _WIN_PDH_QUERY = h_query

            h_hp = wintypes.HANDLE()
            path_hp = r"\Thermal Zone Information(*)\High Precision Temperature"
            if pdh.PdhAddEnglishCounterW(_WIN_PDH_QUERY, path_hp, 0, ctypes.byref(h_hp)) == 0:
                _WIN_PDH_COUNTER_HP = h_hp

            h_std = wintypes.HANDLE()
            path_std = r"\Thermal Zone Information(*)\Temperature"
            if pdh.PdhAddEnglishCounterW(_WIN_PDH_QUERY, path_std, 0, ctypes.byref(h_std)) == 0:
                _WIN_PDH_COUNTER_STD = h_std

            if _WIN_PDH_COUNTER_HP is None and _WIN_PDH_COUNTER_STD is None:
                pdh.PdhCloseQuery(_WIN_PDH_QUERY)
                _WIN_PDH_QUERY = None
                _WIN_PDH_FAILED = True
                return None

            _WIN_PDH_INITIALIZED = True

        if pdh.PdhCollectQueryData(_WIN_PDH_QUERY) != 0:
            return None

        val = PDH_FMT_COUNTERVALUE_DOUBLE()
        if _WIN_PDH_COUNTER_HP is not None:
            if pdh.PdhGetFormattedCounterValue(_WIN_PDH_COUNTER_HP, PDH_FMT_DOUBLE, None, ctypes.byref(val)) == 0:
                raw = val.doubleValue
                # High Precision Temperature is in tenths of Kelvin (e.g. 3412 -> 341.2 K -> 68.05 C),
                # or occasionally directly in Kelvin if < 500
                temp_c = (raw / 10.0 - 273.15) if raw > 1000 else (raw - 273.15)
                if 0.0 < temp_c < 130.0:
                    return round(temp_c, 1)

        if _WIN_PDH_COUNTER_STD is not None:
            if pdh.PdhGetFormattedCounterValue(_WIN_PDH_COUNTER_STD, PDH_FMT_DOUBLE, None, ctypes.byref(val)) == 0:
                raw = val.doubleValue
                temp_c = (raw / 10.0 - 273.15) if raw > 1000 else (raw - 273.15)
                if 0.0 < temp_c < 130.0:
                    return round(temp_c, 1)
    except Exception:
        _WIN_PDH_FAILED = True

    return None


def get_system_temperature(default: Optional[float] = 45.0) -> Optional[float]:
    """
    Cross-platform CPU / SoC temperature reader for Windows, macOS, and Linux.
    Strictly reads CPU / SoC thermal sensors (excluding GPU sensors).
    Tries platform-native thermal interfaces in order of accuracy and speed before falling back.
    """
    _EXCLUDE_KEYWORDS = ("gpu", "amdgpu", "nouveau", "radeon", "nvme", "wifi", "battery")

    # 1. Linux / FreeBSD: psutil.sensors_temperatures()
    try:
        import psutil
        if hasattr(psutil, "sensors_temperatures"):
            temps = psutil.sensors_temperatures() or {}
            if temps:
                # Prefer primary CPU / SoC thermal drivers
                preferred_keys = (
                    "coretemp", "k10temp", "zenpower", "cpu_thermal",
                    "soc_thermal", "cpu-thermal", "acpitz", "arm-scmi", "rp1_adc",
                )
                for key in preferred_keys:
                    if key in temps and temps[key]:
                        entries = temps[key]
                        # Check for a package/die label first
                        for entry in entries:
                            lbl = (entry.label or "").lower()
                            if any(k in lbl for k in ("package", "tctl", "tdie", "cpu", "soc")):
                                if 0.0 < entry.current < 130.0:
                                    return round(float(entry.current), 1)
                        valid = [
                            float(e.current) for e in entries
                            if 0.0 < e.current < 130.0 and not any(ex in (e.label or "").lower() for ex in _EXCLUDE_KEYWORDS)
                        ]
                        if valid:
                            return round(sum(valid) / len(valid), 1)

                # Fallback to any valid CPU sensor reported by psutil (strictly excluding GPUs and peripherals)
                for key, entries in temps.items():
                    k_lower = key.lower()
                    if any(ex in k_lower for ex in _EXCLUDE_KEYWORDS):
                        continue
                    valid = [
                        float(e.current) for e in entries
                        if 0.0 < e.current < 130.0 and not any(ex in (e.label or "").lower() for ex in _EXCLUDE_KEYWORDS)
                    ]
                    if valid:
                        return round(sum(valid) / len(valid), 1)
    except Exception:
        pass

    # 2. Linux / Raspberry Pi sysfs thermal zones & hwmon (strictly excluding GPU zones)
    if sys.platform.startswith("linux"):
        try:
            import glob, os
            thermal_zone_dirs = sorted(glob.glob("/sys/class/thermal/thermal_zone*"))
            for tz in thermal_zone_dirs:
                type_file = os.path.join(tz, "type")
                temp_file = os.path.join(tz, "temp")
                if os.path.exists(type_file):
                    try:
                        with open(type_file, "r") as tf:
                            tz_type = tf.read().strip().lower()
                        if any(ex in tz_type for ex in _EXCLUDE_KEYWORDS):
                            continue
                    except Exception:
                        pass
                if os.path.exists(temp_file):
                    try:
                        with open(temp_file, "r") as f:
                            raw = float(f.read().strip())
                            temp_c = raw / 1000.0 if raw > 200.0 else raw
                            if 0.0 < temp_c < 130.0:
                                return round(temp_c, 1)
                    except Exception:
                        continue

            hwmon_dirs = sorted(glob.glob("/sys/class/hwmon/hwmon*"))
            for hwd in hwmon_dirs:
                name_file = os.path.join(hwd, "name")
                if os.path.exists(name_file):
                    try:
                        with open(name_file, "r") as nf:
                            dev_name = nf.read().strip().lower()
                        if any(ex in dev_name for ex in _EXCLUDE_KEYWORDS):
                            continue
                    except Exception:
                        pass
                for temp_path in sorted(glob.glob(os.path.join(hwd, "temp*_input"))):
                    try:
                        with open(temp_path, "r") as f:
                            raw = float(f.read().strip())
                            temp_c = raw / 1000.0 if raw > 200.0 else raw
                            if 0.0 < temp_c < 130.0:
                                return round(temp_c, 1)
                    except Exception:
                        continue
        except Exception:
            pass

    # 3. Windows: Win32 PDH Thermal Zone counters or CPU WMI
    if sys.platform == "win32":
        # In-process Win32 PDH ACPI thermal zone query
        pdh_temp = _get_windows_pdh_temperature()
        if pdh_temp is not None:
            return pdh_temp

        # Try WMI (LibreHardwareMonitor / OpenHardwareMonitor strictly for CPU sensors)
        try:
            import wmi  # type: ignore
            for ns in (r"root\LibreHardwareMonitor", r"root\OpenHardwareMonitor"):
                try:
                    w = wmi.WMI(namespace=ns)
                    sensors = w.Sensor(SensorType="Temperature")
                    cpu_temps = [
                        float(s.Value) for s in sensors
                        if s.Value is not None and "cpu" in (s.Name or "").lower() and 0.0 < float(s.Value) < 130.0
                    ]
                    if cpu_temps:
                        return round(sum(cpu_temps) / len(cpu_temps), 1)
                except Exception:
                    continue
        except Exception:
            pass

        # Fallback on Windows: query ACPI ThermalZone via CIM/PowerShell
        try:
            import subprocess
            out = subprocess.check_output(
                ["powershell", "-NoProfile", "-Command",
                 "(Get-CimInstance Win32_PerfFormattedData_Counters_ThermalZoneInformation).HighPrecisionTemperature"],
                timeout=1.5,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            first_line = out.splitlines()[0].strip() if out else ""
            if first_line.isdigit():
                raw = float(first_line)
                temp_c = (raw / 10.0 - 273.15) if raw > 1000 else (raw - 273.15)
                if 0.0 < temp_c < 130.0:
                    return round(temp_c, 1)
        except Exception:
            pass

        try:
            import subprocess
            out = subprocess.check_output(
                ["powershell", "-NoProfile", "-Command",
                 "(Get-CimInstance Win32_PerfFormattedData_Counters_ThermalZoneInformation).Temperature"],
                timeout=1.5,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            first_line = out.splitlines()[0].strip() if out else ""
            if first_line.isdigit():
                raw = float(first_line)
                temp_c = (raw / 10.0 - 273.15) if raw > 1000 else (raw - 273.15)
                if 0.0 < temp_c < 130.0:
                    return round(temp_c, 1)
        except Exception:
            pass

    # 4. macOS (Darwin): osx-cpu-temp / smctemp CLI or AppleSmartBattery SMC sensor via ioreg
    if sys.platform == "darwin":
        try:
            import re, shutil, subprocess
            for cli in ("osx-cpu-temp", "smctemp"):
                if shutil.which(cli):
                    args = [cli, "-c"] if cli == "smctemp" else [cli]
                    out = subprocess.check_output(
                        args, timeout=1.0, text=True, stderr=subprocess.DEVNULL
                    ).strip()
                    m = re.search(r"(\d+(?:\.\d+)?)", out)
                    if m:
                        val = float(m.group(1))
                        if 0.0 < val < 130.0:
                            return round(val, 1)

            # Unprivileged macOS fallback via AppleSmartBattery IOKit registry
            if shutil.which("ioreg"):
                out = subprocess.check_output(
                    ["ioreg", "-r", "-n", "AppleSmartBattery", "-d", "1"],
                    timeout=1.0,
                    text=True,
                    stderr=subprocess.DEVNULL,
                )
                m = re.search(r'"Temperature"\s*=\s*(\d+)', out)
                if m:
                    raw = float(m.group(1))
                    temp_c = (raw / 10.0 - 273.15) if 2730 <= raw <= 3800 else (raw / 100.0)
                    if 0.0 < temp_c < 130.0:
                        return round(temp_c, 1)
        except Exception:
            pass

    return default


# Alias for explicit CPU temperature querying
get_cpu_temperature = get_system_temperature


def get_gpu_utilization() -> Optional[float]:
    """
    Best-effort GPU utilization query across platforms (NVIDIA NVML/CLI, AMD/Intel DRM sysfs).
    Returns utilization percentage (0.0 - 100.0) or None if no GPU is detectable/supported.
    """
    global _NVML_INITIALIZED, _NVML_FAILED

    # 1. NVIDIA NVML via pynvml (fastest, in-process, zero subprocess overhead)
    if not _NVML_FAILED:
        try:
            import pynvml
            if not _NVML_INITIALIZED:
                pynvml.nvmlInit()
                _NVML_INITIALIZED = True
            device_count = pynvml.nvmlDeviceGetCount()
            if device_count > 0:
                total_util = 0.0
                for i in range(device_count):
                    handle = pynvml.nvmlDeviceGetHandleByIndex(i)
                    rates = pynvml.nvmlDeviceGetUtilizationRates(handle)
                    total_util += rates.gpu
                return round(total_util / device_count, 1)
        except Exception:
            _NVML_FAILED = True

    # 2. NVIDIA CLI (nvidia-smi fallback)
    try:
        import shutil, subprocess
        if shutil.which("nvidia-smi"):
            out = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=utilization.gpu", "--format=csv,noheader,nounits"],
                timeout=1.0,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            lines = [line.strip() for line in out.splitlines() if line.strip()]
            if lines:
                vals = [float(v) for v in lines if v.replace(".", "", 1).isdigit()]
                if vals:
                    return round(sum(vals) / len(vals), 1)
    except Exception:
        pass

    # 3. Linux AMD/Intel GPU DRM sysfs (/sys/class/drm/card*/device/gpu_busy_percent)
    try:
        import glob
        busy_files = glob.glob("/sys/class/drm/card*/device/gpu_busy_percent")
        if busy_files:
            vals = []
            for path in busy_files:
                try:
                    with open(path, "r") as f:
                        content = f.read().strip()
                        if content.isdigit():
                            vals.append(float(content))
                except Exception:
                    pass
            if vals:
                return round(sum(vals) / len(vals), 1)
    except Exception:
        pass

    return None


_PREV_CPU_TOTAL = 0
_PREV_CPU_IDLE = 0


def collect_telemetry_snapshot(robot_controller: Optional[RobotController] = None) -> dict:
    """Builds a complete system + hardware telemetry snapshot dictionary."""
    global _PREV_CPU_TOTAL, _PREV_CPU_IDLE
    cpu_val = None
    ram_val = None
    temp_val = get_system_temperature(default=45.0)
    gpu_val = get_gpu_utilization()
    power_source = "dc_external"
    battery_pct = None

    try:
        import psutil
        cpu_val = float(psutil.cpu_percent(interval=None))
        ram_val = float(psutil.virtual_memory().percent)
        batt = psutil.sensors_battery()
        if batt is not None:
            battery_pct = round(float(batt.percent), 1)
            power_source = "ac_power" if batt.power_plugged else "battery"
    except Exception:
        pass

    # Fallback to Linux /proc/stat & /proc/meminfo when psutil is not installed
    if cpu_val is None and os.path.isfile("/proc/stat"):
        try:
            with open("/proc/stat", "r") as f:
                parts = [int(x) for x in f.readline().split()[1:]]
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
            total = sum(parts)
            diff_total = total - _PREV_CPU_TOTAL
            diff_idle = idle - _PREV_CPU_IDLE
            _PREV_CPU_TOTAL = total
            _PREV_CPU_IDLE = idle
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

    cpu_val = cpu_val if cpu_val is not None else 0.0
    ram_val = ram_val if ram_val is not None else 0.0

    imu_data = robot_controller.get_mpu6050_telemetry() if robot_controller else {
        "pitch": 0.42,
        "roll": -1.15,
        "yaw": 0.0,
        "accel": {"x": 0.01, "y": -0.04, "z": 0.99},
        "gyro": {"x": 0.2, "y": -0.5, "z": 0.1},
    }

    servo_data = robot_controller.get_servo_angles_dict() if robot_controller else {
        "2": 110, "3": 70, "4": 50,
        "5": 110, "6": 110, "7": 150,
        "8": 130, "9": 70, "10": 40,
        "11": 80, "12": 90, "13": 160,
    }

    return {
        "cpu": round(cpu_val, 1),
        "ram": round(ram_val, 1),
        "temp": round(temp_val, 1) if temp_val is not None else 45.0,
        "cpu_temp": round(temp_val, 1) if temp_val is not None else 45.0,
        "gpu": round(gpu_val, 1) if gpu_val is not None else None,
        "power_source": power_source,
        "battery": battery_pct,
        "voltage": None,
        "imu": imu_data,
        "servos": servo_data,
    }


async def telemetry_loop(
    event_manager: EventManager,
    session_manager: SessionManager,
    stop_event: asyncio.Event,
    interval_s: float = 2.0,
    robot_controller: Optional[RobotController] = None,
) -> None:
    """Periodically emits telemetry events to connected sessions subscribed to 'telemetry'."""
    while not stop_event.is_set():
        try:
            await asyncio.sleep(interval_s)
            if not session_manager.sessions_subscribed_to("telemetry"):
                continue

            telemetry_data = collect_telemetry_snapshot(robot_controller=robot_controller)
            await event_manager.emit("telemetry", telemetry_data)
        except asyncio.CancelledError:
            break
        except Exception as exc:
            logger.debug("telemetry loop error: %s", exc)


async def camera_stream_loop(
    event_manager: EventManager,
    session_manager: SessionManager,
    stop_event: asyncio.Event,
    robot_controller: Optional[RobotController] = None,
) -> None:
    """Streams JPEG frames to mobile app sessions when standalone device_server/main.py is used."""
    camera_worker = None
    try:
        parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        if parent_dir not in sys.path:
            sys.path.insert(0, parent_dir)

        while not stop_event.is_set():
            if robot_controller is None or not getattr(robot_controller, "camera_streaming", False):
                await asyncio.sleep(0.25)
                continue

            sessions = session_manager.all_sessions()
            if not sessions:
                await asyncio.sleep(0.25)
                continue

            if camera_worker is None:
                try:
                    from vision.camera_worker import CameraWorker
                    camera_worker = CameraWorker(camera_index=0, width=640, height=480, fps=30)
                    camera_worker.start()
                except Exception as exc:
                    logger.warning("Could not initialize CameraWorker in standalone server: %s", exc)
                    await asyncio.sleep(1.0)
                    continue

            for s in sessions:
                s.subscribed_events.add("camera_frame")

            jpg = camera_worker.encode_jpeg(quality=75)
            if jpg:
                import base64
                b64_str = base64.b64encode(jpg).decode("utf-8")
                await event_manager.emit(
                    "camera_frame",
                    {
                        "image_b64": b64_str,
                        "format": "jpeg",
                        "width": camera_worker.width,
                        "height": camera_worker.height,
                        "fps": 12,
                        "bytes": len(jpg),
                    },
                )
            await asyncio.sleep(0.08)
    except asyncio.CancelledError:
        pass
    finally:
        if camera_worker is not None:
            try:
                camera_worker.stop()
            except Exception:
                pass


async def run(
    auto_qr: bool = False,
    qr_name: str = "Mobile App",
    port: int = WIFI_PORT,
    pairing_manager: PairingManager = None,
    trust_store: TrustStore = None,
    device_id: Optional[str] = None,
    device_name: Optional[str] = None,
    setup_mode: bool = False,
    auto_detect: Optional[bool] = None,
    robot_project: Optional[bool] = None,
) -> None:
    # Resolve operating mode: explicit argument overrides top-level variable
    should_be_robot = is_robot_project if robot_project is None else robot_project
    should_auto_detect = automatic_detection if auto_detect is None else auto_detect

    # 1. Hardware Mapper: scan available hardware
    hardware_mapper = HardwareMapper()
    hardware_map = hardware_mapper.scan()
    logger.info("hardware scan complete; interfaces=%s", hardware_map["interfaces"])
    if hardware_map["detector_errors"]:
        logger.warning("some detectors were unavailable: %s", hardware_map["detector_errors"])

    config = ConfigurationManager()

    # 2. Operating Mode Routing:
    if should_be_robot:
        # User's Robot Quadruped Project Mode:
        logger.info("Mode: is_robot_project=True -> setting up server specifically for Quadruped Robot")
        identity_cfg = config.identity
        quad_name = device_name or identity_cfg.get("name") or "MachineMake Quadruped Dog"
        identity = DeviceIdentity(
            name=quad_name,
            profile="quadruped",
            device_type="robot",
            device_id=device_id or identity_cfg.get("device_id"),
        )

        # Configure and persist device_config.json for the Quadruped Robot
        from auto_config import build_config_for_profile
        robot_cfg = build_config_for_profile("quadruped", custom_name=quad_name)
        if identity.device_id:
            robot_cfg["identity"]["device_id"] = identity.device_id
        config._config = robot_cfg
        config.save()
        logger.info("Configured %s with Quadruped Robot configuration.", config._config_file)

        physical_config = dict(config.physical_configuration)

    elif should_auto_detect or setup_mode or not os.path.isfile(config._config_file):
        # New implementation: Hardware-scan auto-detection / Setup wizard
        is_interactive = setup_mode or (not should_auto_detect and sys.stdin.isatty())
        logger.info("Mode: automatic_detection=True -> running new hardware auto-detection (interactive=%s)", is_interactive)
        run_setup_wizard(hardware_map, interactive=is_interactive, save_path=config._config_file)
        config.reload()

        identity_cfg = config.identity
        identity = DeviceIdentity(
            name=device_name or identity_cfg.get("name", "Unnamed Device"),
            profile=identity_cfg.get("profile", "generic"),
            device_type=identity_cfg.get("type", "device"),
            device_id=device_id or identity_cfg.get("device_id"),
        )
        physical_config = config.physical_configuration

    else:
        # Old implementation: Load static device_config.json directly without auto-detection
        logger.info("Mode: automatic_detection=False -> running standard/old implementation from device_config.json")
        identity_cfg = config.identity
        identity = DeviceIdentity(
            name=device_name or identity_cfg.get("name", "Unnamed Device"),
            profile=identity_cfg.get("profile", "generic"),
            device_type=identity_cfg.get("type", "device"),
            device_id=device_id or identity_cfg.get("device_id"),
        )
        physical_config = config.physical_configuration

    logger.info("device identity: %s (profile=%s, type=%s, name='%s')", identity.device_id, identity.profile, identity.device_type, identity.name)

    # 4. Physical configuration
    physical_config = physical_config or {}

    is_computer = (
        identity.device_type == "computer"
        or identity.profile in ("computer", "laptop", "desktop", "mac", "pc", "workstation")
    )
    has_robot_hw = bool(physical_config.get("servos") or physical_config.get("pca9685"))

    # Robot Controller: only instantiated when this is a robot or has robot hardware
    if not is_computer or has_robot_hw:
        robot_controller = RobotController(physical_config, hardware_map=hardware_map)
    else:
        robot_controller = None

    # 5. Capability Map
    capability_mapper = CapabilityMapper(robot_controller=robot_controller)
    capabilities = capability_mapper.build(hardware_map, physical_config)
    logger.info("capabilities: %s", capabilities)

    # 6. Tool Registry
    from system_power import SystemPowerManager
    power_manager = SystemPowerManager(robot_controller=robot_controller)
    desktop_controller = DesktopController(power_manager=power_manager)
    tool_registry = ToolRegistry()
    tool_registry.build_default_tools(
        capabilities, robot_controller, config.safety_limits,
        power_manager=power_manager,
        desktop_controller=desktop_controller,
        hardware_map=hardware_map,
    )
    logger.info("registered tools: %s", list(tool_registry.all_tools().keys()))

    # Supporting managers used by the DCP Handler / Device Agent pipeline.
    trust_store = trust_store or TrustStore()
    authenticator = Authenticator(trust_store)
    session_manager = SessionManager()
    safety_manager = SafetyManager()
    event_manager = EventManager(session_manager, identity=identity)
    task_manager = TaskManager(event_manager, safety_manager)
    device_agent = DeviceAgent(tool_registry, safety_manager, task_manager)
    if pairing_manager is None:
        pairing_manager = PairingManager(trust_store, identity=identity)
    else:
        pairing_manager.set_identity(identity)

    dcp_handler = DCPHandler(
        identity=identity,
        hardware_mapper=hardware_mapper,
        config=config,
        tool_registry=tool_registry,
        device_agent=device_agent,
        task_manager=task_manager,
        session_manager=session_manager,
        trust_store=trust_store,
        authenticator=authenticator,
        capabilities=capabilities,
        pairing_manager=pairing_manager,
    )

    # 9. Start Wi-Fi DCP server (WebSocket over TCP)
    wifi_server = WiFiDCPServer(dcp_handler, session_manager, port=port)
    await wifi_server.start()

    # 10. Start Bluetooth DCP server (GATT peripheral with fragmentation/reassembly)
    bluetooth_server = BluetoothDCPServer(dcp_handler, session_manager)
    await bluetooth_server.start()

    # 11. Discovery / advertising (mDNS + BLE)
    advertiser = DiscoveryAdvertiser(
        device_id=identity.device_id,
        profile=identity.profile,
        port=port,
    )
    advertiser.start()

    logger.info("Device Server ready. Waiting for clients on port %s.", port)

    # Optional immediate QR code generation if requested via CLI flag
    if auto_qr:
        pairing_manager.generate_qr_pairing(client_name=qr_name, port=port)

    stop_event = asyncio.Event()

    def _handle_signal():
        stop_event.set()

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, _handle_signal)
        except NotImplementedError:
            pass  # e.g. Windows

    # Start non-blocking interactive console reader
    console_task = asyncio.create_task(
        console_loop(pairing_manager, stop_event, trust_store, session_manager, identity)
    )

    # Start periodic telemetry emitter
    telemetry_task = asyncio.create_task(
        telemetry_loop(event_manager, session_manager, stop_event, robot_controller=robot_controller)
    )
    camera_task = asyncio.create_task(
        camera_stream_loop(event_manager, session_manager, stop_event, robot_controller=robot_controller)
    )

    await stop_event.wait()

    logger.info("shutting down")
    camera_task.cancel()
    telemetry_task.cancel()
    console_task.cancel()
    advertiser.stop()
    await bluetooth_server.stop()
    await wifi_server.stop()


def main():
    parser = argparse.ArgumentParser(description="Machine Make Device Server")
    parser.add_argument("--qr", action="store_true", help="Generate and display a pairing QR code on startup")
    parser.add_argument("--qr-name", default="Mobile App", help="Client name for QR pairing invitation")
    parser.add_argument("--port", type=int, default=WIFI_PORT, help="Wi-Fi WebSocket port (default: 8765)")
    parser.add_argument("--device-id", default=None, help="Explicit device ID (e.g. quadruped-pi-01)")
    parser.add_argument("--name", default=None, help="Device display name (e.g. MachineMake Quadruped Dog)")
    parser.add_argument("--setup", action="store_true", help="Run interactive initial setup wizard to detect & configure device")
    parser.add_argument("--auto-detect", action="store_true", default=None, help="Automatically infer and save device configuration from hardware scan")
    parser.add_argument("--robot", action="store_true", default=None, help="Force server setup for quadruped robot project")
    parser.add_argument("--no-robot", action="store_false", dest="robot", help="Disable forced robot quadruped setup")
    args = parser.parse_args()

    try:
        asyncio.run(run(
            auto_qr=args.qr,
            qr_name=args.qr_name,
            port=args.port,
            device_id=args.device_id,
            device_name=args.name,
            setup_mode=args.setup,
            auto_detect=args.auto_detect,
            robot_project=args.robot,
        ))
    except (KeyboardInterrupt, SystemExit):
        pass


if __name__ == "__main__":
    main()

