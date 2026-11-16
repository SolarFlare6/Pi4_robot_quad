"""
detectors/system.py

System Detector (device_server_hardware_mapper.txt, section 11).
Collects host system information: platform, architecture, OS, CPU, RAM,
storage, hostname. Read-only, best-effort, never raises -- callers get
{"available": False, "error": ...} on failure instead (section 19).
"""

import os
import platform
import socket
from typing import Any, Dict


def _detect_platform() -> str:
    """Best-effort guess of the board/platform we're running on. This is
    intentionally heuristic: precise board identification (e.g. exactly
    which Raspberry Pi model) is a nice-to-have, not something the rest of
    the framework depends on."""
    machine = platform.machine().lower()
    try:
        with open("/proc/device-tree/model", "r") as f:
            model = f.read().strip("\x00").strip()
            if "raspberry pi" in model.lower():
                return "raspberry_pi"
    except (FileNotFoundError, PermissionError, OSError):
        pass

    if machine.startswith("esp32"):
        return "esp32"
    if os.environ.get("DCP_PLATFORM_OVERRIDE"):
        return os.environ["DCP_PLATFORM_OVERRIDE"]
    if machine in ("x86_64", "amd64", "i386", "i686"):
        return "pc"
    if machine.startswith("arm") or machine.startswith("aarch64"):
        return "arm_device"
    return "unknown"


def _total_memory_mb() -> int:
    """Best-effort RAM detection across Windows, macOS, and Linux."""
    try:
        import psutil
        return int(psutil.virtual_memory().total // (1024 * 1024))
    except Exception:
        pass
    try:
        with open("/proc/meminfo", "r") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    kb = int(line.split()[1])
                    return kb // 1024
    except (FileNotFoundError, PermissionError, ValueError, OSError):
        pass
    return None


def _storage_info() -> list:
    """Best-effort disk usage inventory across Windows, macOS, and Linux."""
    storage = []
    try:
        import psutil
        for part in psutil.disk_partitions(all=False):
            try:
                usage = psutil.disk_usage(part.mountpoint)
                total_mb = usage.total // (1024 * 1024)
                free_mb = usage.free // (1024 * 1024)
                used_mb = usage.used // (1024 * 1024)
                percent = round(usage.percent, 1)
                storage.append({
                    "mount": part.mountpoint,
                    "device": part.device,
                    "fstype": part.fstype,
                    "total_mb": total_mb,
                    "free_mb": free_mb,
                    "used_mb": used_mb,
                    "percent": percent,
                })
            except (PermissionError, OSError):
                continue
        if storage:
            return storage
    except Exception:
        pass

    try:
        import shutil
        root = os.path.abspath(os.sep)
        usage = shutil.disk_usage(root)
        total_mb = usage.total // (1024 * 1024)
        free_mb = usage.free // (1024 * 1024)
        used_mb = usage.used // (1024 * 1024)
        percent = round((used_mb / total_mb) * 100, 1) if total_mb else 0.0
        return [{
            "mount": root,
            "device": root,
            "total_mb": total_mb,
            "free_mb": free_mb,
            "used_mb": used_mb,
            "percent": percent,
        }]
    except Exception:
        pass

    try:
        usage = os.statvfs("/")
        total_mb = (usage.f_frsize * usage.f_blocks) // (1024 * 1024)
        free_mb = (usage.f_frsize * usage.f_bavail) // (1024 * 1024)
        return [{"mount": "/", "total_mb": total_mb, "free_mb": free_mb}]
    except (AttributeError, OSError):
        return []


def _battery_info() -> Dict[str, Any]:
    """Detect battery charge level and plugged status if present."""
    try:
        import psutil
        battery = psutil.sensors_battery()
        if battery is not None:
            return {
                "available": True,
                "percent": round(battery.percent, 1),
                "power_plugged": bool(battery.power_plugged),
                "secsleft": battery.secsleft if battery.secsleft != -2 else None,
            }
    except Exception:
        pass
    return {"available": False}


def _gpu_info() -> Dict[str, Any]:
    """Detect presence and model of GPU if available."""
    # 1. Check pynvml
    try:
        import pynvml
        pynvml.nvmlInit()
        cnt = pynvml.nvmlDeviceGetCount()
        if cnt > 0:
            names = []
            for i in range(cnt):
                h = pynvml.nvmlDeviceGetHandleByIndex(i)
                raw_name = pynvml.nvmlDeviceGetName(h)
                names.append(raw_name if isinstance(raw_name, str) else raw_name.decode("utf-8", "ignore"))
            return {
                "available": True,
                "count": cnt,
                "model": ", ".join(names),
            }
    except Exception:
        pass

    # 2. Check nvidia-smi CLI
    try:
        import shutil, subprocess
        if shutil.which("nvidia-smi"):
            out = subprocess.check_output(
                ["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                timeout=1.0,
                text=True,
                stderr=subprocess.DEVNULL,
            ).strip()
            lines = [l.strip() for l in out.splitlines() if l.strip()]
            if lines:
                return {
                    "available": True,
                    "count": len(lines),
                    "model": ", ".join(lines),
                }
    except Exception:
        pass

    return {"available": False}


def detect() -> Dict[str, Any]:
    try:
        cpu_count = os.cpu_count()
        cpu_model = None
        try:
            with open("/proc/cpuinfo", "r") as f:
                for line in f:
                    if line.lower().startswith("model name"):
                        cpu_model = line.split(":", 1)[1].strip()
                        break
        except (FileNotFoundError, PermissionError, OSError):
            pass

        if not cpu_model:
            cpu_model = platform.processor() or None

        return {
            "available": True,
            "device": {
                "platform": _detect_platform(),
                "architecture": platform.machine(),
                "os": platform.system().lower(),
            },
            "system": {
                "cpu": {
                    "cores": cpu_count,
                    "model": cpu_model,
                },
                "gpu": _gpu_info(),
                "memory_mb": _total_memory_mb(),
                "storage": _storage_info(),
                "battery": _battery_info(),
                "hostname": socket.gethostname(),
            },
        }
    except Exception as exc:  # noqa: BLE001 - detectors must never crash the mapper
        return {"available": False, "error": str(exc)}
