"""
system_power.py

System Power Manager: handles cross-platform shutdown and reboot sequences
for Windows, macOS (Darwin), and Linux (including Raspberry Pi).

Supports:
- OS detection (Windows, macOS, Linux)
- Hardware cleanup (cutting servo power, turning off LEDs, silencing audio)
- Delayed execution to allow the DCP response to be sent over WebSocket first
- Simulation / dry-run mode for safety during development and unit testing
"""

import asyncio
import logging
import os
import subprocess
import sys
from typing import Any, Dict, List, Optional

logger = logging.getLogger("system_power")


class SystemPowerManager:
    def __init__(self, robot_controller=None, simulate: bool = False):
        self._robot = robot_controller
        self._simulate = simulate

    @property
    def simulate(self) -> bool:
        # Environment variable override for test safety
        if os.environ.get("DCP_SIMULATE_POWER") == "1":
            return True
        return self._simulate

    @simulate.setter
    def simulate(self, val: bool) -> None:
        self._simulate = bool(val)

    def set_robot_controller(self, robot_controller) -> None:
        self._robot = robot_controller

    def get_os_type(self) -> str:
        """Returns normalized OS type: 'windows', 'mac', or 'linux'."""
        plat = sys.platform.lower()
        if plat.startswith("win"):
            return "windows"
        if plat == "darwin":
            return "mac"
        return "linux"

    def get_command(self, action: str, delay_s: int = 2, force: bool = False) -> List[str]:
        """Builds the OS-specific command array for 'shutdown' or 'reboot'."""
        os_type = self.get_os_type()
        delay_str = str(max(0, int(delay_s)))

        if action == "shutdown":
            if os_type == "windows":
                cmd = ["shutdown", "/s"]
                if force:
                    cmd.append("/f")
                cmd.extend(["/t", delay_str, "/c", "MachineMake Remote Shutdown"])
                return cmd
            elif os_type == "mac":
                # Graceful user session shutdown via AppleScript
                return ["osascript", "-e", 'tell app "System Events" to shut down']
            else:
                # Linux / Raspberry Pi
                return ["sudo", "shutdown", "-h", "now"]

        elif action == "reboot":
            if os_type == "windows":
                cmd = ["shutdown", "/r"]
                if force:
                    cmd.append("/f")
                cmd.extend(["/t", delay_str, "/c", "MachineMake Remote Reboot"])
                return cmd
            elif os_type == "mac":
                # Graceful user session restart via AppleScript
                return ["osascript", "-e", 'tell app "System Events" to restart']
            else:
                # Linux / Raspberry Pi
                return ["sudo", "reboot"]

        elif action in ("sleep", "suspend"):
            if os_type == "windows":
                return ["rundll32.exe", "powrprof.dll,SetSuspendState", "0,1,0"]
            elif os_type == "mac":
                return ["pmset", "sleepnow"]
            else:
                # Linux / Raspberry Pi
                return ["systemctl", "suspend"]

        raise ValueError(f"Unknown power action: {action}")

    async def execute(self, action: str, delay_s: int = 2, force: bool = False,
                      simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Executes hardware pre-shutdown safety cleanup followed by the OS command."""
        os_type = self.get_os_type()
        is_sim = self.simulate if simulate is None else bool(simulate)

        cmd = self.get_command(action, delay_s=delay_s, force=force)
        cmd_str = " ".join(cmd)

        logger.info("Initiating %s sequence for %s: '%s' (simulated=%s)", action, os_type, cmd_str, is_sim)
        print(f"[SystemPower] Initiating {action} sequence for OS: {os_type} (Command: '{cmd_str}', simulated={is_sim})")

        # 1. Graceful hardware safety release
        if self._robot is not None:
            try:
                print(f"[SystemPower] Releasing robot hardware resources before {action}...")
                if hasattr(self._robot, "cleanup_servos"):
                    await self._robot.cleanup_servos()
                if hasattr(self._robot, "turn_off_strip"):
                    await self._robot.turn_off_strip()
                if hasattr(self._robot, "stop_audio"):
                    await self._robot.stop_audio()
                if hasattr(self._robot, "stop_tone"):
                    await self._robot.stop_tone()
                if hasattr(self._robot, "stop_camera"):
                    await self._robot.stop_camera()
            except Exception as exc:
                logger.warning("Error releasing hardware resources during %s: %s", action, exc)

        # 2. Schedule OS execution if not in simulation mode
        if not is_sim:
            asyncio.create_task(self._delayed_spawn(cmd, delay_s=min(0.5, float(delay_s))))

        status_val = "sleeping" if action in ("sleep", "suspend") else ("shutting_down" if action == "shutdown" else "rebooting")
        return {
            "status": status_val,
            "action": action,
            "os": os_type,
            "command": cmd_str,
            "simulated": is_sim,
        }

    async def shutdown(self, delay: int = 2, force: bool = False, simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Initiates system shutdown."""
        return await self.execute("shutdown", delay_s=delay, force=force, simulate=simulate)

    async def reboot(self, delay: int = 2, force: bool = False, simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Initiates system reboot."""
        return await self.execute("reboot", delay_s=delay, force=force, simulate=simulate)

    async def sleep(self, delay: int = 2, simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Initiates system sleep / suspend."""
        return await self.execute("sleep", delay_s=delay, simulate=simulate)

    async def _delayed_spawn(self, cmd: List[str], delay_s: float = 0.5) -> None:
        """Waits briefly so the DCP response is sent over WebSocket before the host restarts."""
        await asyncio.sleep(delay_s)
        try:
            logger.info("Executing system power command: %s", cmd)
            subprocess.Popen(cmd, shell=False)
        except Exception as exc:
            logger.error("Failed to execute power command %s: %s", cmd, exc)
            # Try secondary fallbacks
            os_type = self.get_os_type()
            if os_type == "mac":
                try:
                    fallback = ["sudo", "shutdown", "-h" if "shut down" in " ".join(cmd) else "-r", "now"]
                    subprocess.Popen(fallback, shell=False)
                except Exception:
                    pass
            elif os_type == "linux":
                try:
                    fallback = ["systemctl", "poweroff" if "shutdown" in " ".join(cmd) else "reboot"]
                    subprocess.Popen(fallback, shell=False)
                except Exception:
                    pass
