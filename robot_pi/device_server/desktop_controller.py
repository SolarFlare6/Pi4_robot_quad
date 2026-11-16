"""
desktop_controller.py

Desktop Controller for MachineMake Device Server.
Implements the Mapping Tab and Computer Dashboard operations from
server_catchup_specification.md:
  - Mouse virtual trackpad: mouse_move, mouse_click, mouse_scroll
  - Virtual keyboard: keyboard_type, keyboard_press
  - Workstation lock: lock_device (Windows LockWorkStation, Linux loginctl, macOS pmset)
  - Media controls: media_control (play_pause, next, prev, stop, volume_up, volume_down, mute)
  - System operations: system_reboot, system_shutdown (delegates to SystemPowerManager)

Provides simulation / dry-run safety for testing and headless server environments.
"""

import asyncio
import ctypes
import logging
import os
import sys
from typing import Any, Dict, Optional

logger = logging.getLogger("desktop_controller")

# Safe PyAutoGUI import
try:
    import pyautogui
    pyautogui.FAILSAFE = False  # Avoid exceptions when reaching screen edges
    pyautogui.PAUSE = 0.0       # CRITICAL: Disable default 100ms pause for instant response
    _HAS_PYAUTOGUI = True
except Exception:
    _HAS_PYAUTOGUI = False


class DesktopController:
    def __init__(self, power_manager=None, simulate: bool = False):
        self._power = power_manager
        self._simulate = simulate
        self._mouse_accum_x = 0.0
        self._mouse_accum_y = 0.0
        self._is_windows = sys.platform.startswith("win")
        self._user32 = ctypes.windll.user32 if self._is_windows else None

    @property
    def simulate(self) -> bool:
        # Check simulation override from environment variable
        if os.environ.get("DCP_SIMULATE_DESKTOP") == "1" or os.environ.get("DCP_SIMULATE_POWER") == "1":
            return True
        return self._simulate

    @simulate.setter
    def simulate(self, val: bool) -> None:
        self._simulate = bool(val)

    def set_power_manager(self, power_manager) -> None:
        self._power = power_manager

    # -- Section 2.A: Mouse Control Tools --------------------------------

    async def mouse_move(self, dx: float = 0.0, dy: float = 0.0,
                         simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Relative cursor movement from mobile virtual trackpad."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        self._mouse_accum_x += float(dx)
        self._mouse_accum_y += float(dy)

        move_x = int(self._mouse_accum_x)
        move_y = int(self._mouse_accum_y)
        self._mouse_accum_x -= move_x
        self._mouse_accum_y -= move_y

        if not is_sim and (move_x != 0 or move_y != 0):
            if self._is_windows and self._user32:
                # Sub-millisecond direct Win32 hardware mouse move
                self._user32.mouse_event(0x0001, move_x, move_y, 0, 0)
            elif _HAS_PYAUTOGUI:
                try:
                    pyautogui.moveRel(move_x, move_y, duration=0.0)
                except Exception as exc:
                    logger.warning("pyautogui.moveRel failed: %s", exc)

        return {"status": "ok", "dx": float(dx), "dy": float(dy), "applied_dx": move_x, "applied_dy": move_y, "simulated": is_sim}

    async def mouse_click(self, button: str = "left", double: bool = False,
                          simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Executes mouse click (left, right, middle) or double click."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        btn = str(button).lower().strip()
        if btn not in ["left", "right", "middle"]:
            btn = "left"
        is_dbl = bool(double)

        if not is_sim:
            if self._is_windows and self._user32:
                down_flag, up_flag = {
                    "left": (0x0002, 0x0004),     # MOUSEEVENTF_LEFTDOWN, MOUSEEVENTF_LEFTUP
                    "right": (0x0008, 0x0010),    # MOUSEEVENTF_RIGHTDOWN, MOUSEEVENTF_RIGHTUP
                    "middle": (0x0020, 0x0040),   # MOUSEEVENTF_MIDDLEDOWN, MOUSEEVENTF_MIDDLEUP
                }[btn]
                self._user32.mouse_event(down_flag, 0, 0, 0, 0)
                self._user32.mouse_event(up_flag, 0, 0, 0, 0)
                if is_dbl:
                    self._user32.mouse_event(down_flag, 0, 0, 0, 0)
                    self._user32.mouse_event(up_flag, 0, 0, 0, 0)
            elif _HAS_PYAUTOGUI:
                try:
                    if is_dbl:
                        pyautogui.doubleClick(button=btn)
                    else:
                        pyautogui.click(button=btn)
                except Exception as exc:
                    logger.warning("pyautogui.click failed: %s", exc)

        return {"status": "ok", "button": btn, "double": is_dbl, "simulated": is_sim}

    async def mouse_scroll(self, dy: float = 0.0,
                           simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Vertical mouse wheel scroll."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        clicks = int(dy)

        if not is_sim and clicks != 0:
            if self._is_windows and self._user32:
                # MOUSEEVENTF_WHEEL = 0x0800
                self._user32.mouse_event(0x0800, 0, 0, clicks * 120, 0)
            elif _HAS_PYAUTOGUI:
                try:
                    pyautogui.scroll(clicks)
                except Exception as exc:
                    logger.warning("pyautogui.scroll failed: %s", exc)

        return {"status": "ok", "dy": float(dy), "simulated": is_sim}

    # -- Section 2.B: Virtual Keyboard Typing & Shortcuts ----------------

    async def keyboard_type(self, text: str = "",
                            simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Types raw text into the currently active desktop window."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        s_text = str(text or "")

        if not s_text:
            return {"status": "empty", "length": 0, "simulated": is_sim}

        if not is_sim and _HAS_PYAUTOGUI:
            try:
                pyautogui.write(s_text, interval=0.0)
            except Exception as exc:
                logger.warning("pyautogui.write failed: %s", exc)

        return {"status": "ok", "length": len(s_text), "simulated": is_sim}

    async def keyboard_press(self, key: str = "",
                             simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Executes hotkeys, navigation keys, or special keystrokes."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        k = str(key or "").lower().strip()

        if not k:
            return {"status": "empty", "key": "", "simulated": is_sim}

        if not is_sim and _HAS_PYAUTOGUI:
            try:
                if "+" in k:
                    parts = [p.strip() for p in k.split("+")]
                    mapped = []
                    for p in parts:
                        if p in ("win", "super", "cmd"):
                            mapped.append("win")
                        elif p in ("esc", "escape"):
                            mapped.append("esc")
                        elif p in ("return", "enter"):
                            mapped.append("enter")
                        else:
                            mapped.append(p)
                    pyautogui.hotkey(*mapped)
                elif k in ("win", "super", "cmd"):
                    pyautogui.press("win")
                elif k in ("esc", "escape"):
                    pyautogui.press("esc")
                elif k in ("enter", "return"):
                    pyautogui.press("enter")
                elif k in ("backspace", "space", "tab", "delete", "del", "up", "down", "left", "right"):
                    pyautogui.press("delete" if k == "del" else k)
                else:
                    pyautogui.press(k)
            except Exception as exc:
                logger.warning("pyautogui.press failed for key '%s': %s", k, exc)

        return {"status": "ok", "key": key, "simulated": is_sim}

    # -- Section 2.C: Workstation Lock -----------------------------------

    async def lock_device(self, os_type: Optional[str] = None, os: Optional[str] = None,
                          simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Locks the operating system workstation screen."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        raw_os = os or os_type or sys.platform
        target_os = raw_os.lower()

        if sys.platform.startswith("win") or "win" in target_os:
            plat = "windows"
            if not is_sim:
                try:
                    ctypes.windll.user32.LockWorkStation()
                except Exception as exc:
                    logger.warning("LockWorkStation failed: %s", exc)
            return {"status": "locked", "platform": plat, "simulated": is_sim}

        elif sys.platform.startswith("linux") or "linux" in target_os:
            plat = "linux"
            if not is_sim:
                res = os.system("loginctl lock-session || xdg-screensaver lock || gnome-screensaver-command -l")
                return {"status": "locked", "platform": plat, "code": res, "simulated": is_sim}
            return {"status": "locked", "platform": plat, "simulated": is_sim}

        elif sys.platform == "darwin" or "mac" in target_os or "darwin" in target_os:
            plat = "macos"
            if not is_sim:
                os.system("pmset displaysleepnow")
            return {"status": "locked", "platform": plat, "simulated": is_sim}

        return {"status": "error", "message": f"Unsupported OS: {target_os}", "simulated": is_sim}

    # -- Section 2.D: Media Controls -------------------------------------

    async def media_control(self, action: str = "",
                            simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Controls OS media playback and volume levels."""
        is_sim = self.simulate if simulate is None else bool(simulate)
        act = str(action or "").lower().strip()

        # Windows Virtual-Key hardware codes
        win_vk = {
            "play_pause": 0xB3,  # VK_MEDIA_PLAY_PAUSE
            "playpause": 0xB3,
            "play": 0xB3,
            "pause": 0xB3,
            "next": 0xB0,        # VK_MEDIA_NEXT_TRACK
            "nexttrack": 0xB0,
            "prev": 0xB1,        # VK_MEDIA_PREV_TRACK
            "previous": 0xB1,
            "prevtrack": 0xB1,
            "stop": 0xB2,        # VK_MEDIA_STOP
            "volume_up": 0xAF,   # VK_VOLUME_UP
            "volumeup": 0xAF,
            "volume_down": 0xAE, # VK_VOLUME_DOWN
            "volumedown": 0xAE,
            "mute": 0xAD,        # VK_VOLUME_MUTE
            "volumemute": 0xAD,
        }

        if not is_sim:
            if self._is_windows and self._user32 and act in win_vk:
                vk = win_vk[act]
                # KEYEVENTF_EXTENDEDKEY = 0x0001, KEYEVENTF_KEYUP = 0x0002
                self._user32.keybd_event(vk, 0, 0x0001, 0)
                self._user32.keybd_event(vk, 0, 0x0001 | 0x0002, 0)
            elif _HAS_PYAUTOGUI:
                py_key = {
                    "play_pause": "playpause", "playpause": "playpause", "play": "playpause", "pause": "playpause",
                    "next": "nexttrack", "nexttrack": "nexttrack",
                    "prev": "prevtrack", "previous": "prevtrack", "prevtrack": "prevtrack",
                    "stop": "stop",
                    "volume_up": "volumeup", "volumeup": "volumeup",
                    "volume_down": "volumedown", "volumedown": "volumedown",
                    "mute": "volumemute", "volumemute": "volumemute",
                }.get(act)
                if py_key:
                    try:
                        pyautogui.press(py_key)
                    except Exception as exc:
                        logger.warning("pyautogui.press media key '%s' failed: %s", py_key, exc)

        return {"status": "ok", "action": act, "simulated": is_sim}

    # -- Section 2.E: System Operations ----------------------------------

    async def system_reboot(self, force: bool = False, delay: int = 2,
                            simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Reboots the host machine."""
        print("[DesktopController] system_reboot() called")
        if self._power is not None:
            return await self._power.reboot(delay=delay, force=force, simulate=simulate)
        from system_power import SystemPowerManager
        power = SystemPowerManager()
        return await power.reboot(delay=delay, force=force, simulate=simulate)

    async def system_shutdown(self, force: bool = False, delay: int = 2,
                              simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Halts and powers down the host machine."""
        print("[DesktopController] system_shutdown() called")
        if self._power is not None:
            return await self._power.shutdown(delay=delay, force=force, simulate=simulate)
        from system_power import SystemPowerManager
        power = SystemPowerManager()
        return await power.shutdown(delay=delay, force=force, simulate=simulate)
