# robot_pi/vision/scan_routine.py
"""Active Look-Around and Environment Scanning Routine.

Smoothly sweeps the camera pan and tilt servos across the environment when
the robot is in autonomous mode, allowing the world mapping engine to acquire
panoramic depth and obstacle information.
"""

import asyncio
import logging
import time
from typing import Optional

logger = logging.getLogger("robot.scan")


class LookAroundScanner:
    def __init__(self, pan_tilt_controller):
        self.pt = pan_tilt_controller
        self.is_scanning = False
        self._task: Optional[asyncio.Task] = None

        # Sweep limits
        self.pan_start = 45.0
        self.pan_end = 135.0
        self.tilt_nominal = 58.0  # slightly looking forward/downward toward floor (52 is horizontal center, -> 180 is down)
        self.step_deg = 8.0
        self.step_delay = 0.12

    def start(self):
        """Starts the active scanning routine loop."""
        if self.is_scanning:
            return
        self.is_scanning = True
        self._task = asyncio.create_task(self._sweep_loop(), name="LookAroundScanTask")
        logger.info("Active Look-Around scanner started.")

    def stop(self):
        """Stops the active scanning routine and re-centers camera."""
        if not self.is_scanning:
            return
        self.is_scanning = False
        if self._task and not self._task.done():
            self._task.cancel()
        self.pt.center()
        logger.info("Active Look-Around scanner stopped.")

    async def _sweep_loop(self):
        """Continuously sweeps pan angle back and forth while maintaining tilt."""
        try:
            curr_pan = self.pan_start
            direction = 1  # 1 = left-to-right, -1 = right-to-left

            self.pt.set_tilt(self.tilt_nominal)

            while self.is_scanning:
                self.pt.set_pan(curr_pan)
                await asyncio.sleep(self.step_delay)

                curr_pan += direction * self.step_deg

                if curr_pan >= self.pan_end:
                    curr_pan = self.pan_end
                    direction = -1
                    await asyncio.sleep(0.2)  # pause briefly at edge
                elif curr_pan <= self.pan_start:
                    curr_pan = self.pan_start
                    direction = 1
                    await asyncio.sleep(0.2)  # pause briefly at edge

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error("Error in LookAroundScanner sweep loop: %s", e)
        finally:
            self.is_scanning = False
