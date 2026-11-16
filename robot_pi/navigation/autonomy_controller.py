# robot_pi/navigation/autonomy_controller.py
"""Autonomous Mode State Machine & Navigation Manager.

Controls whether the robot is passively listening to manual mobile app controls
or actively scanning the environment, updating maps, and executing obstacle avoidance.
"""

import asyncio
import logging
import time
from typing import Any, Callable, Dict, Optional

logger = logging.getLogger("robot.autonomy")


class AutonomyController:
    def __init__(
        self,
        scanner,
        local_mapper,
        network_client,
        locomotion_placeholders,
        voice_synth=None,
    ):
        self.scanner = scanner
        self.local_mapper = local_mapper
        self.net_client = network_client
        self.loco = locomotion_placeholders
        self.voice = voice_synth

        self.is_autonomous: bool = False
        self._loop_task: Optional[asyncio.Task] = None

        self.latest_server_command: Optional[Dict[str, Any]] = None
        self.latest_server_command_time: float = 0.0

    def set_autonomy(self, enabled: bool):
        """Enables or disables autonomous exploration and mapping mode."""
        if self.is_autonomous == enabled:
            return

        self.is_autonomous = enabled
        logger.info("Autonomous mode changed -> %s", "ENABLED" if enabled else "DISABLED")

        if self.is_autonomous:
            # Announce via voice if available
            if self.voice and hasattr(self.voice, "speak"):
                self.voice.speak("Autonomous mode activated. Scanning environment.", volume=1.7)

            # Start camera look-around scanning routine
            self.scanner.start()
            self._loop_task = asyncio.create_task(self._autonomy_navigation_loop(), name="AutonomyNavLoop")
        else:
            if self.voice and hasattr(self.voice, "speak"):
                self.voice.speak("Autonomous mode paused. Awaiting manual control.", volume=1.7)

            # Stop scanning and halt motors
            self.scanner.stop()
            if self._loop_task and not self._loop_task.done():
                self._loop_task.cancel()
            self.loco.stop()

    def receive_server_command(self, cmd_data: Dict[str, Any]):
        """Receives high-level directives from the Performance Server."""
        self.latest_server_command = cmd_data
        self.latest_server_command_time = time.time()

    async def _autonomy_navigation_loop(self):
        """Continuous navigation decision loop operating while in autonomous mode."""
        logger.info("Autonomy navigation loop entered.")
        try:
            while self.is_autonomous:
                # 1. Check if Laptop Performance Server is active and providing directives
                now = time.time()
                server_available = (
                    self.latest_server_command is not None
                    and (now - self.latest_server_command_time) < 1.5
                )

                if server_available and self.latest_server_command:
                    # Execute high-level directive computed on Laptop RTX GPU
                    directive = self.latest_server_command.get("command", "stop")
                    speed = float(self.latest_server_command.get("speed", 1.0))
                    duration = float(self.latest_server_command.get("duration", 0.5))

                    self._dispatch_directive(directive, speed=speed)
                    await asyncio.sleep(duration)

                else:
                    # 2. Server offline / disconnected -> Fall back to onboard local mapper
                    # Get guidance from local obstacle mapper running on Pi CPU
                    # (local_mapper directive updated by main loop)
                    directive = getattr(self.local_mapper, "current_directive", "walk_forward")
                    self._dispatch_directive(directive, speed=0.8)
                    await asyncio.sleep(0.5)

        except asyncio.CancelledError:
            pass
        except Exception as e:
            logger.error("Error in autonomy navigation loop: %s", e, exc_info=True)
        finally:
            self.loco.stop()

    def _dispatch_directive(self, directive: str, speed: float = 1.0):
        """Calls the corresponding locomotion placeholder."""
        d = directive.lower().strip()
        if d == "walk_forward":
            self.loco.walk_forward(distance=0.5, speed=speed)
        elif d == "walk_backward":
            self.loco.walk_backward(distance=0.5, speed=speed)
        elif d == "turn_left":
            self.loco.turn_left(angle=30.0)
        elif d == "turn_right":
            self.loco.turn_right(angle=30.0)
        elif d == "walk_left":
            self.loco.walk_left(distance=0.5, speed=speed)
        elif d == "walk_right":
            self.loco.walk_right(distance=0.5, speed=speed)
        elif d == "stop":
            self.loco.stop()
        else:
            self.loco.stop()
