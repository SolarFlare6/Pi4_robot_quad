"""
tool_registry.py

Tool Registry (device_server_hardware_mapper.txt, sections 26-27;
dcp_protocol_specification.txt, sections 14-17, 42;
MachineMake Server Implementation Specification DCP v1.0, sections 3.1-3.6).

A ToolDefinition bundles everything the Device Agent's security pipeline
needs to check a call, plus the callable that actually performs it. The
DCP-visible shape (name/description/arguments/permission) matches the
protocol spec's tool definition exactly so get_tools() can serialize it
directly.
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

Handler = Callable[..., Any]  # async callable; exact signature is tool-specific


@dataclass
class ToolDefinition:
    name: str
    description: str
    arguments: Dict[str, Dict[str, str]]  # {"direction": {"type": "string"}, ...}
    permission: str  # READ_ONLY | CONTROL | ADMIN
    handler: Handler
    is_long_running: bool = False
    safety_constraints: Dict[str, Any] = field(default_factory=dict)

    def to_dcp(self) -> Dict[str, Any]:
        """Serialize as dcp_protocol_specification.txt section 15 shows,
        with toolName/parameters aliases for mobile app client compatibility."""
        return {
            "name": self.name,
            "toolName": self.name,
            "description": self.description,
            "arguments": self.arguments,
            "parameters": self.arguments,
            "params": self.arguments,
            "permission": self.permission,
        }


class ToolRegistry:
    def __init__(self):
        self._tools: Dict[str, ToolDefinition] = {}

    def register(self, tool: ToolDefinition) -> None:
        self._tools[tool.name] = tool

    def get(self, name: str) -> Optional[ToolDefinition]:
        return self._tools.get(name)

    def all_tools(self) -> Dict[str, ToolDefinition]:
        return dict(self._tools)

    def to_dcp_list(self) -> list:
        return [tool.to_dcp() for tool in self._tools.values()]

    def build_default_tools(self, capabilities: list, robot_controller,
                             safety_limits: Dict[str, Any], power_manager=None,
                             desktop_controller=None, hardware_map=None) -> None:
        """Registers the standard tool set conditioned on which
        capabilities the Capability Mapper actually found -- an unknown /
        minimal device simply gets fewer tools rather than the app calling
        into hardware that doesn't exist."""

        cap_ids = set()
        for c in capabilities:
            if isinstance(c, dict):
                cap_ids.add(c.get("id"))
                cap_ids.add(c.get("type"))
            else:
                cap_ids.add(str(c))

        has_robotics = "robotics" in cap_ids or "stand" in cap_ids or robot_controller is not None

        # -- Section 3.1: Locomotion & High-Level Actions ----------------
        if has_robotics:
            self.register(ToolDefinition(
                name="stand",
                description="Make the robot stand up.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.stand(),
            ))

            self.register(ToolDefinition(
                name="sit",
                description="Make the robot sit down.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.sit(),
            ))

            max_distance = safety_limits.get("max_walk_distance_m", 5.0)
            self.register(ToolDefinition(
                name="walk",
                description="Move the robot in a specified direction.",
                arguments={
                    "direction": {"type": "string"},
                    "distance": {"type": "number"},
                },
                permission="CONTROL",
                handler=lambda progress_cb=None, **kw: robot_controller.walk(progress_cb=progress_cb, **kw),
                is_long_running=True,
                safety_constraints={
                    "direction": {"enum": ["forward", "backward", "left", "right"]},
                    "distance": {"min": 0.0, "max": max_distance},
                },
            ))

            self.register(ToolDefinition(
                name="walk_forward",
                description="Move the robot forward by a specified distance.",
                arguments={"distance": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda distance=1.0, progress_cb=None, **kw: robot_controller.walk_forward(distance=distance, progress_cb=progress_cb),
                is_long_running=True,
            ))

            self.register(ToolDefinition(
                name="walk_backward",
                description="Move the robot backward by a specified distance.",
                arguments={"distance": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda distance=1.0, progress_cb=None, **kw: robot_controller.walk_backward(distance=distance, progress_cb=progress_cb),
                is_long_running=True,
            ))

            self.register(ToolDefinition(
                name="walk_left",
                description="Step the robot laterally to the left.",
                arguments={"distance": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda distance=1.0, progress_cb=None, **kw: robot_controller.walk_left(distance=distance, progress_cb=progress_cb),
                is_long_running=True,
            ))

            self.register(ToolDefinition(
                name="walk_right",
                description="Step the robot laterally to the right.",
                arguments={"distance": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda distance=1.0, progress_cb=None, **kw: robot_controller.walk_right(distance=distance, progress_cb=progress_cb),
                is_long_running=True,
            ))

            max_angle = safety_limits.get("max_turn_angle_deg", 180.0)
            self.register(ToolDefinition(
                name="turn",
                description="Turn the robot in place.",
                arguments={
                    "direction": {"type": "string"},
                    "angle": {"type": "number"},
                },
                permission="CONTROL",
                handler=lambda progress_cb=None, **kw: robot_controller.turn(progress_cb=progress_cb, **kw),
                is_long_running=True,
                safety_constraints={
                    "direction": {"enum": ["left", "right"]},
                    "angle": {"min": 0.0, "max": max_angle},
                },
            ))

            self.register(ToolDefinition(
                name="turn_left",
                description="Rotate the robot counter-clockwise (left).",
                arguments={"angle": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda angle=45.0, progress_cb=None, **kw: robot_controller.turn_left(angle=angle, progress_cb=progress_cb),
                is_long_running=True,
            ))

            self.register(ToolDefinition(
                name="turn_right",
                description="Rotate the robot clockwise (right).",
                arguments={"angle": {"type": "number", "required": False}},
                permission="CONTROL",
                handler=lambda angle=45.0, progress_cb=None, **kw: robot_controller.turn_right(angle=angle, progress_cb=progress_cb),
                is_long_running=True,
            ))

            self.register(ToolDefinition(
                name="emergency_stop",
                description="Immediately halt all motors, locomotion loops, and release servo power.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.emergency_stop(),
            ))

        # -- Section 3.1: Servo Control & Realtime Mirroring (PCA9685) ------
        if "robotics" in cap_ids or "pwm" in cap_ids or robot_controller is not None:
            self.register(ToolDefinition(
                name="get_servo_angles",
                description="Returns current angles across quadruped servo channels for realtime 3D simulation mirroring.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_servo_angles(),
            ))

            self.register(ToolDefinition(
                name="driver_set_servo_angle_with_index",
                description="Sets angle of single servo channel (0-15, 0-180 deg).",
                arguments={
                    "index": {"type": "int", "required": True},
                    "angle": {"type": "number", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, angle=90, **kw: robot_controller.driver_set_servo_angle_with_index(index=index, angle=angle),
            ))

            self.register(ToolDefinition(
                name="set_servo_angle",
                description="Sets angle of single servo channel (0-15, 0-180 deg).",
                arguments={
                    "channel": {"type": "int", "required": False},
                    "index": {"type": "int", "required": False},
                    "angle": {"type": "number", "required": True},
                },
                permission="CONTROL",
                handler=lambda channel=0, angle=90, index=None, **kw: (
                    robot_controller.set_servo_angle(channel=index if index is not None else channel, angle=angle)
                ),
            ))

            self.register(ToolDefinition(
                name="set_servo_pulse",
                description="Sets pulse width of single servo channel in microseconds (500-2500 us).",
                arguments={
                    "channel": {"type": "int", "required": False},
                    "index": {"type": "int", "required": False},
                    "pulse_us": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda channel=0, pulse_us=1500, index=None, **kw: (
                    robot_controller.set_servo_pulse(channel=index if index is not None else channel, pulse_us=pulse_us)
                ),
            ))

            self.register(ToolDefinition(
                name="snap_servo_left",
                description="Sets channel angle to 180 deg immediately.",
                arguments={
                    "index": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, **kw: robot_controller.snap_servo_left(index=index),
            ))

            self.register(ToolDefinition(
                name="snap_servo_right",
                description="Sets channel angle to 0 deg immediately.",
                arguments={
                    "index": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, **kw: robot_controller.snap_servo_right(index=index),
            ))

            self.register(ToolDefinition(
                name="pan_to_left",
                description="Sweeps channel smoothly towards 180 deg.",
                arguments={
                    "index": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, **kw: robot_controller.pan_to_left(index=index),
            ))

            self.register(ToolDefinition(
                name="pan_to_right",
                description="Sweeps channel smoothly towards 0 deg.",
                arguments={
                    "index": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, **kw: robot_controller.pan_to_right(index=index),
            ))

            self.register(ToolDefinition(
                name="cleanup_servos",
                description="De-energizes all 16 servo channels to release torque and prevent overheating.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.cleanup_servos(),
            ))

        # -- Section 3.3: WS281x 8-LED Strip Controls (GPIO 10) ----------
        if "lighting" in cap_ids or "led" in cap_ids or robot_controller is not None:
            self.register(ToolDefinition(
                name="turn_on_strip_with_color",
                description="Fills all 8 LEDs with RGB color.",
                arguments={
                    "r": {"type": "int", "required": True},
                    "g": {"type": "int", "required": True},
                    "b": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda r=255, g=255, b=255, **kw: robot_controller.turn_on_strip_with_color(r=r, g=g, b=b),
            ))

            self.register(ToolDefinition(
                name="turn_off_strip",
                description="Turns off all 8 LEDs.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.turn_off_strip(),
            ))

            self.register(ToolDefinition(
                name="turn_on_led_at_index",
                description="Sets single LED color on 8-LED strip.",
                arguments={
                    "index": {"type": "int", "required": True},
                    "r": {"type": "int", "required": True},
                    "g": {"type": "int", "required": True},
                    "b": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, r=255, g=255, b=255, **kw: robot_controller.turn_on_led_at_index(index=index, r=r, g=g, b=b),
            ))

            self.register(ToolDefinition(
                name="turn_off_led_at_index",
                description="Turns off single LED on 8-LED strip.",
                arguments={
                    "index": {"type": "int", "required": True},
                },
                permission="CONTROL",
                handler=lambda index=0, **kw: robot_controller.turn_off_led_at_index(index=index),
            ))

            self.register(ToolDefinition(
                name="animate_running_process",
                description="Runs animated LED chase / cycle sequence (optionally with custom r, g, b color).",
                arguments={
                    "r": {"type": "int", "required": False},
                    "g": {"type": "int", "required": False},
                    "b": {"type": "int", "required": False},
                },
                permission="CONTROL",
                handler=lambda r=0, g=105, b=105, **kw: robot_controller.animate_running_process(r=r, g=g, b=b),
            ))

            self.register(ToolDefinition(
                name="animate_with_color",
                description="Runs animated LED chase / cycle sequence with custom RGB color.",
                arguments={
                    "r": {"type": "int", "required": False},
                    "g": {"type": "int", "required": False},
                    "b": {"type": "int", "required": False},
                },
                permission="CONTROL",
                handler=lambda r=0, g=105, b=105, **kw: robot_controller.animate_with_color(r=r, g=g, b=b),
            ))

            self.register(ToolDefinition(
                name="flash_alert",
                description="Rapid red blinking alert sequence.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.flash_alert(),
            ))

            self.register(ToolDefinition(
                name="blink_oke",
                description="Double green flash confirming command success.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.blink_oke(),
            ))

            self.register(ToolDefinition(
                name="blink_warning",
                description="Amber pulsing warning sequence.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.blink_warning(),
            ))

        # -- Section 3.1: Audio Subsystem (Buzzer & Speaker) ------------
        if "buzzer" in cap_ids or "audio" in cap_ids or robot_controller is not None:
            self.register(ToolDefinition(
                name="play_tone",
                description="Plays frequency/note on piezo buzzer (e.g. 'C4', 'A4').",
                arguments={
                    "tone": {"type": "string", "required": False},
                },
                permission="CONTROL",
                handler=lambda tone="C4", **kw: robot_controller.play_tone(tone=tone),
            ))

            self.register(ToolDefinition(
                name="play_list_of_notes",
                description="Plays melody sequence of notes on tonal buzzer.",
                arguments={
                    "notes": {"type": "list", "required": False},
                },
                permission="CONTROL",
                handler=lambda notes=None, **kw: robot_controller.play_list_of_notes(notes=notes or ["C4", "E4", "G4", "C5"]),
            ))

            self.register(ToolDefinition(
                name="stop_tone",
                description="Silences the tonal buzzer immediately.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.stop_tone(),
            ))

            self.register(ToolDefinition(
                name="play_freq",
                description="Plays buzzer tone at specified frequency (Hz) for given duration (seconds).",
                arguments={
                    "frequency": {"type": "number", "required": True},
                    "duration": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda frequency=1000, duration=0.5, freq=None, **kw: (
                    robot_controller.play_freq(frequency=freq if freq is not None else frequency, duration=duration)
                ),
            ))

            self.register(ToolDefinition(
                name="play_audio",
                description="Plays audio file on the speaker.",
                arguments={
                    "file_path": {"type": "string", "required": False},
                },
                permission="CONTROL",
                handler=lambda file_path="", **kw: robot_controller.play_audio(file_path=file_path),
            ))

            self.register(ToolDefinition(
                name="stop_audio",
                description="Halts speaker audio playback immediately.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.stop_audio(),
            ))

            self.register(ToolDefinition(
                name="set_volume",
                description="Sets speaker playback volume (0.0 to 1.0).",
                arguments={
                    "volume": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda volume=0.8, **kw: (
                    robot_controller.set_volume(volume=volume)
                    if robot_controller is not None and hasattr(robot_controller, "set_volume")
                    else {"status": "ok", "volume": volume}
                ),
            ))

            self.register(ToolDefinition(
                name="speak",
                description="Synthesize and speak text through the robot voice synthesizer (voice_synth.speak).",
                arguments={
                    "text": {"type": "string", "required": True},
                    "volume": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda text="", volume=0.8, **kw: (
                    robot_controller.speak(text=text, volume=volume)
                    if robot_controller is not None and hasattr(robot_controller, "speak")
                    else {"status": "ok", "text": text, "volume": volume}
                ),
            ))

        # -- Section 3.5: MPU6050 IMU Telemetry (I2C 0x68) ---------------
        if "imu" in cap_ids or robot_controller is not None:
            self.register(ToolDefinition(
                name="get_sensor_accel",
                description="Read 3-axis accelerometer values (x, y, z in g).",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_sensor_accel(),
            ))

            self.register(ToolDefinition(
                name="get_sensor_gyro",
                description="Read 3-axis gyroscope values (x, y, z in deg/s).",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_sensor_gyro(),
            ))

            self.register(ToolDefinition(
                name="get_pitch_roll",
                description="Read IMU pitch and roll angles in degrees.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_pitch_roll(),
            ))

            self.register(ToolDefinition(
                name="get_orientation",
                description="Read the current IMU orientation.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_orientation(),
            ))

        # -- Section 3.6: System & Power Options (Windows / Mac / Linux) -----
        from system_power import SystemPowerManager
        power_mgr = power_manager or getattr(robot_controller, "_power_manager", None) or SystemPowerManager(robot_controller=robot_controller)

        self.register(ToolDefinition(
            name="shutdown",
            description="Executes host system shutdown sequence (Windows, macOS, Linux).",
            arguments={
                "force": {"type": "bool", "required": False},
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda force=False, delay=2, simulate=None, **kw: (
                robot_controller.shutdown(force=force, delay=delay, simulate=simulate)
                if robot_controller is not None
                else power_mgr.shutdown(force=force, delay=delay, simulate=simulate)
            ),
        ))

        self.register(ToolDefinition(
            name="reboot",
            description="Executes host system reboot sequence (Windows, macOS, Linux).",
            arguments={
                "force": {"type": "bool", "required": False},
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda force=False, delay=2, simulate=None, **kw: (
                robot_controller.reboot(force=force, delay=delay, simulate=simulate)
                if robot_controller is not None
                else power_mgr.reboot(force=force, delay=delay, simulate=simulate)
            ),
        ))

        self.register(ToolDefinition(
            name="system_shutdown",
            description="Powers down the host machine (alias for shutdown).",
            arguments={
                "force": {"type": "bool", "required": False},
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda force=False, delay=2, simulate=None, **kw: (
                robot_controller.shutdown(force=force, delay=delay, simulate=simulate)
                if robot_controller is not None
                else power_mgr.shutdown(force=force, delay=delay, simulate=simulate)
            ),
        ))

        self.register(ToolDefinition(
            name="system_reboot",
            description="Reboots the host machine (alias for reboot).",
            arguments={
                "force": {"type": "bool", "required": False},
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda force=False, delay=2, simulate=None, **kw: (
                robot_controller.reboot(force=force, delay=delay, simulate=simulate)
                if robot_controller is not None
                else power_mgr.reboot(force=force, delay=delay, simulate=simulate)
            ),
        ))

        self.register(ToolDefinition(
            name="sleep",
            description="Puts host machine into sleep / suspend mode (Windows, macOS, Linux).",
            arguments={
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda delay=2, simulate=None, **kw: (
                power_mgr.sleep(delay=delay, simulate=simulate)
            ),
        ))

        self.register(ToolDefinition(
            name="system_sleep",
            description="Puts host machine into sleep mode (alias for sleep).",
            arguments={
                "delay": {"type": "int", "required": False},
                "simulate": {"type": "bool", "required": False},
            },
            permission="CONTROL",
            handler=lambda delay=2, simulate=None, **kw: (
                power_mgr.sleep(delay=delay, simulate=simulate)
            ),
        ))

        # -- Section 2: PC / Laptop / Desktop Mapping & Control -------------
        from desktop_controller import DesktopController
        desktop_ctrl = desktop_controller or DesktopController(power_manager=power_mgr)

        self.register(ToolDefinition(
            name="mouse_move",
            description="Relative cursor movement from mobile virtual trackpad.",
            arguments={
                "dx": {"type": "number", "required": True},
                "dy": {"type": "number", "required": True},
            },
            permission="CONTROL",
            handler=lambda dx=0.0, dy=0.0, simulate=None, **kw: desktop_ctrl.mouse_move(dx=dx, dy=dy, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="mouse_click",
            description="Executes mouse click (left, right, middle) or double click.",
            arguments={
                "button": {"type": "string", "required": False},
                "double": {"type": "boolean", "required": False},
            },
            permission="CONTROL",
            handler=lambda button="left", double=False, simulate=None, **kw: desktop_ctrl.mouse_click(button=button, double=double, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="mouse_scroll",
            description="Vertical mouse wheel scroll.",
            arguments={
                "dy": {"type": "number", "required": True},
            },
            permission="CONTROL",
            handler=lambda dy=0.0, simulate=None, **kw: desktop_ctrl.mouse_scroll(dy=dy, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="keyboard_type",
            description="Types raw text into the currently active desktop window.",
            arguments={
                "text": {"type": "string", "required": True},
            },
            permission="CONTROL",
            handler=lambda text="", simulate=None, **kw: desktop_ctrl.keyboard_type(text=text, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="keyboard_press",
            description="Executes hotkeys, navigation keys, or special keystrokes.",
            arguments={
                "key": {"type": "string", "required": True},
            },
            permission="CONTROL",
            handler=lambda key="", simulate=None, **kw: desktop_ctrl.keyboard_press(key=key, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="lock_device",
            description="Locks the operating system workstation screen.",
            arguments={
                "os": {"type": "string", "required": False},
                "os_type": {"type": "string", "required": False},
            },
            permission="CONTROL",
            handler=lambda os=None, os_type=None, simulate=None, **kw: desktop_ctrl.lock_device(os_type=os_type, os=os, simulate=simulate),
        ))

        self.register(ToolDefinition(
            name="media_control",
            description="Controls OS media playback and volume levels.",
            arguments={
                "action": {"type": "string", "required": True},
            },
            permission="CONTROL",
            handler=lambda action="", simulate=None, **kw: desktop_ctrl.media_control(action=action, simulate=simulate),
        ))


        # -- Kinematics, Gait, Camera, Config & Telemetry ----------------
        if robot_controller is not None:
            self.register(ToolDefinition(
                name="set_gait",
                description="Set locomotion gait mode (walk, trot, bound, gallop).",
                arguments={"mode": {"type": "string"}},
                permission="CONTROL",
                handler=lambda mode="walk", **kw: robot_controller.set_gait(mode=mode),
            ))

            self.register(ToolDefinition(
                name="set_pose",
                description="Set body kinematic orientation (pitch, roll, yaw) and height.",
                arguments={
                    "pitch": {"type": "number", "required": False},
                    "roll": {"type": "number", "required": False},
                    "yaw": {"type": "number", "required": False},
                    "height": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda **kw: robot_controller.set_pose(**kw),
            ))

            self.register(ToolDefinition(
                name="start_camera",
                description="Start camera video streaming.",
                arguments={
                    "resolution": {"type": "string", "required": False},
                    "fps": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda **kw: robot_controller.start_camera(**kw),
            ))

            self.register(ToolDefinition(
                name="stop_camera",
                description="Stop camera video streaming.",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.stop_camera(),
            ))

            self.register(ToolDefinition(
                name="camera_stream_start",
                description="Start camera video streaming endpoint (alias for start_camera).",
                arguments={
                    "resolution": {"type": "string", "required": False},
                    "fps": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda resolution="640x480", fps=30, **kw: robot_controller.camera_stream_start(resolution=resolution, fps=fps),
            ))

            self.register(ToolDefinition(
                name="camera_stream_stop",
                description="Stop camera video streaming endpoint (alias for stop_camera).",
                arguments={},
                permission="CONTROL",
                handler=lambda **kw: robot_controller.camera_stream_stop(),
            ))

            self.register(ToolDefinition(
                name="set_stabilizer",
                description="Toggle active IMU balance stabilization on the robot.",
                arguments={
                    "enabled": {"type": "boolean", "required": True},
                },
                permission="CONTROL",
                handler=lambda enabled=True, **kw: robot_controller.set_stabilizer(enabled=enabled),
            ))

            self.register(ToolDefinition(
                name="set_autonomy",
                description="Toggle autonomous exploration and obstacle avoidance mode.",
                arguments={
                    "enabled": {"type": "boolean", "required": True},
                },
                permission="CONTROL",
                handler=lambda enabled=True, **kw: robot_controller.set_autonomy(enabled=enabled),
            ))

            self.register(ToolDefinition(
                name="camera_snapshot",
                description="Capture a still snapshot from the camera (alias for take_picture).",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.camera_snapshot(),
            ))

            self.register(ToolDefinition(
                name="take_picture",
                description="Capture a still image from the camera.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.take_picture(),
            ))

            self.register(ToolDefinition(
                name="get_battery",
                description="Get device battery level and charging status.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_battery(),
            ))

            self.register(ToolDefinition(
                name="set_config",
                description="Set dynamic robot runtime parameter.",
                arguments={
                    "param": {"type": "string", "required": False},
                    "value": {"type": "any", "required": False},
                },
                permission="CONTROL",
                handler=lambda param="", value=None, **kw: robot_controller.set_config(param=param, value=value, **kw),
            ))

            self.register(ToolDefinition(
                name="set_led",
                description="Turn the status LED on or off.",
                arguments={"on": {"type": "boolean"}},
                permission="CONTROL",
                handler=lambda on=True, **kw: robot_controller.set_led(on=on),
            ))

            self.register(ToolDefinition(
                name="needle_prompt",
                description="Processes high-level natural language instructions using on-device Needle AI.",
                arguments={
                    "prompt": {"type": "string", "required": True}
                },
                permission="CONTROL",
                handler=lambda prompt="", **kw: robot_controller.needle_prompt(prompt=prompt),
                is_long_running=False,
            ))

            self.register(ToolDefinition(
                name="track_object",
                description=(
                    "Selects and tracks an object with the robot's Pan/Tilt camera. "
                    "Accepts bounding box (x,y,w,h), tap point (x,y or normalized u,v 0.0-1.0), "
                    "or object class label (e.g. 'person', 'sports ball', 'cup', 'center')."
                ),
                arguments={
                    "x": {"type": "number", "required": False},
                    "y": {"type": "number", "required": False},
                    "w": {"type": "number", "required": False},
                    "h": {"type": "number", "required": False},
                    "u": {"type": "number", "required": False},
                    "v": {"type": "number", "required": False},
                    "label": {"type": "string", "required": False},
                    "target": {"type": "string", "required": False},
                    "box_size": {"type": "number", "required": False},
                },
                permission="CONTROL",
                handler=lambda **kw: robot_controller.track_object(**kw),
                is_long_running=False,
            ))

            self.register(ToolDefinition(
                name="select_object",
                description="Alias for track_object to select and track a target on the robot camera feed.",
                arguments={
                    "x": {"type": "number", "required": False},
                    "y": {"type": "number", "required": False},
                    "w": {"type": "number", "required": False},
                    "h": {"type": "number", "required": False},
                    "u": {"type": "number", "required": False},
                    "v": {"type": "number", "required": False},
                    "label": {"type": "string", "required": False},
                    "target": {"type": "string", "required": False},
                },
                permission="CONTROL",
                handler=lambda **kw: robot_controller.track_object(**kw),
                is_long_running=False,
            ))

            self.register(ToolDefinition(
                name="stop_tracking",
                description="Cancels active camera object tracking and optionally recenters the camera pan/tilt.",
                arguments={
                    "recenter": {"type": "boolean", "required": False},
                },
                permission="CONTROL",
                handler=lambda recenter=False, **kw: robot_controller.stop_tracking(recenter=recenter),
                is_long_running=False,
            ))

            self.register(ToolDefinition(
                name="get_tracked_objects",
                description="Returns current camera object tracking state and latest YOLOv8 detections.",
                arguments={},
                permission="READ_ONLY",
                handler=lambda **kw: robot_controller.get_tracked_objects(),
                is_long_running=False,
            ))

        # -- Camera Snapshot & Still Capture -----------------------------
        def _handle_camera_snapshot():
            if robot_controller is not None and hasattr(robot_controller, "camera_snapshot"):
                return robot_controller.camera_snapshot()
            if robot_controller is not None and hasattr(robot_controller, "take_picture"):
                return robot_controller.take_picture()
            return {"status": "ok", "simulated": True, "message": "Camera snapshot captured"}

        self.register(ToolDefinition(
            name="camera_snapshot",
            description="Capture a still snapshot from the camera (alias for take_picture).",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_camera_snapshot(),
        ))

        self.register(ToolDefinition(
            name="take_picture",
            description="Capture a still image from the camera.",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_camera_snapshot(),
        ))

        # -- Battery Query Tool (Host PC Laptop / SBC Robot) ------------
        def _handle_get_battery():
            if robot_controller is not None and hasattr(robot_controller, "get_battery"):
                return robot_controller.get_battery()
            try:
                import psutil
                b = psutil.sensors_battery()
                if b is not None:
                    return {
                        "available": True,
                        "percent": round(b.percent, 1),
                        "power_plugged": bool(b.power_plugged),
                        "charging": bool(b.power_plugged),
                        "secsleft": b.secsleft if b.secsleft != -2 else None,
                    }
            except Exception:
                pass
            return {"available": False, "percent": None, "power_plugged": True}

        self.register(ToolDefinition(
            name="get_battery",
            description="Get device battery level and charging status.",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_get_battery(),
        ))

        # -- Host Hardware Specs & Storage Telemetry ---------------------
        def _handle_get_hardware_info():
            import platform, os, socket
            from main import get_system_temperature, get_gpu_utilization
            temp_val = get_system_temperature(default=45.0)
            gpu_val = get_gpu_utilization()
            res = {
                "os": platform.system().lower(),
                "release": platform.release(),
                "architecture": platform.machine(),
                "hostname": socket.gethostname(),
                "temp": temp_val,
                "cpu_temp": temp_val,
                "gpu": gpu_val,
                "cpu": {
                    "cores": os.cpu_count(),
                    "model": platform.processor(),
                    "temp": temp_val,
                },
            }
            try:
                import psutil
                res["cpu"]["percent"] = round(float(psutil.cpu_percent(interval=None)), 1)
                mem = psutil.virtual_memory()
                res["memory"] = {
                    "total_mb": int(mem.total // (1024 * 1024)),
                    "available_mb": int(mem.available // (1024 * 1024)),
                    "used_mb": int(mem.used // (1024 * 1024)),
                    "percent": round(mem.percent, 1),
                }
                b = psutil.sensors_battery()
                if b is not None:
                    res["battery"] = {
                        "available": True,
                        "percent": round(b.percent, 1),
                        "power_plugged": bool(b.power_plugged),
                    }
            except Exception:
                pass
            return res

        self.register(ToolDefinition(
            name="get_hardware_info",
            description="Get host system hardware details (CPU, RAM, Temp, GPU, OS, Battery).",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_get_hardware_info(),
        ))

        def _handle_get_telemetry():
            from main import collect_telemetry_snapshot
            return collect_telemetry_snapshot(robot_controller=robot_controller)

        self.register(ToolDefinition(
            name="get_telemetry",
            description="Get current system and device telemetry (CPU, RAM, Temp, GPU, Battery, IMU, Servos).",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_get_telemetry(),
        ))

        self.register(ToolDefinition(
            name="system_status",
            description="Get current system telemetry and status (alias for get_telemetry).",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_get_telemetry(),
        ))

        def _handle_get_storage():
            storage = []
            try:
                import psutil
                for part in psutil.disk_partitions(all=False):
                    try:
                        usage = psutil.disk_usage(part.mountpoint)
                        storage.append({
                            "mount": part.mountpoint,
                            "device": part.device,
                            "fstype": part.fstype,
                            "total_mb": usage.total // (1024 * 1024),
                            "free_mb": usage.free // (1024 * 1024),
                            "used_mb": usage.used // (1024 * 1024),
                            "percent": round(usage.percent, 1),
                        })
                    except (PermissionError, OSError):
                        continue
            except Exception:
                pass
            if not storage:
                try:
                    import shutil, os
                    root = os.path.abspath(os.sep)
                    usage = shutil.disk_usage(root)
                    total_mb = usage.total // (1024 * 1024)
                    free_mb = usage.free // (1024 * 1024)
                    used_mb = usage.used // (1024 * 1024)
                    storage.append({
                        "mount": root,
                        "device": root,
                        "total_mb": total_mb,
                        "free_mb": free_mb,
                        "used_mb": used_mb,
                        "percent": round((used_mb / total_mb) * 100, 1) if total_mb else 0.0,
                    })
                except Exception:
                    pass
            return {"storage": storage}

        self.register(ToolDefinition(
            name="get_storage",
            description="Get storage drives and filesystem usage.",
            arguments={},
            permission="READ_ONLY",
            handler=lambda **kw: _handle_get_storage(),
        ))

        # -- Low-Level GPIO & PWM Control Tools --------------------------
        if robot_controller is not None or "gpio" in cap_ids:
            self.register(ToolDefinition(
                name="gpio_write",
                description="Set digital state (HIGH/LOW) of a GPIO pin.",
                arguments={
                    "pin": {"type": "int", "required": True},
                    "state": {"type": "bool", "required": True},
                },
                permission="CONTROL",
                handler=lambda pin=0, state=False, **kw: (
                    robot_controller.gpio_write(pin=pin, state=state)
                    if robot_controller is not None
                    else {"pin": pin, "state": state}
                ),
            ))

            self.register(ToolDefinition(
                name="gpio_read",
                description="Read digital state of a GPIO pin.",
                arguments={
                    "pin": {"type": "int", "required": True},
                },
                permission="READ_ONLY",
                handler=lambda pin=0, **kw: (
                    robot_controller.gpio_read(pin=pin)
                    if robot_controller is not None
                    else {"pin": pin, "state": False}
                ),
            ))

        if robot_controller is not None or "pwm" in cap_ids:
            self.register(ToolDefinition(
                name="pwm_set",
                description="Configure PWM channel duty cycle (0.0 to 1.0).",
                arguments={
                    "channel": {"type": "int", "required": True},
                    "value": {"type": "float", "required": True},
                },
                permission="CONTROL",
                handler=lambda channel=0, value=0.0, **kw: (
                    robot_controller.pwm_set(channel=channel, value=value)
                    if robot_controller is not None
                    else {"channel": channel, "value": value}
                ),
            ))
