"""
robot_controller.py

Example Robot Controller for a quadruped
(device_server_hardware_mapper.txt, sections 33-34;
dcp_protocol_specification.txt, section 37;
MachineMake Server Implementation Specification DCP v1.0).

This is where DCP tool calls interface with hardware/simulated hardware.
Per user specification: simulated hardware functions with console prints
confirming execution, maintaining full API compatibility with the mobile app.
"""

import asyncio
from typing import Any, Callable, Dict, List, Optional


class RobotController:
    def __init__(self, physical_config: Dict[str, Any], hardware_map: Optional[Dict[str, Any]] = None):
        self._servos = physical_config.get("servos", {})
        self._pca9685_cfg = physical_config.get("pca9685", {})
        self.hardware_map: Dict[str, Any] = hardware_map or {}
        self.standing = False
        self.orientation = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.height = 0.0
        self.led_state = False
        self.gait = "walk"
        self.camera_streaming = False
        self.dynamic_config: Dict[str, Any] = {}
        self.gpio_pins: Dict[int, bool] = {}
        self.pwm_channels: Dict[int, float] = {}

        # Simulated 16-channel PCA9685 servo positions (0-15, angles 0-180 deg or None if de-energized)
        self.servo_angles: Dict[int, Optional[float]] = {i: None for i in range(16)}

        # Active servo angle cache for Realtime 3D simulation mirroring (Section 3.1)
        self.current_servo_angles: Dict[str, float] = {
            "2": 110, "3": 70,  "4": 50,
            "5": 110, "6": 110, "7": 150,
            "8": 130, "9": 70,  "10": 40,
            "11": 80, "12": 90, "13": 160,
        }

        # Simulated WS281x 8-LED strip (GPIO 10)
        self.strip_leds: List[Dict[str, int]] = [{"r": 0, "g": 0, "b": 0} for _ in range(8)]

        # Simulated buzzer (GPIO 23)
        self.current_tone: Optional[str] = None

        # Autonomous mode & IMU stabilizer flags
        self.stabilizer_enabled: bool = True
        self.autonomy_enabled: bool = False

        # Bind controller to robot_conf_fn
        try:
            import robot_conf_fn
            robot_conf_fn.set_robot_controller(self)
        except Exception:
            pass

    def set_hardware_map(self, hardware_map: Dict[str, Any]) -> None:
        """Stores the hardware scan result for Needle AI awareness."""
        self.hardware_map = hardware_map or {}

    # -- simulated low-level hardware I/O --------------------------------

    async def _drive_servos(self, positions: Dict[str, float], duration_s: float) -> None:
        """Stand-in for real servo motion. `positions` maps servo name ->
        target angle; a real implementation would compute inverse
        kinematics and stream PWM updates to the PCA9685 here."""
        await asyncio.sleep(duration_s)

    # -- Section 3.1: Quadruped Locomotion & High-Level Actions ----------

    async def stand(self, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Make the robot stand up using calibrated standing angles."""
        print("[RobotController] stand() called")
        try:
            import robot_conf_fn
            robot_conf_fn.stand()
        except Exception:
            pass
        await self._drive_servos({name: 90 for name in self._servos}, duration_s=0.1)
        self.standing = True
        # Calibrate standing angles across PCA9685 channels
        for i in (3, 4, 6, 7, 9, 10, 12, 13):
            self.servo_angles[i] = 90.0
        return {"standing": True, "status": "standing"}

    async def sit(self, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Make the robot sit down using calibrated resting angles."""
        print("[RobotController] sit() called")
        try:
            import robot_conf_fn
            robot_conf_fn.sit()
        except Exception:
            pass
        await self._drive_servos({name: 0 for name in self._servos}, duration_s=0.1)
        self.standing = False
        for i in (3, 4, 6, 7, 9, 10, 12, 13):
            self.servo_angles[i] = 0.0
        return {"standing": False, "status": "sitting"}

    async def walk(self, direction: str, distance: float,
                   progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Executes forward/backward/lateral step cycles."""
        print(f"[RobotController] walk() called with direction={direction}, distance={distance}")
        try:
            import robot_conf_fn
            robot_conf_fn.walk(direction=direction, distance=distance)
        except Exception:
            pass
        steps = 5
        for step in range(1, steps + 1):
            await self._drive_servos({}, duration_s=0.05)
            if progress_cb:
                progress_cb(step / steps)
        return {"direction": direction, "distance": float(distance), "status": "completed"}

    async def walk_forward(self, distance: float = 1.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for walking forward."""
        return await self.walk(direction="forward", distance=distance, progress_cb=progress_cb)

    async def walk_backward(self, distance: float = 1.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for walking backward."""
        return await self.walk(direction="backward", distance=distance, progress_cb=progress_cb)

    async def walk_left(self, distance: float = 1.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for stepping left."""
        return await self.walk(direction="left", distance=distance, progress_cb=progress_cb)

    async def walk_right(self, distance: float = 1.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for stepping right."""
        return await self.walk(direction="right", distance=distance, progress_cb=progress_cb)

    async def turn(self, direction: str, angle: float,
                   progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Rotates robot body by specified degrees."""
        print(f"[RobotController] turn() called with direction={direction}, angle={angle}")
        try:
            import robot_conf_fn
            robot_conf_fn.turn(direction=direction, angle=angle)
        except Exception:
            pass
        steps = 3
        for step in range(1, steps + 1):
            await self._drive_servos({}, duration_s=0.05)
            if progress_cb:
                progress_cb(step / steps)
        return {"direction": direction, "angle": float(angle), "status": "completed"}

    async def turn_left(self, angle: float = 45.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for turning left."""
        return await self.turn(direction="left", angle=angle, progress_cb=progress_cb)

    async def turn_right(self, angle: float = 45.0, progress_cb: Optional[Callable[[float], None]] = None) -> Dict[str, Any]:
        """Convenience method for turning right."""
        return await self.turn(direction="right", angle=angle, progress_cb=progress_cb)

    async def emergency_stop(self) -> Dict[str, Any]:
        """Immediately halts all motion, disables servo drive, and sits down."""
        print("[RobotController] emergency_stop() called (halting locomotion, cutting servo power)")
        try:
            import robot_conf_fn
            robot_conf_fn.emergency_stop()
        except Exception:
            pass
        self.standing = False
        for i in range(16):
            self.servo_angles[i] = None
        await self._drive_servos({name: 0 for name in self._servos}, duration_s=0.01)
        return {"stopped": True, "standing": False, "message": "Emergency stop triggered"}

    # -- Section 3.1: Direct Servo Control & Realtime Mirroring (PCA9685) -
    async def get_servo_angles(self) -> Dict[str, Any]:
        """Returns servo angles dictionary for 3D model realtime mirroring."""
        print("[RobotController] get_servo_angles() called")
        return {"servos": dict(self.current_servo_angles)}

    def get_servo_angles_dict(self) -> Dict[str, float]:
        """Helper to get current servo angles dictionary."""
        return dict(self.current_servo_angles)

    async def driver_set_servo_angle_with_index(self, index: int, angle: float) -> Dict[str, Any]:
        """Sets angle of single servo channel (0-15, 0-180)."""
        idx = int(index)
        clamped_angle = max(0.0, min(180.0, float(angle)))
        if 0 <= idx < 16:
            self.servo_angles[idx] = clamped_angle
            self.current_servo_angles[str(idx)] = clamped_angle
        print(f"[RobotController] driver_set_servo_angle_with_index() called with index={idx}, angle={clamped_angle}")
        try:
            import robot_conf_fn
            robot_conf_fn.set_servo_angle(idx, clamped_angle)
        except Exception:
            pass
        return {"index": idx, "angle": clamped_angle, "status": "success"}

    async def set_servo_angle(self, channel: int = 0, angle: float = 90.0, **kw) -> Dict[str, Any]:
        """Sets angle of single servo channel (0-15, 0-180). Accepts either 'channel' or 'index'."""
        ch = kw.get("index", channel)
        return await self.driver_set_servo_angle_with_index(ch, angle)

    async def set_servo_pulse(self, channel: int = 0, pulse_us: int = 1500, **kw) -> Dict[str, Any]:
        """Sets servo pulse width in microseconds (500us - 2500us -> 0 - 180 deg)."""
        ch = kw.get("index", channel)
        p_us = max(500, min(2500, int(pulse_us)))
        calc_angle = (p_us - 500) * 180.0 / 2000.0
        print(f"[RobotController] set_servo_pulse() called with channel={ch}, pulse_us={p_us} (angle={calc_angle:.1f})")
        res = await self.driver_set_servo_angle_with_index(ch, calc_angle)
        res["pulse_us"] = p_us
        return res

    async def snap_servo_left(self, index: int = 0) -> Dict[str, Any]:
        """Sets channel angle to 180 deg immediately."""
        idx = int(index)
        if 0 <= idx < 16:
            self.servo_angles[idx] = 180.0
            self.current_servo_angles[str(idx)] = 180.0
        print(f"[RobotController] snap_servo_left() called with index={idx}")
        try:
            import robot_conf_fn
            robot_conf_fn.snap_servo_left(idx)
        except Exception:
            pass
        return {"index": idx, "angle": 180.0, "status": "snapped_left"}

    async def snap_servo_right(self, index: int = 0) -> Dict[str, Any]:
        """Sets channel angle to 0 deg immediately."""
        idx = int(index)
        if 0 <= idx < 16:
            self.servo_angles[idx] = 0.0
            self.current_servo_angles[str(idx)] = 0.0
        print(f"[RobotController] snap_servo_right() called with index={idx}")
        try:
            import robot_conf_fn
            robot_conf_fn.snap_servo_right(idx)
        except Exception:
            pass
        return {"index": idx, "angle": 0.0, "status": "snapped_right"}

    async def pan_to_left(self, index: int = 0) -> Dict[str, Any]:
        """Sweeps channel smoothly towards 180 deg."""
        idx = int(index)
        if 0 <= idx < 16:
            self.servo_angles[idx] = 180.0
            self.current_servo_angles[str(idx)] = 180.0
        print(f"[RobotController] pan_to_left() called with index={idx}")
        try:
            import robot_conf_fn
            robot_conf_fn.pan_to_left(idx)
        except Exception:
            pass
        return {"index": idx, "angle": 180.0, "status": "panned_left"}

    async def pan_to_right(self, index: int = 0) -> Dict[str, Any]:
        """Sweeps channel smoothly towards 0 deg."""
        idx = int(index)
        if 0 <= idx < 16:
            self.servo_angles[idx] = 0.0
            self.current_servo_angles[str(idx)] = 0.0
        print(f"[RobotController] pan_to_right() called with index={idx}")
        try:
            import robot_conf_fn
            robot_conf_fn.pan_to_right(idx)
        except Exception:
            pass
        return {"index": idx, "angle": 0.0, "status": "panned_right"}

    async def cleanup_servos(self) -> Dict[str, Any]:
        """Critical for Halt: Sets servo angle to None across all 16 channels to de-energize coils."""
        self.standing = False
        for i in range(16):
            self.servo_angles[i] = None
        print("[RobotController] cleanup_servos() called: all 16 servos de-energized and coils released")
        try:
            import robot_conf_fn
            robot_conf_fn.cleanup_servos()
        except Exception:
            pass
        return {"status": "servos_released", "released": True}

    # -- Section 3.3: WS281x 8-LED Strip Controls (GPIO 10) ---------------

    async def turn_on_strip_with_color(self, r: int = 255, g: int = 255, b: int = 255) -> Dict[str, Any]:
        """Fills all 8 LEDs with RGB color."""
        r_c, g_c, b_c = max(0, min(255, int(r))), max(0, min(255, int(g))), max(0, min(255, int(b)))
        self.strip_leds = [{"r": r_c, "g": g_c, "b": b_c} for _ in range(8)]
        print(f"[RobotController] turn_on_strip_with_color() called with r={r_c}, g={g_c}, b={b_c}")
        try:
            import robot_conf_fn
            robot_conf_fn.turn_on_strip_with_color(r_c, g_c, b_c)
        except Exception:
            pass
        return {"color": [r_c, g_c, b_c], "status": "success"}

    async def turn_off_strip(self) -> Dict[str, Any]:
        """Turns off all 8 LEDs."""
        self.strip_leds = [{"r": 0, "g": 0, "b": 0} for _ in range(8)]
        print("[RobotController] turn_off_strip() called")
        try:
            import robot_conf_fn
            robot_conf_fn.turn_off_strip()
        except Exception:
            pass
        return {"status": "off"}

    async def turn_on_led_at_index(self, index: int = 0, r: int = 255, g: int = 255, b: int = 255) -> Dict[str, Any]:
        """Sets single LED color at index 0-7."""
        idx = int(index)
        r_c, g_c, b_c = max(0, min(255, int(r))), max(0, min(255, int(g))), max(0, min(255, int(b)))
        if 0 <= idx < len(self.strip_leds):
            self.strip_leds[idx] = {"r": r_c, "g": g_c, "b": b_c}
        print(f"[RobotController] turn_on_led_at_index() called with index={idx}, r={r_c}, g={g_c}, b={b_c}")
        try:
            import robot_conf_fn
            robot_conf_fn.turn_on_led_at_index(idx, r_c, g_c, b_c)
        except Exception:
            pass
        return {"index": idx, "color": [r_c, g_c, b_c], "status": "success"}

    async def turn_off_led_at_index(self, index: int = 0) -> Dict[str, Any]:
        """Turns off single LED at index 0-7."""
        idx = int(index)
        if 0 <= idx < len(self.strip_leds):
            self.strip_leds[idx] = {"r": 0, "g": 0, "b": 0}
        print(f"[RobotController] turn_off_led_at_index() called with index={idx}")
        try:
            import robot_conf_fn
            robot_conf_fn.turn_off_led_at_index(idx)
        except Exception:
            pass
        return {"index": idx, "status": "off"}

    async def animate_running_process(self, r: int = 0, g: int = 105, b: int = 105) -> Dict[str, Any]:
        """Runs animated LED chase / cycle with optional custom RGB color."""
        r_c = max(0, min(255, int(r)))
        g_c = max(0, min(255, int(g)))
        b_c = max(0, min(255, int(b)))
        print(f"[RobotController] animate_running_process() called with r={r_c}, g={g_c}, b={b_c}")
        try:
            import robot_conf_fn
            robot_conf_fn.animate_running_process(r_c, g_c, b_c)
        except Exception:
            pass
        return {"status": "running", "animation": "running_process", "color": [r_c, g_c, b_c]}

    async def animate_with_color(self, r: int = 0, g: int = 105, b: int = 105) -> Dict[str, Any]:
        """Runs animated LED chase / cycle with the specified RGB color."""
        return await self.animate_running_process(r=r, g=g, b=b)

    async def flash_alert(self) -> Dict[str, Any]:
        """Rapid red blinking alert sequence."""
        print("[RobotController] flash_alert() called")
        try:
            import robot_conf_fn
            robot_conf_fn.flash_alert()
        except Exception:
            pass
        return {"status": "alert", "animation": "flash_alert"}

    async def blink_oke(self) -> Dict[str, Any]:
        """Double green flash confirming command success."""
        print("[RobotController] blink_oke() called")
        try:
            import robot_conf_fn
            robot_conf_fn.blink_oke()
        except Exception:
            pass
        return {"status": "ok", "animation": "blink_oke"}

    async def blink_warning(self) -> Dict[str, Any]:
        """Amber pulsing warning sequence."""
        print("[RobotController] blink_warning() called")
        try:
            import robot_conf_fn
            robot_conf_fn.blink_warning()
        except Exception:
            pass
        return {"status": "warning", "animation": "blink_warning"}

    # -- Section 3.4: Audio Subsystem (Buzzer & Speaker) -----------------

    async def play_tone(self, tone: str = "C4") -> Dict[str, Any]:
        """Plays frequency using tone generator."""
        self.current_tone = str(tone)
        print(f"[RobotController] play_tone() called with tone={tone}")
        try:
            import robot_conf_fn
            robot_conf_fn.play_tone(tone)
        except Exception:
            pass
        return {"tone": str(tone), "status": "playing"}

    async def play_list_of_notes(self, notes: Any = None) -> Dict[str, Any]:
        """Plays melody sequence asynchronously."""
        note_list = list(notes) if isinstance(notes, (list, tuple)) else [str(notes or "C4")]
        print(f"[RobotController] play_list_of_notes() called with notes={note_list}")
        try:
            import robot_conf_fn
            robot_conf_fn.play_list_of_notes(note_list)
        except Exception:
            pass
        return {"notes": note_list, "status": "playing_melody"}

    async def stop_tone(self) -> Dict[str, Any]:
        """Silences the tonal buzzer immediately."""
        self.current_tone = None
        print("[RobotController] stop_tone() called")
        try:
            import robot_conf_fn
            robot_conf_fn.stop_tone()
        except Exception:
            pass
        return {"status": "stopped"}

    async def play_freq(self, frequency: int = 1000, duration: float = 0.5) -> Dict[str, Any]:
        """Plays buzzer tone at specified frequency (Hz) for given duration (seconds)."""
        freq = int(frequency)
        dur = float(duration)
        self.current_tone = f"{freq}Hz"
        print(f"[RobotController] play_freq() called with frequency={freq}Hz, duration={dur}s")
        try:
            import robot_conf_fn
            robot_conf_fn.play_freq(freq, dur)
        except Exception:
            try:
                from drivers import mini_speaker_imp
                mini_speaker_imp.play_tone_adv(freq, dur)
            except Exception:
                pass
        self.current_tone = None
        return {"status": "completed", "freq": freq, "frequency": freq, "duration": dur}

    async def play_audio(self, file_path: str = "") -> Dict[str, Any]:
        """Plays audio file via speaker."""
        path = str(file_path or "")
        print(f"[RobotController] play_audio() called with file_path={path}")
        try:
            import robot_conf_fn
            robot_conf_fn.play_audio(path)
        except Exception:
            try:
                from drivers import upgraded_audio_interface_imp
                upgraded_audio_interface_imp.play_audio(path)
            except Exception:
                pass
        return {"status": "playing", "file": path}

    async def stop_audio(self) -> Dict[str, Any]:
        """Halts speaker audio playback immediately."""
        print("[RobotController] stop_audio() called")
        try:
            import robot_conf_fn
            robot_conf_fn.stop_audio()
        except Exception:
            try:
                from drivers import upgraded_audio_interface_imp
                upgraded_audio_interface_imp.stop_audio()
            except Exception:
                pass
        return {"status": "stopped"}

    async def set_volume(self, volume: float = 1.7) -> Dict[str, Any]:
        """Sets speaker and voice synth volume."""
        vol = max(0.0, min(3.0, float(volume)))
        print(f"[RobotController] set_volume() called with volume={vol}")
        try:
            import robot_conf_fn
            robot_conf_fn.set_volume(vol)
        except Exception:
            try:
                from drivers import upgraded_audio_interface_imp
                upgraded_audio_interface_imp.set_volume(vol)
            except Exception:
                pass
        return {"volume": vol, "status": "success"}

    async def speak(self, text: str = "", volume: float = 1.7) -> Dict[str, Any]:
        """Synthesizes speech on the robot using upgraded_audio_interface_imp / voice_synth."""
        vol = max(0.0, min(3.0, float(volume)))
        clean_text = str(text or "").strip()
        print(f"[RobotController] speak() called with text='{clean_text}', volume={vol}")
        try:
            from drivers import upgraded_audio_interface_imp
            upgraded_audio_interface_imp.robot_speak_with_volume(clean_text, vol=vol)
        except Exception:
            try:
                import robot_conf_fn
                robot_conf_fn.speak(clean_text, volume=vol)
            except Exception:
                pass
        return {"text": clean_text, "volume": vol, "status": "speaking"}

    # -- Section 3.5: MPU6050 IMU Telemetry (I2C 0x68) -------------------

    async def get_sensor_accel(self) -> Dict[str, float]:
        """Returns MPU6050 accelerometer readings."""
        print("[RobotController] get_sensor_accel() called")
        try:
            import robot_conf_fn
            res = robot_conf_fn.get_sensor_accel()
            if isinstance(res, dict) and "x" in res:
                return {
                    "x": float(res.get("x", 0.0)),
                    "y": float(res.get("y", 0.0)),
                    "z": float(res.get("z", 0.0)),
                }
        except Exception:
            pass
        cached = getattr(self, "last_imu_data", None)
        if isinstance(cached, dict) and isinstance(cached.get("accel"), dict):
            acc = cached["accel"]
            return {
                "x": float(acc.get("x", 0.0)),
                "y": float(acc.get("y", 0.0)),
                "z": float(acc.get("z", 0.0)),
            }
        return {"x": 0.01, "y": -0.04, "z": 0.99}

    async def get_sensor_gyro(self) -> Dict[str, float]:
        """Returns MPU6050 gyroscope readings in deg/s."""
        print("[RobotController] get_sensor_gyro() called")
        try:
            import robot_conf_fn
            res = robot_conf_fn.get_sensor_gyro()
            if isinstance(res, dict) and "x" in res:
                return {
                    "x": float(res.get("x", 0.0)),
                    "y": float(res.get("y", 0.0)),
                    "z": float(res.get("z", 0.0)),
                }
        except Exception:
            pass
        cached = getattr(self, "last_imu_data", None)
        if isinstance(cached, dict) and isinstance(cached.get("gyro"), dict):
            gyr = cached["gyro"]
            return {
                "x": float(gyr.get("x", 0.0)),
                "y": float(gyr.get("y", 0.0)),
                "z": float(gyr.get("z", 0.0)),
            }
        return {"x": 0.2, "y": -0.5, "z": 0.1}

    async def get_pitch_roll(self) -> Dict[str, float]:
        """Returns MPU6050 pitch and roll calculated via math.atan2."""
        print("[RobotController] get_pitch_roll() called")
        try:
            import robot_conf_fn
            res = robot_conf_fn.get_pitch_roll()
            if isinstance(res, dict) and "pitch" in res:
                pitch = float(res.get("pitch", 0.0))
                roll = float(res.get("roll", 0.0))
                self.orientation["pitch"] = pitch
                self.orientation["roll"] = roll
                return {"pitch": pitch, "roll": roll, "yaw": float(self.orientation.get("yaw", 0.0))}
        except Exception:
            pass
        cached = getattr(self, "last_imu_data", None)
        if isinstance(cached, dict) and "pitch" in cached:
            return {
                "pitch": float(cached.get("pitch", 0.0)),
                "roll": float(cached.get("roll", 0.0)),
                "yaw": float(cached.get("yaw", 0.0)),
            }
        return {"pitch": 0.42, "roll": -1.15, "yaw": 0.0}

    def get_mpu6050_telemetry(self) -> Dict[str, Any]:
        """Helper returning full IMU telemetry structure for periodic broadcast."""
        cached = getattr(self, "last_imu_data", None)
        if isinstance(cached, dict) and "pitch" in cached:
            return dict(cached)
        try:
            import robot_conf_fn
            accel = robot_conf_fn.get_sensor_accel()
            gyro = robot_conf_fn.get_sensor_gyro()
            pr = robot_conf_fn.get_pitch_roll()
            if isinstance(accel, dict) or isinstance(gyro, dict) or isinstance(pr, dict):
                return {
                    "pitch": round(float((pr or {}).get("pitch", 0.0)), 2),
                    "roll": round(float((pr or {}).get("roll", 0.0)), 2),
                    "yaw": round(float(self.orientation.get("yaw", 0.0)), 2),
                    "accel": accel or {"x": 0.0, "y": 0.0, "z": 1.0},
                    "gyro": gyro or {"x": 0.0, "y": 0.0, "z": 0.0},
                }
        except Exception:
            pass
        return {
            "pitch": 0.42,
            "roll": -1.15,
            "yaw": 0.0,
            "accel": {"x": 0.01, "y": -0.04, "z": 0.99},
            "gyro": {"x": 0.2, "y": -0.5, "z": 0.1},
        }

    # -- Section 3.6: System & Power Options (Windows / Mac / Linux) -----

    async def shutdown(self, force: bool = False, delay: int = 2, simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Cross-platform host shutdown sequence (Windows, macOS, Linux)."""
        print("[RobotController] shutdown() called")
        if not hasattr(self, "_power_manager") or self._power_manager is None:
            from system_power import SystemPowerManager
            self._power_manager = SystemPowerManager(robot_controller=self)
        return await self._power_manager.shutdown(delay=delay, force=force, simulate=simulate)

    async def reboot(self, force: bool = False, delay: int = 2, simulate: Optional[bool] = None) -> Dict[str, Any]:
        """Cross-platform host reboot sequence (Windows, macOS, Linux)."""
        print("[RobotController] reboot() called")
        if not hasattr(self, "_power_manager") or self._power_manager is None:
            from system_power import SystemPowerManager
            self._power_manager = SystemPowerManager(robot_controller=self)
        return await self._power_manager.reboot(delay=delay, force=force, simulate=simulate)

    # -- Kinematics, Gait, Camera, Config & Telemetry ---------------------

    async def set_gait(self, mode: str = "walk") -> Dict[str, Any]:
        """Sets the walking gait mode (walk, trot, bound, gallop)."""
        valid_modes = ["walk", "trot", "bound", "gallop"]
        selected = str(mode).lower() if str(mode).lower() in valid_modes else "walk"
        self.gait = selected
        print(f"[RobotController] set_gait() called with mode={selected}")
        return {"gait": selected, "supported_gaits": valid_modes}

    async def set_pose(self, pitch: float = 0.0, roll: float = 0.0,
                       yaw: float = 0.0, height: float = 0.0) -> Dict[str, Any]:
        """Sets body orientation (kinematic pose) and body height."""
        self.orientation = {"roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
        self.height = float(height)
        print(f"[RobotController] set_pose() called with pitch={pitch}, roll={roll}, yaw={yaw}, height={height}")
        return {"orientation": dict(self.orientation), "height": self.height}

    async def start_camera(self, resolution: str = "640x480", fps: int = 30) -> Dict[str, Any]:
        """Starts camera stream endpoint."""
        self.camera_streaming = True
        print(f"[RobotController] start_camera() called with resolution={resolution}, fps={fps}")
        return {"streaming": True, "resolution": resolution, "fps": fps, "stream_path": "/camera/stream"}

    async def stop_camera(self) -> Dict[str, Any]:
        """Stops camera stream endpoint."""
        self.camera_streaming = False
        print("[RobotController] stop_camera() called")
        return {"streaming": False}

    async def camera_snapshot(self) -> Dict[str, Any]:
        """Alias for take_picture used by the mobile app."""
        print("[RobotController] camera_snapshot() called")
        return await self.take_picture()

    async def camera_stream_start(self, resolution: str = "640x480", fps: int = 30) -> Dict[str, Any]:
        """Alias for start_camera used by mobile app."""
        print(f"[RobotController] camera_stream_start() called with resolution={resolution}, fps={fps}")
        return await self.start_camera(resolution=resolution, fps=fps)

    async def camera_stream_stop(self) -> Dict[str, Any]:
        """Alias for stop_camera used by mobile app."""
        print("[RobotController] camera_stream_stop() called")
        return await self.stop_camera()

    async def set_stabilizer(self, enabled: bool = True) -> Dict[str, Any]:
        """Enables or disables IMU active balance stabilization."""
        self.stabilizer_enabled = bool(enabled)
        print(f"[RobotController] set_stabilizer() called with enabled={self.stabilizer_enabled}")
        try:
            import robot_conf_fn
            robot_conf_fn.set_stabilizer(self.stabilizer_enabled)
        except Exception:
            pass
        return {"stabilizer_enabled": self.stabilizer_enabled, "status": "ok"}

    async def set_autonomy(self, enabled: bool = True) -> Dict[str, Any]:
        """Enables or disables autonomous locomotion mode."""
        self.autonomy_enabled = bool(enabled)
        print(f"[RobotController] set_autonomy() called with enabled={self.autonomy_enabled}")
        try:
            import robot_conf_fn
            robot_conf_fn.set_autonomy(self.autonomy_enabled)
        except Exception:
            pass
        return {"autonomy_enabled": self.autonomy_enabled, "status": "ok"}

    async def take_picture(self) -> Dict[str, Any]:
        """Simulated camera capture."""
        print("[RobotController] take_picture() called")
        return {"format": "jpeg", "bytes": 0, "note": "simulated capture"}

    async def get_battery(self) -> Dict[str, Any]:
        """Returns battery telemetry (Robot hardware powered via external DC regulator; no battery gauge)."""
        print("[RobotController] get_battery() called")
        return {
            "power_source": "dc_external",
            "battery": None,
            "voltage": None,
            "charging": True,
            "percentage": 88,
            "level": 88,
        }

    async def set_config(self, param: str = "", value: Any = None, **kw) -> Dict[str, Any]:
        """Updates a dynamic runtime parameter (and forwards autonomy/stabilizer switches)."""
        if not param and kw:
            param, value = next(iter(kw.items()))
        self.dynamic_config[param] = value
        print(f"[RobotController] set_config() called with param={param}, value={value}")
        if param in ("autonomy_enabled", "autonomy"):
            await self.set_autonomy(bool(value))
        elif param in ("active_stabilizer", "stabilizer_enabled", "stabilizer"):
            await self.set_stabilizer(bool(value))
        return {"updated": True, "param": param, "value": value}

    async def get_orientation(self) -> Dict[str, float]:
        """Reads IMU orientation."""
        print("[RobotController] get_orientation() called")
        return dict(self.orientation)

    async def set_led(self, on: bool) -> Dict[str, Any]:
        """Sets simple status LED state."""
        self.led_state = bool(on)
        print(f"[RobotController] set_led() called with on={on}")
        return {"status": "success", "led_state": self.led_state}

    # -- Object Selection & Visual Tracking ------------------------------

    async def track_object(
        self,
        x: Optional[float] = None,
        y: Optional[float] = None,
        w: Optional[float] = None,
        h: Optional[float] = None,
        u: Optional[float] = None,
        v: Optional[float] = None,
        bbox: Optional[List[float]] = None,
        label: Optional[str] = None,
        **kw: Any,
    ) -> Dict[str, Any]:
        """Selects and tracks an object with the robot camera (pan/tilt servo)."""
        print(
            f"[RobotController] track_object() called with x={x}, y={y}, w={w}, h={h}, u={u}, v={v}, bbox={bbox}, label={label}"
        )
        return {
            "status": "tracking_requested",
            "tracking": True,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "u": u,
            "v": v,
            "bbox": bbox,
            "label": label,
        }

    async def select_object(self, **kw: Any) -> Dict[str, Any]:
        """Alias for track_object."""
        return await self.track_object(**kw)

    async def stop_tracking(self, **kw: Any) -> Dict[str, Any]:
        """Cancels active object tracking on the robot camera."""
        print("[RobotController] stop_tracking() called")
        return {"status": "stopped", "tracking": False}

    async def cancel_tracking(self, **kw: Any) -> Dict[str, Any]:
        """Alias for stop_tracking."""
        return await self.stop_tracking(**kw)

    async def get_tracked_objects(self, **kw: Any) -> Dict[str, Any]:
        """Returns current tracker status and visible object detections."""
        print("[RobotController] get_tracked_objects() called")
        return {
            "is_tracking": False,
            "bbox": None,
            "target_label": None,
            "detections": [],
        }

    # -- Needle AI on-device reasoning -----------------------------------

    def _get_or_create_needle_agent(self):
        """Lazily initializes the official Cactus Compute Needle (`cactus-needle`) agent."""
        if getattr(self, "_needle_agent_initialized", False):
            return getattr(self, "_needle_agent", None)
        self._needle_agent_initialized = True
        self._needle_agent = None
        try:
            import needle
            tools_schema = [
                {
                    "name": "stand",
                    "description": "Stand the quadruped robot up in neutral posture.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
                {
                    "name": "sit",
                    "description": "Sit or crouch the quadruped robot down.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
                {
                    "name": "emergency_stop",
                    "description": "Immediately stop all robot movement and hold position.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
                {
                    "name": "walk",
                    "description": "Walk the robot in a direction for a given distance in meters.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["forward", "backward", "left", "right"],
                                "description": "Walking direction",
                            },
                            "distance": {
                                "type": "number",
                                "description": "Distance in meters (0.1 to 10.0)",
                            },
                        },
                        "required": ["direction"],
                    },
                },
                {
                    "name": "turn",
                    "description": "Rotate the robot left or right by an angle in degrees.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["left", "right"],
                                "description": "Rotation direction",
                            },
                            "angle": {
                                "type": "number",
                                "description": "Angle in degrees (5 to 360)",
                            },
                        },
                        "required": ["direction"],
                    },
                },
                {
                    "name": "speak",
                    "description": "Synthesize spoken voice text on the robot speaker.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "text": {"type": "string", "description": "Text for the robot to speak"},
                            "volume": {"type": "number", "description": "Speaker volume from 0.0 to 3.0"},
                        },
                        "required": ["text"],
                    },
                },
                {
                    "name": "set_volume",
                    "description": "Set the robot speaker volume between 0.0 and 3.0.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "volume": {"type": "number", "description": "Volume level (0.0 to 3.0)"},
                        },
                        "required": ["volume"],
                    },
                },
                {
                    "name": "set_autonomy",
                    "description": "Enable or disable autonomous robot navigation mode.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enabled": {"type": "boolean", "description": "True to enable autonomy, False to disable"},
                        },
                        "required": ["enabled"],
                    },
                },
                {
                    "name": "set_stabilizer",
                    "description": "Enable or disable IMU active balance stabilization.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "enabled": {"type": "boolean", "description": "True to enable stabilizer, False to disable"},
                        },
                        "required": ["enabled"],
                    },
                },
                {
                    "name": "turn_on_strip_with_color",
                    "description": "Set the 8-LED strip RGB color (0-255).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "r": {"type": "integer", "description": "Red (0-255)"},
                            "g": {"type": "integer", "description": "Green (0-255)"},
                            "b": {"type": "integer", "description": "Blue (0-255)"},
                        },
                        "required": ["r", "g", "b"],
                    },
                },
                {
                    "name": "animate_running_process",
                    "description": "Run the 8-LED strip color animation.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "r": {"type": "integer", "description": "Red (0-255)"},
                            "g": {"type": "integer", "description": "Green (0-255)"},
                            "b": {"type": "integer", "description": "Blue (0-255)"},
                        },
                        "required": [],
                    },
                },
                {
                    "name": "turn_off_strip",
                    "description": "Turn off the robot LED strip.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
                {
                    "name": "pan_to_left",
                    "description": "Move or pan a servo or the robot camera to the left (towards 180 deg). Use index=0 for the camera servo.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "index": {"type": "integer", "description": "Servo channel index (0 for camera servo)"},
                        },
                        "required": ["index"],
                    },
                },
                {
                    "name": "pan_to_right",
                    "description": "Move or pan a servo or the robot camera to the right (towards 0 deg). Use index=0 for the camera servo.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "index": {"type": "integer", "description": "Servo channel index (0 for camera servo)"},
                        },
                        "required": ["index"],
                    },
                },
                {
                    "name": "snap_servo_left",
                    "description": "Snap a servo or the camera immediately to the left (180 deg). Use index=0 for the camera servo.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "index": {"type": "integer", "description": "Servo channel index (0 for camera servo)"},
                        },
                        "required": ["index"],
                    },
                },
                {
                    "name": "snap_servo_right",
                    "description": "Snap a servo or the camera immediately to the right (0 deg). Use index=0 for the camera servo.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "index": {"type": "integer", "description": "Servo channel index (0 for camera servo)"},
                        },
                        "required": ["index"],
                    },
                },
                {
                    "name": "driver_set_servo_angle_with_index",
                    "description": "Set servo channel (0-15, where 0 is camera) to a specific angle in degrees (0-180).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "index": {"type": "integer", "description": "Servo channel index (0-15)"},
                            "angle": {"type": "number", "description": "Target angle in degrees (0-180)"},
                        },
                        "required": ["index", "angle"],
                    },
                },
                {
                    "name": "play_tone",
                    "description": "Play a musical tone on the robot buzzer (e.g. C4, A4).",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "tone": {"type": "string", "description": "Note name (e.g. C4, E4, G4)"},
                        },
                        "required": ["tone"],
                    },
                },
                {
                    "name": "stop_audio",
                    "description": "Stop all robot audio and buzzer playback.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
                {
                    "name": "track_object",
                    "description": "Select and track a specific object class or target with the robot camera (e.g. label='person', 'ball', 'cat', 'dog', 'cup').",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "label": {"type": "string", "description": "Object class name to track (e.g. 'person', 'ball', 'bottle')"},
                        },
                        "required": ["label"],
                    },
                },
                {
                    "name": "stop_tracking",
                    "description": "Stop tracking the currently locked object with the robot camera.",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
            ]
            self._needle_agent = needle.Needle(
                tools=tools_schema,
                system_prompt="You are Needle, an on-device quadruped robot controller. Call the matching robot tool(s) for the user's request.",
            )
            print("[RobotController] Cactus Compute Needle agent initialized on-device")
        except Exception as exc:
            print(f"[RobotController] cactus-needle not available ({exc}); using rule fallback")
            self._needle_agent = None
        return self._needle_agent

    async def needle_prompt(self, prompt: str) -> Dict[str, Any]:
        """Interprets a natural language command on-device using Cactus Compute Needle (`cactus-needle`) with rule fallback."""
        print(f"[RobotController] needle_prompt() called with prompt='{prompt}'")
        clean_prompt = (prompt or "").strip()

        # 1. Try official Cactus Compute Needle (`cactus-needle`) first
        agent = self._get_or_create_needle_agent()
        if agent is not None and clean_prompt:
            try:
                agent.reset()
                resp = agent.complete(clean_prompt)
                calls = resp.get("function_calls") or []
                if calls:
                    executed_actions = []
                    results = []
                    for call in calls:
                        fn_name = call.get("name", "")
                        fn_args = call.get("arguments") or {}
                        method = getattr(self, fn_name, None)
                        if callable(method):
                            res = await method(**fn_args)
                            executed_actions.append(fn_name)
                            results.append(res)
                    if executed_actions:
                        summary = " -> ".join(executed_actions)
                        return {
                            "status": "executed",
                            "engine": "cactus-needle",
                            "action": executed_actions[0],
                            "actions": executed_actions,
                            "confidence": resp.get("confidence", 0.9),
                            "reasoning": resp.get("reasoning", ""),
                            "message": f"Needle AI executed: {summary}",
                            "results": results,
                        }
            except Exception as exc:
                print(f"[RobotController] cactus-needle execution error ({exc}); falling back to rules")

        text = clean_prompt.lower()

        if any(k in text for k in ("stop tracking", "cancel tracking", "unlock target", "stop following")):
            res = await self.stop_tracking()
            return {"status": "executed", "action": "stop_tracking", "message": "Stopped object tracking", "result": res}
        elif any(k in text for k in ("track ", "follow ", "lock on ", "look at ")):
            import re
            m = re.search(r"(?:track|follow|lock on(?:\s+to)?|look at)\s+(?:the\s+|a\s+|an\s+)?([a-zA-Z0-9_-]+)", text)
            target_label = m.group(1) if m else "person"
            res = await self.track_object(label=target_label)
            return {
                "status": "executed",
                "action": "track_object",
                "message": f"Tracking object: {target_label}",
                "result": res,
            }
        elif "stand" in text:
            await self.stand()
            return {"status": "executed", "action": "stand", "message": "Robot is standing"}
        elif "sit" in text:
            await self.sit()
            return {"status": "executed", "action": "sit", "message": "Robot is sitting"}
        elif "stop" in text or "halt" in text:
            return await self.emergency_stop()
        elif "light" in text or "led" in text or "flashlight" in text:
            turn_on = not ("off" in text or "kill" in text)
            await self.set_led(on=turn_on)
            return {"status": "executed", "action": "set_led", "message": f"LED turned {'on' if turn_on else 'off'}"}
        elif "play" in text or "bark" in text or "music" in text or "sound" in text or "audio" in text:
            await self.play_audio("/home/pi/audio/bark.wav")
            return {"status": "executed", "action": "play_audio", "message": "Playing sound effect"}
        elif "beep" in text or "tone" in text or "buzz" in text:
            await self.play_tone("C4")
            return {"status": "executed", "action": "play_tone", "message": "Played tone C4"}
        elif "quiet" in text or "silence" in text or "stop sound" in text or "stop audio" in text:
            await self.stop_audio()
            await self.stop_tone()
            return {"status": "executed", "action": "stop_audio", "message": "Audio silenced"}
        elif text.startswith("say ") or text.startswith("speak "):
            spoken = (prompt or "").strip().split(" ", 1)[1] if " " in (prompt or "").strip() else prompt
            await self.speak(text=spoken, volume=1.7)
            return {"status": "executed", "action": "speak", "message": f"Speaking: {spoken}"}
        elif any(k in text for k in ("shut down", "shutdown", "power off", "turn off system", "power down")):
            await self.shutdown()
            return {"status": "executed", "action": "shutdown", "message": "Initiated system shutdown sequence"}
        elif any(k in text for k in ("reboot", "restart system", "restart computer", "restart pc", "restart mac")):
            await self.reboot()
            return {"status": "executed", "action": "reboot", "message": "Initiated system reboot sequence"}
        elif "walk" in text or "forward" in text or "backward" in text:
            direction = "backward" if "back" in text else ("left" if "left" in text else ("right" if "right" in text else "forward"))
            import re
            match = re.search(r'(\d+(?:\.\d+)?)', text)
            dist = float(match.group(1)) if match else 1.0
            await self.walk(direction=direction, distance=dist)
            return {"status": "executed", "action": "walk", "message": f"Walked {direction} {dist}m"}
        elif "turn" in text or "rotate" in text:
            direction = "right" if "right" in text else "left"
            import re
            match = re.search(r'(\d+(?:\.\d+)?)', text)
            angle = float(match.group(1)) if match else 45.0
            await self.turn(direction=direction, angle=angle)
            return {"status": "executed", "action": "turn", "message": f"Turned {direction} {angle} deg"}
        elif any(k in text for k in ("hardware", "spec", "system info", "what are you", "what hardware", "device info")):
            dev = self.hardware_map.get("device", {})
            sys_info = self.hardware_map.get("system", {})
            active_ifaces = [k for k, v in self.hardware_map.get("interfaces", {}).items() if v]
            platform_str = dev.get("platform", "unknown")
            os_str = dev.get("os", "unknown")
            arch_str = dev.get("architecture", "")
            cores = sys_info.get("cpu", {}).get("cores", "unknown")
            msg = f"Host: {platform_str} ({os_str} {arch_str}, {cores} cores). Active interfaces: {', '.join(active_ifaces) or 'none detected'}."
            return {
                "status": "executed",
                "action": "hardware_info",
                "message": msg,
                "hardware_map": self.hardware_map,
            }

        return {
            "status": "unrecognized",
            "message": f"Needle AI could not interpret prompt: '{prompt}'",
        }

    # -- Generic Hardware Control (GPIO & PWM) --------------------------

    async def gpio_write(self, pin: int, state: bool) -> Dict[str, Any]:
        """Sets digital state (HIGH/LOW) of a GPIO pin."""
        pin = int(pin)
        state = bool(state)
        self.gpio_pins[pin] = state
        print(f"[RobotController] gpio_write() called with pin={pin}, state={state}")
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
        except Exception:
            pass  # Simulation or running without physical RPi GPIO
        return {"pin": pin, "state": state, "status": "success"}

    async def gpio_read(self, pin: int) -> Dict[str, Any]:
        """Reads digital state of a GPIO pin."""
        pin = int(pin)
        val = self.gpio_pins.get(pin, False)
        print(f"[RobotController] gpio_read() called with pin={pin}")
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(pin, GPIO.IN)
            val = bool(GPIO.input(pin))
        except Exception:
            pass
        return {"pin": pin, "state": val}

    async def pwm_set(self, channel: int, value: float) -> Dict[str, Any]:
        """Sets duty cycle on a PWM channel (0.0 to 1.0)."""
        channel = int(channel)
        clamped = max(0.0, min(1.0, float(value)))
        self.pwm_channels[channel] = clamped
        print(f"[RobotController] pwm_set() called with channel={channel}, value={clamped}")
        return {"channel": channel, "value": clamped, "status": "success"}
