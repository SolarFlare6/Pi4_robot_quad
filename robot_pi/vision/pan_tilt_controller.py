# robot_pi/vision/pan_tilt_controller.py
"""PID Pan and Tilt servo motor controller for the robot's camera mount.

Controls Servo Channel 0 (CAM_X - Pan) and Channel 1 (CAM_Y - Tilt) via PCA9685.

Features:
1. **True Discrete PID Control (`dt`-normalized) with Anti-Windup**:
   - **Proportional (P)**: Uses soft deadband-subtracted error so the servo starts moving
     gently instead of jumping when the outer trigger threshold is crossed.
   - **Integral (I)**: Eliminates steady-state lag when following a smoothly moving target,
     with anti-windup clamping and automatic zero-crossing reset to prevent overshoot.
   - **Derivative (D)**: Low-pass filtered derivative braking (`d(err)/dt`) that slows the
     servo down as the object approaches the center settle zone.
2. **Eliminated Target Windup & Slew-Rate Limiting**:
   - Updates `current_pan` / `current_tilt` directly (no separate `target_pan` accumulator
     running ahead of the physical servo) and caps maximum angular velocity per step.
"""

import logging
import math
import time
from typing import Optional, Tuple

logger = logging.getLogger("robot.pan_tilt")


class PanTiltController:
    def __init__(
        self,
        pan_channel: int = 0,
        tilt_channel: int = 1,
        pan_center: float = 90.0,
        tilt_center: float = 52.0,
        pan_limits: Tuple[int, int] = (0, 180),
        tilt_limits: Tuple[int, int] = (0, 180),
        # Outer trigger zone: servo starts moving when object moves beyond this offset from center
        trigger_x: float = 85.0,
        trigger_y: float = 65.0,
        # Inner settle zone: once moving, servo smoothly brings object back within this offset and stops
        settle_x: float = 28.0,
        settle_y: float = 22.0,
        # Minimum distance (px) allowed between object bbox edge and camera border before servo moves
        edge_margin_px: int = 55,
        # PID gains (output in degrees/second per pixel error)
        kp_pan: float = 0.14,
        ki_pan: float = 0.025,
        kd_pan: float = 0.012,
        kp_tilt: float = 0.12,
        ki_tilt: float = 0.020,
        kd_tilt: float = 0.010,
        # Anti-windup & velocity limits
        max_integral_px_s: float = 180.0,
        max_speed_deg_s: float = 24.0,   # Max ~0.8 deg per frame at 30 FPS -> smooth, non-aggressive motion
        d_filter_alpha: float = 0.35,    # Low-pass filter on derivative term to suppress bbox jitter
    ):
        self.pan_channel = pan_channel
        self.tilt_channel = tilt_channel
        self.pan_center = float(pan_center)
        self.tilt_center = float(tilt_center)
        self.pan_min, self.pan_max = pan_limits
        self.tilt_min, self.tilt_max = tilt_limits

        self.current_pan: float = self.pan_center
        self.current_tilt: float = self.tilt_center
        self.target_pan: float = self.pan_center
        self.target_tilt: float = self.tilt_center

        self.trigger_x = trigger_x
        self.trigger_y = trigger_y
        self.settle_x = settle_x
        self.settle_y = settle_y
        self.edge_margin_px = edge_margin_px

        self.kp_pan = kp_pan
        self.ki_pan = ki_pan
        self.kd_pan = kd_pan
        self.kp_tilt = kp_tilt
        self.ki_tilt = ki_tilt
        self.kd_tilt = kd_tilt

        self.max_integral = max_integral_px_s
        self.max_speed_deg_s = max_speed_deg_s
        self.d_filter_alpha = d_filter_alpha

        # Hysteresis & PID internal states
        self._moving_x: bool = False
        self._moving_y: bool = False
        self._integral_x: float = 0.0
        self._integral_y: float = 0.0
        self._prev_err_x: float = 0.0
        self._prev_err_y: float = 0.0
        self._d_term_x: float = 0.0
        self._d_term_y: float = 0.0
        self._last_update_time: float = 0.0

        # Last applied float angles to avoid redundant I2C writes when stationary
        self._last_hw_pan: float = -999.0
        self._last_hw_tilt: float = -999.0

        self._driver = None
        self._init_driver()

    def _init_driver(self):
        try:
            from drivers import servo_driver_imp
            self._driver = servo_driver_imp
            try:
                self._driver.set_servo_range(self.pan_channel, 180)
                self._driver.set_servo_range(self.tilt_channel, 180)
            except Exception as e:
                logger.debug("Servo range init skipped: %s", e)
            logger.info("PCA9685 servo driver linked to precision Pan/Tilt controller.")
        except Exception as e:
            logger.warning("Could not load servo_driver_imp: %s. Using simulated pan/tilt.", e)
            self._driver = None

    def set_pan(self, angle: float) -> float:
        """Sets the pan (horizontal) angle with sub-degree precision."""
        clamped = max(float(self.pan_min), min(float(self.pan_max), float(angle)))
        self.current_pan = clamped
        self.target_pan = clamped
        if self._driver is not None and abs(clamped - self._last_hw_pan) >= 0.08:
            try:
                self._driver.driver_set_servo_angle_with_index(self.pan_channel, round(clamped, 2))
                self._last_hw_pan = clamped
            except Exception as e:
                logger.debug("Error updating pan servo: %s", e)
        return self.current_pan

    def set_tilt(self, angle: float) -> float:
        """Sets the tilt (vertical) angle with sub-degree precision."""
        clamped = max(float(self.tilt_min), min(float(self.tilt_max), float(angle)))
        self.current_tilt = clamped
        self.target_tilt = clamped
        if self._driver is not None and abs(clamped - self._last_hw_tilt) >= 0.08:
            try:
                self._driver.driver_set_servo_angle_with_index(self.tilt_channel, round(clamped, 2))
                self._last_hw_tilt = clamped
            except Exception as e:
                logger.debug("Error updating tilt servo: %s", e)
        return self.current_tilt

    def set_angles(self, pan: float, tilt: float) -> Tuple[float, float]:
        """Sets both pan and tilt angles simultaneously."""
        return self.set_pan(pan), self.set_tilt(tilt)

    def reset_pid(self):
        """Resets PID integrals, derivatives, and hysteresis states."""
        self._moving_x = False
        self._moving_y = False
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_err_x = 0.0
        self._prev_err_y = 0.0
        self._d_term_x = 0.0
        self._d_term_y = 0.0
        self._last_update_time = 0.0

    def center(self) -> Tuple[float, float]:
        """Resets camera orientation to centered forward position (X=90, Y=52)."""
        self.reset_pid()
        return self.set_angles(self.pan_center, self.tilt_center)

    def get_angles(self) -> Tuple[float, float]:
        """Returns the current (pan, tilt) angles in degrees."""
        return round(self.current_pan, 2), round(self.current_tilt, 2)

    @staticmethod
    def _soft_error(err: float, deadband: float) -> float:
        """Subtracts the inner settle deadband so PID starts gently from 0 instead of jumping."""
        if abs(err) <= deadband:
            return 0.0
        return math.copysign(abs(err) - deadband, err)

    def track_error(
        self,
        err_x: float,
        err_y: float,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        frame_w: int = 640,
        frame_h: int = 480,
    ) -> Tuple[float, float]:
        """Moves the camera servos ONLY when the object starts moving outside the safe camera view zone,
        then uses a true discrete PID controller (with anti-windup and derivative braking) to smoothly
        bring the target back into view without aggressive jumps or overshoot.
        """
        now = time.time()
        if self._last_update_time <= 0.0:
            dt = 0.033
        else:
            dt = max(0.010, min(0.120, now - self._last_update_time))
        self._last_update_time = now

        # Check if the object's bounding box is approaching the outer edges of the camera frame
        near_left_or_right_edge = False
        near_top_or_bottom_edge = False
        if bbox is not None and len(bbox) == 4:
            bx, by, bw, bh = bbox
            if (bx <= self.edge_margin_px and err_x < 0) or (bx + bw >= frame_w - self.edge_margin_px and err_x > 0):
                near_left_or_right_edge = True
            if (by <= self.edge_margin_px and err_y < 0) or (by + bh >= frame_h - self.edge_margin_px and err_y > 0):
                near_top_or_bottom_edge = True

        # 1. Hysteresis check for Horizontal Pan (X)
        if abs(err_x) >= self.trigger_x or near_left_or_right_edge:
            self._moving_x = True
        elif abs(err_x) <= self.settle_x:
            self._moving_x = False
            self._integral_x = 0.0
            self._d_term_x = 0.0

        # 2. Hysteresis check for Vertical Tilt (Y)
        if abs(err_y) >= self.trigger_y or near_top_or_bottom_edge:
            self._moving_y = True
        elif abs(err_y) <= self.settle_y:
            self._moving_y = False
            self._integral_y = 0.0
            self._d_term_y = 0.0

        # Reset integral on zero-crossing to prevent overshoot
        if (err_x * self._prev_err_x) < 0.0:
            self._integral_x = 0.0
        if (err_y * self._prev_err_y) < 0.0:
            self._integral_y = 0.0

        # Low-pass filtered derivative (rate of change of error in px/sec)
        raw_dx = (err_x - self._prev_err_x) / dt if self._prev_err_x != 0.0 else 0.0
        raw_dy = (err_y - self._prev_err_y) / dt if self._prev_err_y != 0.0 else 0.0
        self._d_term_x = (self.d_filter_alpha * raw_dx) + ((1.0 - self.d_filter_alpha) * self._d_term_x)
        self._d_term_y = (self.d_filter_alpha * raw_dy) + ((1.0 - self.d_filter_alpha) * self._d_term_y)
        self._prev_err_x = err_x
        self._prev_err_y = err_y

        max_step_this_frame = self.max_speed_deg_s * dt

        # 3. PID Horizontal Pan (X-axis)
        #    Left (err_x < 0) -> move toward 180 (increase angle)
        #    Right (err_x > 0) -> move toward 0 (decrease angle)
        if self._moving_x:
            eff_x = self._soft_error(err_x, self.settle_x)
            self._integral_x = max(-self.max_integral, min(self.max_integral, self._integral_x + eff_x * dt))
            vel_x_deg_s = (self.kp_pan * eff_x) + (self.ki_pan * self._integral_x) + (self.kd_pan * self._d_term_x)
            # Derivative braking slows or stops the servo as error shrinks, without reversing direction
            if (vel_x_deg_s * eff_x) < 0.0:
                vel_x_deg_s = 0.0
            step_x = max(-max_step_this_frame, min(max_step_this_frame, vel_x_deg_s * dt))
            self.current_pan -= step_x
            self.target_pan = self.current_pan

        # 4. PID Vertical Tilt (Y-axis)
        #    Up (err_y < 0) -> move toward 0 (decrease angle)
        #    Down (err_y > 0) -> move toward 180 (increase angle)
        if self._moving_y:
            eff_y = self._soft_error(err_y, self.settle_y)
            self._integral_y = max(-self.max_integral, min(self.max_integral, self._integral_y + eff_y * dt))
            vel_y_deg_s = (self.kp_tilt * eff_y) + (self.ki_tilt * self._integral_y) + (self.kd_tilt * self._d_term_y)
            if (vel_y_deg_s * eff_y) < 0.0:
                vel_y_deg_s = 0.0
            step_y = max(-max_step_this_frame, min(max_step_this_frame, vel_y_deg_s * dt))
            self.current_tilt += step_y
            self.target_tilt = self.current_tilt

        self.current_pan = max(float(self.pan_min), min(float(self.pan_max), self.current_pan))
        self.current_tilt = max(float(self.tilt_min), min(float(self.tilt_max), self.current_tilt))

        if self._driver is not None:
            if abs(self.current_pan - self._last_hw_pan) >= 0.08:
                try:
                    self._driver.driver_set_servo_angle_with_index(self.pan_channel, round(self.current_pan, 2))
                    self._last_hw_pan = self.current_pan
                except Exception as e:
                    logger.debug("Error updating pan servo: %s", e)
            if abs(self.current_tilt - self._last_hw_tilt) >= 0.08:
                try:
                    self._driver.driver_set_servo_angle_with_index(self.tilt_channel, round(self.current_tilt, 2))
                    self._last_hw_tilt = self.current_tilt
                except Exception as e:
                    logger.debug("Error updating tilt servo: %s", e)

        return round(self.current_pan, 2), round(self.current_tilt, 2)
