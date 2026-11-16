# robot_pi/sensors/imu_handler.py
"""IMU Reader and Telemetry Integrator for MPU-6050.

Reads accelerometer and gyroscope data from gyro_imp.py without console print spam,
calculates pitch/roll in a single I2C transaction, and integrates Z-gyro angular velocity.
"""

import logging
import math
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("robot.imu")


class IMUHandler:
    def __init__(self, sample_rate_hz: float = 50.0):
        self.sample_rate = sample_rate_hz
        self.heading_yaw: float = 0.0
        self._last_timestamp: float = time.time()
        self._gyro_z_offset: float = 0.0

        self.last_accel = {"x": 0.0, "y": 0.0, "z": 1.0}
        self.last_gyro = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_pitch: float = 0.0
        self.last_roll: float = 0.0

        self._driver = None
        self._sensor = None
        self._init_driver()

    def _init_driver(self):
        try:
            from drivers import gyro_imp
            self._driver = gyro_imp
            self._sensor = getattr(gyro_imp, "sensor", None)
            logger.info("MPU-6050 driver linked to IMUHandler.")
            self.calibrate(samples=30)
        except Exception as e:
            logger.warning("Could not load gyro_imp: %s. Using simulated IMU.", e)
            self._driver = None
            self._sensor = None

    def _read_accel_quiet(self) -> Optional[Dict[str, float]]:
        try:
            if self._sensor is not None:
                return self._sensor.get_accel_data()
            if self._driver is not None:
                return self._driver.get_sensor_accel()
        except Exception:
            pass
        return None

    def _read_gyro_quiet(self) -> Optional[Dict[str, float]]:
        try:
            if self._sensor is not None:
                return self._sensor.get_gyro_data()
            if self._driver is not None:
                return self._driver.get_sensor_gyro()
        except Exception:
            pass
        return None

    def calibrate(self, samples: int = 30):
        """Calculates zero-rate gyroscope bias offset while stationary."""
        if self._driver is None and self._sensor is None:
            return
        total_z = 0.0
        count = 0
        for _ in range(samples):
            data = self._read_gyro_quiet()
            if data and "z" in data:
                total_z += data["z"]
                count += 1
            time.sleep(0.01)
        if count > 0:
            self._gyro_z_offset = total_z / count
            logger.info("Calibrated MPU-6050 Gyro Z offset: %.3f deg/s", self._gyro_z_offset)

    def update(self) -> Dict[str, Any]:
        """Polls sensor quietly, updates orientation filters, and integrates heading."""
        now = time.time()
        dt = max(0.001, min(0.2, now - self._last_timestamp))
        self._last_timestamp = now

        if self._driver is not None or self._sensor is not None:
            try:
                accel = self._read_accel_quiet()
                gyro = self._read_gyro_quiet()

                if accel:
                    self.last_accel = accel
                    ax = accel.get("x", 0.0)
                    ay = accel.get("y", 0.0)
                    az = accel.get("z", 1.0)
                    self.last_pitch = math.atan2(ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi
                    self.last_roll = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi

                if gyro:
                    self.last_gyro = gyro
                    raw_gz = gyro.get("z", 0.0) - self._gyro_z_offset
                    self.heading_yaw = (self.heading_yaw + raw_gz * dt) % 360.0

            except Exception as e:
                logger.debug("IMU read failed: %s", e)

        return self.get_telemetry()

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns standard IMU telemetry structure."""
        return {
            "pitch": round(self.last_pitch, 2),
            "roll": round(self.last_roll, 2),
            "yaw": round(self.heading_yaw, 2),
            "accel": self.last_accel,
            "gyro": self.last_gyro,
            "timestamp": self._last_timestamp,
        }

    def reset_heading(self, heading: float = 0.0):
        """Resets the integrated heading yaw to zero or a known angle."""
        self.heading_yaw = float(heading)
