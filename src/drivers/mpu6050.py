"""
MPU6050 — 6-Axis IMU (Accelerometer + Gyroscope) Driver
=========================================================
InvenSense MPU6050 motion tracking device.
I²C interface, 16-bit ADC, ±2g/±4g/±8g/±16g accel, ±250/500/1000/2000°/s gyro.
"""

import struct
import time
import math
from typing import Optional, Dict
from .base_sensor import BaseSensor, SensorReading


class MPU6050(BaseSensor):
    """
    MPU6050 6-axis IMU driver with tilt angle computation.

    Parameters
    ----------
    bus_id : int
        I²C bus number.
    address : int
        I²C address (default: 0x68, alt: 0x69 with AD0 high).
    accel_range : str
        Accelerometer range: '2g', '4g', '8g', '16g'.
    gyro_range : str
        Gyroscope range: '250dps', '500dps', '1000dps', '2000dps'.
    """

    REG_PWR_MGMT_1 = 0x6B
    REG_SMPLRT_DIV = 0x19
    REG_CONFIG = 0x1A
    REG_ACCEL_CONFIG = 0x1C
    REG_GYRO_CONFIG = 0x1B
    REG_ACCEL_XOUT_H = 0x3B
    REG_TEMP_OUT_H = 0x41
    REG_WHO_AM_I = 0x75
    CHIP_ID = 0x68

    ACCEL_RANGES = {'2g': (0x00, 16384.0), '4g': (0x08, 8192.0),
                    '8g': (0x10, 4096.0), '16g': (0x18, 2048.0)}
    GYRO_RANGES = {'250dps': (0x00, 131.0), '500dps': (0x08, 65.5),
                   '1000dps': (0x10, 32.8), '2000dps': (0x18, 16.4)}

    def __init__(self, bus_id: int = 1, address: int = 0x68,
                 accel_range: str = '4g', gyro_range: str = '500dps',
                 sensor_id: Optional[str] = None):
        super().__init__(
            sensor_id=sensor_id or f"mpu6050_{address:#x}",
            sensor_type="imu",
            bus_id=bus_id,
            address=address,
        )
        self._accel_reg, self._accel_scale = self.ACCEL_RANGES.get(accel_range, self.ACCEL_RANGES['4g'])
        self._gyro_reg, self._gyro_scale = self.GYRO_RANGES.get(gyro_range, self.GYRO_RANGES['500dps'])
        self._bus = None
        self._sim_mode = True
        self._sim_t = 0.0

    def _init_hardware(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus_id)
            self._sim_mode = False

            who = self._bus.read_byte_data(self.address, self.REG_WHO_AM_I)
            if who != self.CHIP_ID:
                raise ValueError(f"Unexpected WHO_AM_I: 0x{who:02X}")

            # Wake up (clear sleep bit)
            self._bus.write_byte_data(self.address, self.REG_PWR_MGMT_1, 0x00)
            time.sleep(0.1)

            # Configure sample rate divider (1 kHz / (1 + 9) = 100 Hz)
            self._bus.write_byte_data(self.address, self.REG_SMPLRT_DIV, 9)
            # Low-pass filter: ~44 Hz bandwidth
            self._bus.write_byte_data(self.address, self.REG_CONFIG, 0x03)
            # Set accel and gyro ranges
            self._bus.write_byte_data(self.address, self.REG_ACCEL_CONFIG, self._accel_reg)
            self._bus.write_byte_data(self.address, self.REG_GYRO_CONFIG, self._gyro_reg)
            time.sleep(0.05)

        except (ImportError, FileNotFoundError, OSError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            return self._simulate_raw()
        return bytes(self._bus.read_i2c_block_data(self.address, self.REG_ACCEL_XOUT_H, 14))

    def _simulate_raw(self) -> bytes:
        import random
        self._sim_t += 0.01
        # Simulate gentle motion
        ax = 0.0 + random.gauss(0, 0.02)
        ay = 0.0 + random.gauss(0, 0.02)
        az = 1.0 + random.gauss(0, 0.01)  # Gravity
        gx = 5.0 * math.sin(self._sim_t * 0.5) + random.gauss(0, 0.5)
        gy = 3.0 * math.cos(self._sim_t * 0.3) + random.gauss(0, 0.5)
        gz = random.gauss(0, 0.3)
        temp_raw = int((25.0 + random.gauss(0, 0.1)) * 340 + 36.53 * 340)

        # Convert to raw ADC values
        ax_raw = int(ax * self._accel_scale)
        ay_raw = int(ay * self._accel_scale)
        az_raw = int(az * self._accel_scale)
        gx_raw = int(gx * self._gyro_scale)
        gy_raw = int(gy * self._gyro_scale)
        gz_raw = int(gz * self._gyro_scale)

        return struct.pack('>hhhhhhh', ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw)

    def _parse(self, raw: bytes) -> SensorReading:
        ax_r, ay_r, az_r, temp_r, gx_r, gy_r, gz_r = struct.unpack('>hhhhhhh', raw[:14])

        ax = ax_r / self._accel_scale
        ay = ay_r / self._accel_scale
        az = az_r / self._accel_scale
        gx = gx_r / self._gyro_scale
        gy = gy_r / self._gyro_scale
        gz = gz_r / self._gyro_scale
        temp = temp_r / 340.0 + 36.53

        # Compute tilt angles from accelerometer
        pitch = math.atan2(ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi
        roll = math.atan2(ay, math.sqrt(ax * ax + az * az)) * 180.0 / math.pi

        return SensorReading(
            sensor_id=self.sensor_id,
            sensor_type=self.sensor_type,
            values={
                "accel_x": round(ax, 4),
                "accel_y": round(ay, 4),
                "accel_z": round(az, 4),
                "gyro_x": round(gx, 2),
                "gyro_y": round(gy, 2),
                "gyro_z": round(gz, 2),
                "temperature": round(temp, 1),
                "pitch": round(pitch, 2),
                "roll": round(roll, 2),
            },
            unit="g/°s/°C/°",
            quality=1.0 if abs(math.sqrt(ax**2 + ay**2 + az**2) - 1.0) < 0.2 else 0.7,
            raw=raw,
        )
