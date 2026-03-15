"""
BME280 — Temperature, Humidity & Pressure Sensor Driver
========================================================
Bosch BME280 digital environmental sensor.
I²C interface, 16-bit resolution, ±1°C / ±3% RH / ±1 hPa accuracy.
"""

import struct
import time
from typing import Optional
from .base_sensor import BaseSensor, SensorReading


class BME280(BaseSensor):
    """
    BME280 environmental sensor driver.

    Parameters
    ----------
    bus_id : int
        I²C bus number (default: 1).
    address : int
        I²C address (default: 0x76, alt: 0x77).
    oversample : int
        Oversampling factor: 1, 2, 4, 8, or 16 (default: 16).
    """

    # Register addresses
    REG_CHIP_ID = 0xD0
    REG_RESET = 0xE0
    REG_CTRL_HUM = 0xF2
    REG_STATUS = 0xF3
    REG_CTRL_MEAS = 0xF4
    REG_CONFIG = 0xF5
    REG_DATA = 0xF7
    REG_CALIB_00 = 0x88
    REG_CALIB_26 = 0xE1

    CHIP_ID = 0x60
    OVERSAMPLE_MAP = {1: 0x01, 2: 0x02, 4: 0x03, 8: 0x04, 16: 0x05}

    def __init__(self, bus_id: int = 1, address: int = 0x76,
                 oversample: int = 16, sensor_id: Optional[str] = None):
        super().__init__(
            sensor_id=sensor_id or f"bme280_{address:#x}",
            sensor_type="environmental",
            bus_id=bus_id,
            address=address,
        )
        self.oversample = self.OVERSAMPLE_MAP.get(oversample, 0x05)
        self._bus = None
        self._calib = {}

        # Simulation mode (for desktop testing)
        self._sim_mode = True
        self._sim_temp = 22.5
        self._sim_hum = 45.0
        self._sim_pres = 1013.25

    def _init_hardware(self):
        """Initialize BME280 registers and read calibration data."""
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus_id)
            self._sim_mode = False

            # Verify chip ID
            chip_id = self._bus.read_byte_data(self.address, self.REG_CHIP_ID)
            if chip_id != self.CHIP_ID:
                raise ValueError(f"Unexpected chip ID: 0x{chip_id:02X}")

            # Soft reset
            self._bus.write_byte_data(self.address, self.REG_RESET, 0xB6)
            time.sleep(0.01)

            # Read calibration data
            self._read_calibration()

            # Configure: oversample T/H/P, normal mode
            self._bus.write_byte_data(self.address, self.REG_CTRL_HUM, self.oversample)
            ctrl = (self.oversample << 5) | (self.oversample << 2) | 0x03
            self._bus.write_byte_data(self.address, self.REG_CTRL_MEAS, ctrl)
            self._bus.write_byte_data(self.address, self.REG_CONFIG, 0x00)
            time.sleep(0.05)

        except (ImportError, FileNotFoundError, OSError):
            # Fallback to simulation mode
            self._sim_mode = True
            self._read_calibration_sim()

    def _read_calibration(self):
        """Read factory calibration coefficients from sensor."""
        if self._sim_mode:
            self._read_calibration_sim()
            return

        # Temperature & pressure calibration (0x88–0x9F)
        cal1 = self._bus.read_i2c_block_data(self.address, self.REG_CALIB_00, 26)
        # Humidity calibration (0xE1–0xE7)
        cal2 = self._bus.read_i2c_block_data(self.address, self.REG_CALIB_26, 7)

        self._calib = {
            'T1': struct.unpack_from('<H', bytes(cal1), 0)[0],
            'T2': struct.unpack_from('<h', bytes(cal1), 2)[0],
            'T3': struct.unpack_from('<h', bytes(cal1), 4)[0],
            'P1': struct.unpack_from('<H', bytes(cal1), 6)[0],
            'P2': struct.unpack_from('<h', bytes(cal1), 8)[0],
            'P3': struct.unpack_from('<h', bytes(cal1), 10)[0],
            'P4': struct.unpack_from('<h', bytes(cal1), 12)[0],
            'P5': struct.unpack_from('<h', bytes(cal1), 14)[0],
            'P6': struct.unpack_from('<h', bytes(cal1), 16)[0],
            'P7': struct.unpack_from('<h', bytes(cal1), 18)[0],
            'P8': struct.unpack_from('<h', bytes(cal1), 20)[0],
            'P9': struct.unpack_from('<h', bytes(cal1), 22)[0],
            'H1': cal1[25],
            'H2': struct.unpack_from('<h', bytes(cal2), 0)[0],
            'H3': cal2[2],
            'H4': (cal2[3] << 4) | (cal2[4] & 0x0F),
            'H5': (cal2[5] << 4) | ((cal2[4] >> 4) & 0x0F),
            'H6': struct.unpack_from('<b', bytes(cal2), 6)[0],
        }

    def _read_calibration_sim(self):
        """Default calibration for simulation mode."""
        self._calib = {
            'T1': 27504, 'T2': 26435, 'T3': -1000,
            'P1': 36477, 'P2': -10685, 'P3': 3024,
            'P4': 2855, 'P5': 140, 'P6': -7,
            'P7': 15500, 'P8': -14600, 'P9': 6000,
            'H1': 75, 'H2': 370, 'H3': 0,
            'H4': 313, 'H5': 50, 'H6': 30,
        }

    def _read_raw(self) -> bytes:
        """Read 8 bytes of raw sensor data."""
        if self._sim_mode:
            return self._simulate_raw()

        # Wait for measurement to complete
        for _ in range(20):
            status = self._bus.read_byte_data(self.address, self.REG_STATUS)
            if not (status & 0x08):
                break
            time.sleep(0.002)

        return bytes(self._bus.read_i2c_block_data(self.address, self.REG_DATA, 8))

    def _simulate_raw(self) -> bytes:
        """Generate realistic simulated sensor data."""
        import random
        self._sim_temp += random.gauss(0, 0.05)
        self._sim_hum = max(10, min(95, self._sim_hum + random.gauss(0, 0.2)))
        self._sim_pres += random.gauss(0, 0.05)
        # Pack as fake raw bytes (we'll bypass compensation in sim)
        return struct.pack('>fff', self._sim_temp, self._sim_hum, self._sim_pres) + b'\x00\x00'

    def _parse(self, raw: bytes) -> SensorReading:
        """Parse raw sensor data into calibrated readings."""
        if self._sim_mode:
            temp, hum, pres = struct.unpack_from('>fff', raw, 0)
            return SensorReading(
                sensor_id=self.sensor_id,
                sensor_type=self.sensor_type,
                values={
                    "temperature": round(temp, 2),
                    "humidity": round(hum, 2),
                    "pressure": round(pres, 2),
                },
                unit="°C/%RH/hPa",
                quality=0.95,
                raw=raw,
            )

        # Unpack raw ADC values
        pres_raw = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
        temp_raw = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)
        hum_raw = (raw[6] << 8) | raw[7]

        # Compensate temperature
        temp = self._compensate_temperature(temp_raw)
        pres = self._compensate_pressure(pres_raw)
        hum = self._compensate_humidity(hum_raw)

        return SensorReading(
            sensor_id=self.sensor_id,
            sensor_type=self.sensor_type,
            values={
                "temperature": round(temp, 2),
                "humidity": round(max(0, min(100, hum)), 2),
                "pressure": round(pres, 2),
            },
            unit="°C/%RH/hPa",
            quality=1.0 if 0 < temp < 65 and 0 < hum < 100 else 0.5,
            raw=raw,
        )

    def _compensate_temperature(self, adc: int) -> float:
        """BME280 temperature compensation formula."""
        c = self._calib
        var1 = (adc / 16384.0 - c['T1'] / 1024.0) * c['T2']
        var2 = ((adc / 131072.0 - c['T1'] / 8192.0) ** 2) * c['T3']
        self._t_fine = var1 + var2
        return self._t_fine / 5120.0

    def _compensate_pressure(self, adc: int) -> float:
        """BME280 pressure compensation formula."""
        c = self._calib
        var1 = self._t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * c['P6'] / 32768.0
        var2 = var2 + var1 * c['P5'] * 2.0
        var2 = var2 / 4.0 + c['P4'] * 65536.0
        var1 = (c['P3'] * var1 * var1 / 524288.0 + c['P2'] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * c['P1']
        if var1 == 0:
            return 0
        p = 1048576.0 - adc
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = c['P9'] * p * p / 2147483648.0
        var2 = p * c['P8'] / 32768.0
        return (p + (var1 + var2 + c['P7']) / 16.0) / 100.0

    def _compensate_humidity(self, adc: int) -> float:
        """BME280 humidity compensation formula."""
        c = self._calib
        h = self._t_fine - 76800.0
        if h == 0:
            return 0
        h = (adc - (c['H4'] * 64.0 + c['H5'] / 16384.0 * h)) * \
            (c['H2'] / 65536.0 * (1.0 + c['H6'] / 67108864.0 * h *
             (1.0 + c['H3'] / 67108864.0 * h)))
        h = h * (1.0 - c['H1'] * h / 524288.0)
        return max(0.0, min(100.0, h))
