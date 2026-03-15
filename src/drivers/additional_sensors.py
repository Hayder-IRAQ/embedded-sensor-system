"""
Additional Sensor Drivers
==========================
Lightweight drivers for common IoT sensors.
All support hardware + simulation modes.
"""

import struct
import time
import math
import random
from typing import Optional
from .base_sensor import BaseSensor, SensorReading


# ──────────────────────────────────────────────
# ADS1115 — 16-bit ADC (4 channels)
# ──────────────────────────────────────────────
class ADS1115(BaseSensor):
    """Texas Instruments ADS1115 16-bit ADC. I²C, 4 channels, PGA."""

    GAIN_MAP = {
        '6.144V': (0x0000, 6.144), '4.096V': (0x0200, 4.096),
        '2.048V': (0x0400, 2.048), '1.024V': (0x0600, 1.024),
        '0.512V': (0x0800, 0.512), '0.256V': (0x0A00, 0.256),
    }

    def __init__(self, bus_id=1, address=0x48, gain='4.096V', channel=0, sensor_id=None):
        super().__init__(sensor_id=sensor_id or f"ads1115_ch{channel}", sensor_type="adc",
                         bus_id=bus_id, address=address)
        self.channel = channel
        self._gain_reg, self._gain_volts = self.GAIN_MAP.get(gain, self.GAIN_MAP['4.096V'])
        self._sim_mode = True

    def _init_hardware(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus_id)
            self._sim_mode = False
        except (ImportError, OSError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            voltage = 1.65 + random.gauss(0, 0.01)
            raw_val = int(voltage / self._gain_volts * 32767)
            return struct.pack('>h', raw_val)
        mux = (0x04 + self.channel) << 12
        config = 0x8000 | mux | self._gain_reg | 0x0100 | 0x0003
        self._bus.write_i2c_block_data(self.address, 0x01, [(config >> 8) & 0xFF, config & 0xFF])
        time.sleep(0.01)
        result = self._bus.read_i2c_block_data(self.address, 0x00, 2)
        return bytes(result)

    def _parse(self, raw: bytes) -> SensorReading:
        adc_val = struct.unpack('>h', raw[:2])[0]
        voltage = adc_val * self._gain_volts / 32767.0
        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"voltage": round(voltage, 6), "raw_adc": float(adc_val)},
            unit="V", quality=1.0, raw=raw,
        )


# ──────────────────────────────────────────────
# GPS NEO-6M — UART GPS Receiver
# ──────────────────────────────────────────────
class GPSNEO6M(BaseSensor):
    """u-blox NEO-6M GPS module. NMEA sentence parsing."""

    def __init__(self, port='/dev/ttyAMA0', baudrate=9600, sensor_id=None):
        super().__init__(sensor_id=sensor_id or "gps_neo6m", sensor_type="gps",
                         bus_id=0, address=0)
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._sim_mode = True
        self._sim_lat, self._sim_lon = 33.3128, 44.3615  # Baghdad

    def _init_hardware(self):
        try:
            import serial
            self._serial = serial.Serial(self.port, self.baudrate, timeout=2)
            self._sim_mode = False
        except (ImportError, OSError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            self._sim_lat += random.gauss(0, 0.00001)
            self._sim_lon += random.gauss(0, 0.00001)
            alt = 34.0 + random.gauss(0, 0.5)
            speed = max(0, 2.5 + random.gauss(0, 0.3))
            nmea = f"$SIMGGA,{self._sim_lat:.6f},{self._sim_lon:.6f},{alt:.1f},{speed:.1f}"
            return nmea.encode()
        line = self._serial.readline()
        return line

    def _parse(self, raw: bytes) -> SensorReading:
        sentence = raw.decode('ascii', errors='ignore').strip()
        if self._sim_mode and sentence.startswith("$SIMGGA"):
            parts = sentence.split(',')
            lat, lon, alt, speed = float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])
        else:
            lat, lon, alt, speed = self._parse_nmea(sentence)

        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"latitude": lat, "longitude": lon, "altitude": alt, "speed_kmh": speed},
            unit="°/m/km/h",
            quality=1.0 if lat != 0 and lon != 0 else 0.0,
            raw=raw,
        )

    @staticmethod
    def _parse_nmea(sentence: str):
        if sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA'):
            parts = sentence.split(',')
            try:
                lat_raw = float(parts[2]) if parts[2] else 0
                lat = int(lat_raw / 100) + (lat_raw % 100) / 60
                if parts[3] == 'S': lat = -lat
                lon_raw = float(parts[4]) if parts[4] else 0
                lon = int(lon_raw / 100) + (lon_raw % 100) / 60
                if parts[5] == 'W': lon = -lon
                alt = float(parts[9]) if parts[9] else 0
                return lat, lon, alt, 0.0
            except (ValueError, IndexError):
                pass
        return 0.0, 0.0, 0.0, 0.0


# ──────────────────────────────────────────────
# HC-SR04 — Ultrasonic Distance Sensor
# ──────────────────────────────────────────────
class HCSR04(BaseSensor):
    """HC-SR04 ultrasonic ranging. GPIO trigger/echo, 2–400 cm range."""

    def __init__(self, trigger_pin=23, echo_pin=24, sensor_id=None):
        super().__init__(sensor_id=sensor_id or "hcsr04", sensor_type="distance",
                         bus_id=0, address=0)
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self._sim_mode = True
        self._sim_dist = 50.0

    def _init_hardware(self):
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            self._sim_mode = False
        except (ImportError, RuntimeError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            self._sim_dist = max(2, min(400, self._sim_dist + random.gauss(0, 0.5)))
            return struct.pack('>f', self._sim_dist)
        import RPi.GPIO as GPIO
        GPIO.output(self.trigger_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, False)
        start = time.time()
        while GPIO.input(self.echo_pin) == 0:
            start = time.time()
            if time.time() - start > 0.04: break
        while GPIO.input(self.echo_pin) == 1:
            end = time.time()
            if time.time() - start > 0.04: break
        dist = (end - start) * 34300 / 2
        return struct.pack('>f', dist)

    def _parse(self, raw: bytes) -> SensorReading:
        dist = struct.unpack('>f', raw[:4])[0]
        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"distance_cm": round(dist, 1)},
            unit="cm", quality=1.0 if 2 < dist < 400 else 0.3, raw=raw,
        )


# ──────────────────────────────────────────────
# DHT22 — Temperature & Humidity (Digital)
# ──────────────────────────────────────────────
class DHT22(BaseSensor):
    """AOSONG DHT22/AM2302. Single-wire, ±0.5°C, ±2% RH."""

    def __init__(self, pin=4, sensor_id=None):
        super().__init__(sensor_id=sensor_id or "dht22", sensor_type="environmental",
                         bus_id=0, address=0)
        self.pin = pin
        self._sim_mode = True
        self._sim_temp, self._sim_hum = 24.0, 50.0

    def _init_hardware(self):
        self._sim_mode = True  # Always use sim unless adafruit lib available
        try:
            import adafruit_dht
            import board
            self._dht = adafruit_dht.DHT22(getattr(board, f'D{self.pin}'))
            self._sim_mode = False
        except (ImportError, NotImplementedError):
            pass

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            self._sim_temp += random.gauss(0, 0.1)
            self._sim_hum = max(20, min(90, self._sim_hum + random.gauss(0, 0.3)))
            return struct.pack('>ff', self._sim_temp, self._sim_hum)
        temp = self._dht.temperature
        hum = self._dht.humidity
        return struct.pack('>ff', temp or 0, hum or 0)

    def _parse(self, raw: bytes) -> SensorReading:
        temp, hum = struct.unpack('>ff', raw[:8])
        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"temperature": round(temp, 1), "humidity": round(hum, 1)},
            unit="°C/%RH", quality=1.0 if -40 < temp < 80 else 0.5, raw=raw,
        )


# ──────────────────────────────────────────────
# BH1750 — Ambient Light Sensor
# ──────────────────────────────────────────────
class BH1750(BaseSensor):
    """ROHM BH1750FVI ambient light sensor. I²C, 1–65535 lux."""

    def __init__(self, bus_id=1, address=0x23, sensor_id=None):
        super().__init__(sensor_id=sensor_id or "bh1750", sensor_type="light",
                         bus_id=bus_id, address=address)
        self._sim_mode = True
        self._sim_lux = 350.0

    def _init_hardware(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus_id)
            self._bus.write_byte(self.address, 0x10)  # Continuous high-res mode
            self._sim_mode = False
            time.sleep(0.18)
        except (ImportError, OSError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            self._sim_lux = max(0, self._sim_lux + random.gauss(0, 5))
            return struct.pack('>H', int(self._sim_lux * 1.2))
        data = self._bus.read_i2c_block_data(self.address, 0x10, 2)
        return bytes(data)

    def _parse(self, raw: bytes) -> SensorReading:
        raw_val = struct.unpack('>H', raw[:2])[0]
        lux = raw_val / 1.2
        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"illuminance": round(lux, 1)},
            unit="lux", quality=1.0, raw=raw,
        )


# ──────────────────────────────────────────────
# INA219 — Current/Power Monitor
# ──────────────────────────────────────────────
class INA219(BaseSensor):
    """Texas Instruments INA219 current & power monitor. I²C, ±3.2A, 0–26V."""

    def __init__(self, bus_id=1, address=0x40, shunt_ohms=0.1, sensor_id=None):
        super().__init__(sensor_id=sensor_id or "ina219", sensor_type="power",
                         bus_id=bus_id, address=address)
        self.shunt_ohms = shunt_ohms
        self._sim_mode = True

    def _init_hardware(self):
        try:
            import smbus2
            self._bus = smbus2.SMBus(self.bus_id)
            self._bus.write_i2c_block_data(self.address, 0x05, [0x10, 0x00])
            self._bus.write_i2c_block_data(self.address, 0x00, [0x39, 0x9F])
            self._sim_mode = False
        except (ImportError, OSError):
            self._sim_mode = True

    def _read_raw(self) -> bytes:
        if self._sim_mode:
            voltage = 5.0 + random.gauss(0, 0.02)
            current = 0.25 + random.gauss(0, 0.01)
            return struct.pack('>ff', voltage, current)
        shunt = self._bus.read_i2c_block_data(self.address, 0x01, 2)
        bus = self._bus.read_i2c_block_data(self.address, 0x02, 2)
        return bytes(shunt + bus)

    def _parse(self, raw: bytes) -> SensorReading:
        if self._sim_mode:
            voltage, current = struct.unpack('>ff', raw[:8])
        else:
            shunt_raw = struct.unpack('>h', raw[0:2])[0]
            bus_raw = struct.unpack('>H', raw[2:4])[0]
            voltage = (bus_raw >> 3) * 0.004
            current = (shunt_raw * 0.01) / self.shunt_ohms / 1000
        power = voltage * abs(current)
        return SensorReading(
            sensor_id=self.sensor_id, sensor_type=self.sensor_type,
            values={"voltage": round(voltage, 3), "current_a": round(current, 4),
                    "power_w": round(power, 4)},
            unit="V/A/W", quality=1.0, raw=raw,
        )
