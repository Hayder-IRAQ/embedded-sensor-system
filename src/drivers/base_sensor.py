"""
Base Sensor — Abstract Hardware Interface
==========================================
All sensor drivers inherit from this class to ensure a consistent API.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from datetime import datetime, timezone
import time
import json


@dataclass
class SensorReading:
    """Immutable sensor reading with metadata."""
    sensor_id: str
    sensor_type: str
    values: Dict[str, float]
    unit: str
    timestamp: float = field(default_factory=time.time)
    quality: float = 1.0  # 0.0 = bad, 1.0 = perfect
    raw: Optional[bytes] = None

    @property
    def primary_value(self) -> float:
        """Return the first/primary measurement value."""
        return next(iter(self.values.values()))

    @property
    def iso_timestamp(self) -> str:
        return datetime.fromtimestamp(self.timestamp, tz=timezone.utc).isoformat()

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sensor_id": self.sensor_id,
            "sensor_type": self.sensor_type,
            "values": self.values,
            "unit": self.unit,
            "timestamp": self.iso_timestamp,
            "quality": self.quality,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict())

    def __getattr__(self, name: str) -> float:
        if name in self.__dict__.get("values", {}):
            return self.values[name]
        raise AttributeError(f"No measurement '{name}' in {self.sensor_type}")


class BaseSensor(ABC):
    """
    Abstract base class for all sensor drivers.

    Subclasses must implement:
        - _read_raw() → read raw bytes from hardware
        - _parse(raw) → convert raw bytes to SensorReading
        - _init_hardware() → initialize the sensor hardware
    """

    def __init__(self, sensor_id: str, sensor_type: str, bus_id: int = 1,
                 address: int = 0x00, sample_rate_hz: float = 1.0):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.bus_id = bus_id
        self.address = address
        self.sample_rate_hz = sample_rate_hz
        self._initialized = False
        self._last_reading: Optional[SensorReading] = None
        self._read_count = 0
        self._error_count = 0
        self._calibration_offset: Dict[str, float] = {}

    def initialize(self) -> bool:
        """Initialize the sensor hardware. Returns True on success."""
        try:
            self._init_hardware()
            self._initialized = True
            return True
        except Exception as e:
            self._error_count += 1
            raise SensorInitError(f"Failed to initialize {self.sensor_id}: {e}")

    def read(self) -> SensorReading:
        """Read, parse, calibrate, and return a sensor reading."""
        if not self._initialized:
            self.initialize()

        try:
            raw = self._read_raw()
            reading = self._parse(raw)

            # Apply calibration offsets
            if self._calibration_offset:
                calibrated = {}
                for key, val in reading.values.items():
                    offset = self._calibration_offset.get(key, 0.0)
                    calibrated[key] = val + offset
                reading = SensorReading(
                    sensor_id=reading.sensor_id,
                    sensor_type=reading.sensor_type,
                    values=calibrated,
                    unit=reading.unit,
                    timestamp=reading.timestamp,
                    quality=reading.quality,
                    raw=reading.raw,
                )

            self._last_reading = reading
            self._read_count += 1
            return reading

        except Exception as e:
            self._error_count += 1
            raise SensorReadError(f"Read failed on {self.sensor_id}: {e}")

    def calibrate(self, offsets: Dict[str, float]):
        """Set calibration offsets for each measurement channel."""
        self._calibration_offset = offsets

    @property
    def health(self) -> Dict[str, Any]:
        """Return sensor health metrics."""
        total = self._read_count + self._error_count
        return {
            "sensor_id": self.sensor_id,
            "initialized": self._initialized,
            "read_count": self._read_count,
            "error_count": self._error_count,
            "error_rate": self._error_count / total if total > 0 else 0.0,
            "last_reading_age": (
                time.time() - self._last_reading.timestamp
                if self._last_reading else None
            ),
        }

    @abstractmethod
    def _init_hardware(self):
        """Initialize sensor registers and configuration."""
        ...

    @abstractmethod
    def _read_raw(self) -> bytes:
        """Read raw bytes from the sensor hardware."""
        ...

    @abstractmethod
    def _parse(self, raw: bytes) -> SensorReading:
        """Parse raw bytes into a SensorReading."""
        ...

    def __repr__(self):
        return f"{self.__class__.__name__}(id={self.sensor_id}, addr=0x{self.address:02X})"


class SensorInitError(Exception):
    """Raised when sensor initialization fails."""
    pass


class SensorReadError(Exception):
    """Raised when a sensor read operation fails."""
    pass
