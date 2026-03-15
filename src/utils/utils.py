"""
Utilities — Fixed-Point Math, CRC, Bitfields, Retry Logic
============================================================
Embedded-friendly utilities for resource-constrained platforms.
"""

import time
import struct
import functools
from typing import Any, Callable, Optional


# ════════════════════════════════════════════════
# Fixed-Point Arithmetic (No FPU Required)
# ════════════════════════════════════════════════

class FixedPoint:
    """
    Q-format fixed-point number for FPU-less microcontrollers.

    Parameters
    ----------
    value : float or int
        Initial value.
    frac_bits : int
        Number of fractional bits (Q16 = 16 fractional bits).
    """

    def __init__(self, value: float = 0.0, frac_bits: int = 16):
        self.frac_bits = frac_bits
        self._scale = 1 << frac_bits
        self._raw = int(round(value * self._scale))

    @classmethod
    def from_raw(cls, raw: int, frac_bits: int = 16):
        obj = cls(0, frac_bits)
        obj._raw = raw
        return obj

    @property
    def value(self) -> float:
        return self._raw / self._scale

    @property
    def raw(self) -> int:
        return self._raw

    def __add__(self, other):
        if isinstance(other, FixedPoint):
            return FixedPoint.from_raw(self._raw + other._raw, self.frac_bits)
        return FixedPoint(self.value + float(other), self.frac_bits)

    def __sub__(self, other):
        if isinstance(other, FixedPoint):
            return FixedPoint.from_raw(self._raw - other._raw, self.frac_bits)
        return FixedPoint(self.value - float(other), self.frac_bits)

    def __mul__(self, other):
        if isinstance(other, FixedPoint):
            raw = (self._raw * other._raw) >> self.frac_bits
            return FixedPoint.from_raw(raw, self.frac_bits)
        return FixedPoint(self.value * float(other), self.frac_bits)

    def __truediv__(self, other):
        if isinstance(other, FixedPoint):
            raw = (self._raw << self.frac_bits) // other._raw
            return FixedPoint.from_raw(raw, self.frac_bits)
        return FixedPoint(self.value / float(other), self.frac_bits)

    def __repr__(self):
        return f"Q{self.frac_bits}({self.value:.6f})"

    def __float__(self):
        return self.value

    def __eq__(self, other):
        if isinstance(other, FixedPoint):
            return self._raw == other._raw
        return abs(self.value - float(other)) < (1 / self._scale)


# ════════════════════════════════════════════════
# CRC Checksums
# ════════════════════════════════════════════════

class CRC:
    """CRC-8, CRC-16, and CRC-32 checksum computation."""

    @staticmethod
    def crc8(data: bytes, poly: int = 0x31, init: int = 0xFF) -> int:
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ poly) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    @staticmethod
    def crc16(data: bytes, poly: int = 0xA001, init: int = 0xFFFF) -> int:
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def crc32(data: bytes) -> int:
        crc = 0xFFFFFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xEDB88320
                else:
                    crc >>= 1
        return crc ^ 0xFFFFFFFF

    @staticmethod
    def verify(data: bytes, expected: int, method: str = 'crc16') -> bool:
        func = getattr(CRC, method)
        return func(data) == expected


# ════════════════════════════════════════════════
# Bitfield — Register Manipulation
# ════════════════════════════════════════════════

class BitField:
    """
    Bit-level manipulation for hardware register access.

    Example
    -------
    >>> reg = BitField(0x3F)
    >>> reg.get_bits(2, 4)  # Get bits [5:2]
    15
    >>> reg.set_bits(2, 4, 0x0A)
    """

    def __init__(self, value: int = 0, width: int = 8):
        self.value = value & ((1 << width) - 1)
        self.width = width

    def get_bit(self, pos: int) -> bool:
        return bool(self.value & (1 << pos))

    def set_bit(self, pos: int, val: bool = True):
        if val:
            self.value |= (1 << pos)
        else:
            self.value &= ~(1 << pos)

    def get_bits(self, start: int, length: int) -> int:
        mask = (1 << length) - 1
        return (self.value >> start) & mask

    def set_bits(self, start: int, length: int, val: int):
        mask = ((1 << length) - 1) << start
        self.value = (self.value & ~mask) | ((val << start) & mask)

    def __int__(self):
        return self.value

    def __repr__(self):
        return f"BitField(0x{self.value:0{self.width // 4}X}, 0b{self.value:0{self.width}b})"


# ════════════════════════════════════════════════
# Retry with Exponential Backoff
# ════════════════════════════════════════════════

def retry(max_attempts: int = 3, base_delay: float = 0.1,
          max_delay: float = 5.0, exceptions: tuple = (Exception,)):
    """
    Decorator: retry a function with exponential backoff.

    Parameters
    ----------
    max_attempts : int
        Maximum number of retry attempts.
    base_delay : float
        Initial delay between retries (seconds).
    max_delay : float
        Maximum delay cap.
    exceptions : tuple
        Exception types to catch and retry.
    """
    def decorator(func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            last_exception = None
            for attempt in range(max_attempts):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt < max_attempts - 1:
                        delay = min(base_delay * (2 ** attempt), max_delay)
                        time.sleep(delay)
            raise last_exception
        return wrapper
    return decorator
