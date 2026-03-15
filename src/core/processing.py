"""
Core Processing Modules
=========================
Anomaly detection, PID control, and ring buffer data structures.
"""

import time
import math
import numpy as np
from typing import List, Optional, Tuple, Any
from dataclasses import dataclass
from collections import deque


# ════════════════════════════════════════════════
# Anomaly Detector
# ════════════════════════════════════════════════

@dataclass
class AnomalyEvent:
    """Detected anomaly with context."""
    timestamp: float
    value: float
    expected: float
    deviation: float
    severity: str  # 'warning', 'critical'
    method: str

    def to_dict(self):
        return {
            "timestamp": self.timestamp, "value": self.value,
            "expected": self.expected, "deviation": round(self.deviation, 3),
            "severity": self.severity, "method": self.method,
        }


class AnomalyDetector:
    """
    Real-time anomaly detection for sensor streams.

    Methods:
        - 'zscore': Standard deviation threshold
        - 'iqr': Interquartile range
        - 'rate': Rate-of-change threshold
        - 'combined': Union of all methods

    Parameters
    ----------
    method : str
        Detection method.
    threshold : float
        Sensitivity threshold (lower = more sensitive).
    window_size : int
        Sliding window size for statistics computation.
    """

    METHODS = ('zscore', 'iqr', 'rate', 'combined')

    def __init__(self, method: str = 'zscore', threshold: float = 3.0,
                 window_size: int = 100, rate_limit: float = 10.0):
        if method not in self.METHODS:
            raise ValueError(f"Unknown method '{method}'. Use: {self.METHODS}")
        self.method = method
        self.threshold = threshold
        self.window_size = window_size
        self.rate_limit = rate_limit
        self._window: deque = deque(maxlen=window_size)
        self._timestamps: deque = deque(maxlen=window_size)
        self._anomalies: List[AnomalyEvent] = []

    def check(self, value: float, timestamp: Optional[float] = None) -> Optional[AnomalyEvent]:
        """
        Check a new value for anomalies.

        Returns AnomalyEvent if anomaly detected, None otherwise.
        """
        ts = timestamp or time.time()
        self._window.append(value)
        self._timestamps.append(ts)

        if len(self._window) < 10:
            return None

        anomaly = None
        if self.method == 'zscore':
            anomaly = self._check_zscore(value, ts)
        elif self.method == 'iqr':
            anomaly = self._check_iqr(value, ts)
        elif self.method == 'rate':
            anomaly = self._check_rate(value, ts)
        elif self.method == 'combined':
            for check in [self._check_zscore, self._check_iqr, self._check_rate]:
                anomaly = check(value, ts)
                if anomaly:
                    break

        if anomaly:
            self._anomalies.append(anomaly)
        return anomaly

    def _check_zscore(self, value: float, ts: float) -> Optional[AnomalyEvent]:
        arr = np.array(self._window)
        mean, std = arr.mean(), arr.std()
        if std < 1e-10:
            return None
        z = abs(value - mean) / std
        if z > self.threshold:
            severity = 'critical' if z > self.threshold * 1.5 else 'warning'
            return AnomalyEvent(ts, value, mean, z, severity, 'zscore')
        return None

    def _check_iqr(self, value: float, ts: float) -> Optional[AnomalyEvent]:
        arr = np.array(self._window)
        q1, q3 = np.percentile(arr, [25, 75])
        iqr = q3 - q1
        lower, upper = q1 - self.threshold * iqr, q3 + self.threshold * iqr
        if value < lower or value > upper:
            deviation = min(abs(value - lower), abs(value - upper)) / max(iqr, 1e-6)
            severity = 'critical' if deviation > 2 else 'warning'
            return AnomalyEvent(ts, value, (q1 + q3) / 2, deviation, severity, 'iqr')
        return None

    def _check_rate(self, value: float, ts: float) -> Optional[AnomalyEvent]:
        if len(self._window) < 2:
            return None
        prev = self._window[-2]
        dt = ts - self._timestamps[-2] if len(self._timestamps) >= 2 else 1.0
        rate = abs(value - prev) / max(dt, 1e-6)
        if rate > self.rate_limit:
            return AnomalyEvent(ts, value, prev, rate, 'warning', 'rate')
        return None

    @property
    def anomaly_count(self) -> int:
        return len(self._anomalies)

    @property
    def recent_anomalies(self) -> List[AnomalyEvent]:
        return self._anomalies[-20:]

    def reset(self):
        self._window.clear()
        self._timestamps.clear()
        self._anomalies.clear()


# ════════════════════════════════════════════════
# PID Controller
# ════════════════════════════════════════════════

class PIDController:
    """
    Discrete PID controller with anti-windup and derivative filtering.

    Parameters
    ----------
    kp, ki, kd : float
        Proportional, integral, derivative gains.
    setpoint : float
        Target value.
    output_limits : tuple
        (min, max) output clamping.
    sample_time : float
        Minimum time between updates (seconds).
    """

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 setpoint: float = 0.0, output_limits: Tuple[float, float] = (-100, 100),
                 sample_time: float = 0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.output_min, self.output_max = output_limits
        self.sample_time = sample_time

        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._derivative_filter = 0.0
        self._alpha = 0.1  # Derivative low-pass filter coefficient

    def update(self, measured_value: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output.

        Parameters
        ----------
        measured_value : float
            Current process variable.
        dt : float or None
            Time step (auto-computed if None).
        """
        now = time.time()
        if dt is None:
            dt = (now - self._prev_time) if self._prev_time else self.sample_time
        dt = max(dt, 1e-6)

        error = self.setpoint - measured_value

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative with low-pass filter
        raw_derivative = (error - self._prev_error) / dt
        self._derivative_filter = (self._alpha * raw_derivative +
                                    (1 - self._alpha) * self._derivative_filter)
        d_term = self.kd * self._derivative_filter

        # Sum and clamp
        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))

        # Anti-windup: back-calculate integral if saturated
        if output == self.output_max or output == self.output_min:
            self._integral -= error * dt * 0.5

        self._prev_error = error
        self._prev_time = now

        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None
        self._derivative_filter = 0.0


# ════════════════════════════════════════════════
# Ring Buffer
# ════════════════════════════════════════════════

class RingBuffer:
    """
    Fixed-size circular buffer for time-series data.

    Thread-safe for single-producer/single-consumer patterns.
    Zero allocation after initialization.
    """

    def __init__(self, capacity: int, dtype: type = float):
        self._capacity = capacity
        self._buffer = np.zeros(capacity, dtype=dtype)
        self._timestamps = np.zeros(capacity, dtype=float)
        self._head = 0
        self._count = 0

    def push(self, value, timestamp: Optional[float] = None):
        """Add a value to the buffer. Overwrites oldest if full."""
        self._buffer[self._head] = value
        self._timestamps[self._head] = timestamp or time.time()
        self._head = (self._head + 1) % self._capacity
        self._count = min(self._count + 1, self._capacity)

    def get_all(self) -> np.ndarray:
        """Return all values in chronological order."""
        if self._count < self._capacity:
            return self._buffer[:self._count].copy()
        idx = np.roll(np.arange(self._capacity), -self._head)
        return self._buffer[idx]

    def get_latest(self, n: int = 1) -> np.ndarray:
        """Return the n most recent values."""
        n = min(n, self._count)
        if n == 0:
            return np.array([])
        indices = [(self._head - 1 - i) % self._capacity for i in range(n)]
        return self._buffer[indices[::-1]]

    @property
    def mean(self) -> float:
        return float(np.mean(self._buffer[:self._count])) if self._count > 0 else 0.0

    @property
    def std(self) -> float:
        return float(np.std(self._buffer[:self._count])) if self._count > 1 else 0.0

    @property
    def is_full(self) -> bool:
        return self._count >= self._capacity

    def __len__(self) -> int:
        return self._count

    def __repr__(self):
        return f"RingBuffer(capacity={self._capacity}, count={self._count})"
