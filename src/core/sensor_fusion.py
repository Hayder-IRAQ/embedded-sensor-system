"""
Sensor Fusion — Multi-Source Data Integration
================================================
Fuse readings from multiple sensors measuring the same physical quantity.
Supports weighted average, Kalman-based, and voting strategies.
"""

import time
import numpy as np
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from ..drivers.base_sensor import BaseSensor, SensorReading


@dataclass
class FusedReading:
    """Result of sensor fusion with confidence scoring."""
    value: float
    confidence: float  # 0.0–1.0
    sources: Dict[str, float]  # source_id → individual value
    weights: Dict[str, float]  # source_id → effective weight
    strategy: str
    timestamp: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "value": round(self.value, 4),
            "confidence": round(self.confidence, 3),
            "sources": {k: round(v, 4) for k, v in self.sources.items()},
            "strategy": self.strategy,
            "timestamp": self.timestamp,
        }


class SensorFusion:
    """
    Multi-sensor data fusion engine.

    Strategies:
        - 'weighted_average': Static weighted average
        - 'adaptive': Dynamically adjusts weights based on sensor agreement
        - 'kalman': Fuses via sequential Kalman filter updates
        - 'voting': Majority-vote outlier rejection + average

    Example
    -------
    >>> fusion = SensorFusion(strategy='adaptive')
    >>> fusion.add_source('indoor_bme', weight=0.6, sensor=bme280)
    >>> fusion.add_source('outdoor_dht', weight=0.4, sensor=dht22)
    >>> result = fusion.read('temperature')
    >>> print(f"{result.value:.1f}°C (confidence: {result.confidence:.0%})")
    """

    STRATEGIES = ('weighted_average', 'adaptive', 'kalman', 'voting')

    def __init__(self, strategy: str = 'weighted_average'):
        if strategy not in self.STRATEGIES:
            raise ValueError(f"Unknown strategy '{strategy}'. Use one of {self.STRATEGIES}")
        self.strategy = strategy
        self._sources: Dict[str, dict] = {}
        self._history: Dict[str, List[float]] = {}
        self._history_size = 50

    def add_source(self, source_id: str, weight: float = 1.0,
                   sensor: Optional[BaseSensor] = None,
                   max_age_seconds: float = 10.0):
        """Register a sensor source for fusion."""
        self._sources[source_id] = {
            'weight': weight,
            'sensor': sensor,
            'max_age': max_age_seconds,
            'last_value': None,
            'last_time': 0,
            'error_count': 0,
        }
        self._history[source_id] = []

    def remove_source(self, source_id: str):
        self._sources.pop(source_id, None)
        self._history.pop(source_id, None)

    def read(self, measurement_key: str = 'temperature') -> FusedReading:
        """
        Read from all sources and fuse.

        Parameters
        ----------
        measurement_key : str
            Which measurement to fuse (e.g., 'temperature', 'humidity').
        """
        readings = self._collect_readings(measurement_key)

        if not readings:
            return FusedReading(value=0, confidence=0, sources={}, weights={},
                                strategy=self.strategy, timestamp=time.time())

        if self.strategy == 'weighted_average':
            return self._fuse_weighted(readings)
        elif self.strategy == 'adaptive':
            return self._fuse_adaptive(readings)
        elif self.strategy == 'kalman':
            return self._fuse_kalman(readings)
        elif self.strategy == 'voting':
            return self._fuse_voting(readings)

    def _collect_readings(self, key: str) -> Dict[str, float]:
        """Read from each sensor and collect values."""
        readings = {}
        now = time.time()

        for src_id, src in self._sources.items():
            try:
                if src['sensor']:
                    reading = src['sensor'].read()
                    if key in reading.values:
                        val = reading.values[key]
                        readings[src_id] = val
                        src['last_value'] = val
                        src['last_time'] = now

                        # Update history
                        self._history[src_id].append(val)
                        if len(self._history[src_id]) > self._history_size:
                            self._history[src_id].pop(0)

                elif src['last_value'] is not None and (now - src['last_time']) < src['max_age']:
                    readings[src_id] = src['last_value']

            except Exception:
                src['error_count'] += 1

        return readings

    def _fuse_weighted(self, readings: Dict[str, float]) -> FusedReading:
        """Static weighted average."""
        total_weight = 0
        weighted_sum = 0

        for src_id, val in readings.items():
            w = self._sources[src_id]['weight']
            weighted_sum += val * w
            total_weight += w

        fused = weighted_sum / total_weight if total_weight > 0 else 0
        weights = {sid: self._sources[sid]['weight'] for sid in readings}

        # Confidence based on agreement among sources
        values = list(readings.values())
        spread = np.std(values) if len(values) > 1 else 0
        confidence = max(0, 1.0 - spread / (abs(fused) + 1e-6))

        return FusedReading(value=fused, confidence=min(1.0, confidence),
                            sources=readings, weights=weights,
                            strategy='weighted_average', timestamp=time.time())

    def _fuse_adaptive(self, readings: Dict[str, float]) -> FusedReading:
        """Adaptive weighted average — sensors closer to consensus get higher weight."""
        values = list(readings.values())
        median_val = np.median(values)

        adaptive_weights = {}
        for src_id, val in readings.items():
            deviation = abs(val - median_val)
            base_weight = self._sources[src_id]['weight']
            # Reduce weight for outliers
            adaptive_w = base_weight / (1 + deviation ** 2)
            adaptive_weights[src_id] = adaptive_w

        total = sum(adaptive_weights.values())
        if total > 0:
            adaptive_weights = {k: v / total for k, v in adaptive_weights.items()}

        fused = sum(readings[sid] * w for sid, w in adaptive_weights.items())
        spread = np.std(values) if len(values) > 1 else 0
        confidence = max(0, 1.0 - spread / (abs(fused) + 1e-6))

        return FusedReading(value=fused, confidence=min(1.0, confidence),
                            sources=readings, weights=adaptive_weights,
                            strategy='adaptive', timestamp=time.time())

    def _fuse_kalman(self, readings: Dict[str, float]) -> FusedReading:
        """Sequential Kalman fusion — treat each source as a measurement."""
        from ..core.kalman_filter import KalmanFilter

        values = list(readings.values())
        kf = KalmanFilter(process_noise=0.001, measurement_noise=0.1,
                          initial_estimate=values[0])

        for val in values[1:]:
            kf.update(val)

        weights = {sid: 1.0 / len(readings) for sid in readings}
        confidence = max(0, 1.0 - kf.P)

        return FusedReading(value=kf.x, confidence=min(1.0, confidence),
                            sources=readings, weights=weights,
                            strategy='kalman', timestamp=time.time())

    def _fuse_voting(self, readings: Dict[str, float]) -> FusedReading:
        """Majority voting — reject outliers, average the rest."""
        values = list(readings.values())
        if len(values) < 3:
            return self._fuse_weighted(readings)

        median = np.median(values)
        mad = np.median([abs(v - median) for v in values])
        threshold = 3 * max(mad, 0.01)

        inliers = {sid: val for sid, val in readings.items()
                    if abs(val - median) <= threshold}

        if not inliers:
            inliers = readings

        fused = np.mean(list(inliers.values()))
        weights = {sid: (1.0 if sid in inliers else 0.0) for sid in readings}
        confidence = len(inliers) / len(readings)

        return FusedReading(value=fused, confidence=confidence,
                            sources=readings, weights=weights,
                            strategy='voting', timestamp=time.time())

    @property
    def source_health(self) -> Dict[str, Any]:
        return {
            sid: {
                'weight': src['weight'],
                'last_value': src['last_value'],
                'error_count': src['error_count'],
                'history_len': len(self._history.get(sid, [])),
            }
            for sid, src in self._sources.items()
        }
