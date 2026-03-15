"""
Kalman Filter — Adaptive Noise Estimation
============================================
1D and multi-dimensional Kalman filter for real-time sensor data smoothing.
Supports adaptive process/measurement noise estimation.
"""

import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass, field


@dataclass
class KalmanState:
    """Kalman filter internal state snapshot."""
    x: float          # State estimate
    P: float          # Error covariance
    K: float          # Kalman gain
    innovation: float # Measurement residual
    step: int = 0


class KalmanFilter:
    """
    1D Kalman filter with optional adaptive noise estimation.

    Parameters
    ----------
    process_noise : float
        Process noise covariance Q (how much the system changes between steps).
    measurement_noise : float
        Measurement noise covariance R (sensor noise level).
    initial_estimate : float
        Initial state estimate x₀.
    initial_covariance : float
        Initial error covariance P₀.
    adaptive : bool
        Enable adaptive measurement noise estimation.
    """

    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1,
                 initial_estimate: float = 0.0, initial_covariance: float = 1.0,
                 adaptive: bool = False):
        self.Q = process_noise
        self.R = measurement_noise
        self.x = initial_estimate
        self.P = initial_covariance
        self.K = 0.0
        self.adaptive = adaptive
        self._step = 0
        self._innovation_history = []
        self._window = 50

    def predict(self, control_input: float = 0.0, B: float = 0.0) -> float:
        """
        Predict step: project state ahead.
        x̂ₖ⁻ = x̂ₖ₋₁ + B·uₖ
        Pₖ⁻ = Pₖ₋₁ + Q
        """
        self.x = self.x + B * control_input
        self.P = self.P + self.Q
        return self.x

    def update(self, measurement: float) -> float:
        """
        Update step: correct prediction with measurement.

        Parameters
        ----------
        measurement : float
            New sensor measurement.

        Returns
        -------
        float — Filtered (corrected) state estimate.
        """
        # Predict
        self.predict()

        # Innovation (measurement residual)
        innovation = measurement - self.x

        # Adaptive noise estimation
        if self.adaptive:
            self._innovation_history.append(innovation)
            if len(self._innovation_history) > self._window:
                self._innovation_history.pop(0)
            if len(self._innovation_history) >= 10:
                var = np.var(self._innovation_history)
                self.R = max(0.001, 0.7 * self.R + 0.3 * var)

        # Kalman gain
        S = self.P + self.R  # Innovation covariance
        self.K = self.P / S if S > 0 else 0.0

        # Update estimate
        self.x = self.x + self.K * innovation
        self.P = (1 - self.K) * self.P

        self._step += 1

        return self.x

    @property
    def state(self) -> KalmanState:
        return KalmanState(
            x=self.x, P=self.P, K=self.K,
            innovation=self._innovation_history[-1] if self._innovation_history else 0.0,
            step=self._step,
        )

    def reset(self, initial_estimate: float = 0.0, initial_covariance: float = 1.0):
        """Reset filter to initial conditions."""
        self.x = initial_estimate
        self.P = initial_covariance
        self.K = 0.0
        self._step = 0
        self._innovation_history.clear()

    def batch_filter(self, measurements: list) -> list:
        """Process a batch of measurements and return all filtered values."""
        return [self.update(m) for m in measurements]


class KalmanFilter2D:
    """
    2D Kalman filter for position + velocity tracking.

    State: [position, velocity]
    Useful for GPS smoothing, IMU dead-reckoning, etc.
    """

    def __init__(self, dt: float = 0.1,
                 process_noise: float = 0.1,
                 measurement_noise: float = 1.0):
        self.dt = dt
        # State transition matrix
        self.F = np.array([[1, dt], [0, 1]], dtype=float)
        # Measurement matrix (we observe position only)
        self.H = np.array([[1, 0]], dtype=float)
        # Process noise
        self.Q = process_noise * np.array([
            [dt**4 / 4, dt**3 / 2],
            [dt**3 / 2, dt**2]
        ], dtype=float)
        # Measurement noise
        self.R = np.array([[measurement_noise]], dtype=float)
        # State and covariance
        self.x = np.array([[0], [0]], dtype=float)
        self.P = np.eye(2) * 100

    def update(self, measurement: float) -> Tuple[float, float]:
        """
        Run predict + update cycle.

        Returns
        -------
        (position, velocity) estimates.
        """
        # Predict
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # Update
        z = np.array([[measurement]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def position(self) -> float:
        return float(self.x[0, 0])

    @property
    def velocity(self) -> float:
        return float(self.x[1, 0])
