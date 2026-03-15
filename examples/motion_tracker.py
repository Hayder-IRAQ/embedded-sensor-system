#!/usr/bin/env python3
"""
Example: IMU Motion Tracking with Kalman Fusion
==================================================
Reads MPU6050 accelerometer/gyroscope data, applies 2D Kalman
filtering for position estimation, and detects sudden motion events.

Usage:
    python examples/motion_tracker.py
"""

import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.drivers.mpu6050 import MPU6050
from src.core.kalman_filter import KalmanFilter, KalmanFilter2D
from src.core.processing import AnomalyDetector, RingBuffer


def main():
    print("=" * 60)
    print("  IMU Motion Tracker — MPU6050 + Kalman Filter")
    print("=" * 60)

    imu = MPU6050(accel_range='4g', gyro_range='500dps')
    imu.initialize()
    print(f"  [✓] {imu.sensor_id} initialized")

    # Kalman filters for pitch and roll
    kf_pitch = KalmanFilter(process_noise=0.1, measurement_noise=2.0, adaptive=True)
    kf_roll = KalmanFilter(process_noise=0.1, measurement_noise=2.0, adaptive=True)

    # 2D Kalman for position tracking (from accel integration)
    kf_pos_x = KalmanFilter2D(dt=0.01, process_noise=0.5, measurement_noise=2.0)
    kf_pos_y = KalmanFilter2D(dt=0.01, process_noise=0.5, measurement_noise=2.0)

    # Motion anomaly detector
    motion_detector = AnomalyDetector(method='rate', rate_limit=50.0)

    # History buffers
    pitch_buf = RingBuffer(200)
    accel_mag_buf = RingBuffer(200)

    print(f"\n{'─' * 70}")
    print(f"  {'#':>4}  {'Pitch':>7}  {'Roll':>7}  {'|A|':>6}  {'Gx':>7}  {'Gy':>7}  {'Gz':>7}  {'Event':>8}")
    print(f"{'─' * 70}")

    for i in range(50):
        reading = imu.read()
        v = reading.values

        # Kalman-filtered angles
        pitch = kf_pitch.update(v["pitch"])
        roll = kf_roll.update(v["roll"])

        # Acceleration magnitude
        accel_mag = math.sqrt(v["accel_x"]**2 + v["accel_y"]**2 + v["accel_z"]**2)

        # Position estimation
        px, vx = kf_pos_x.update(v["accel_x"] * 9.81)
        py, vy = kf_pos_y.update(v["accel_y"] * 9.81)

        # Motion event detection
        event = motion_detector.check(accel_mag)
        event_str = event.severity.upper() if event else ""

        pitch_buf.push(pitch)
        accel_mag_buf.push(accel_mag)

        print(f"  {i:>4}  {pitch:>7.2f}  {roll:>7.2f}  {accel_mag:>6.3f}  "
              f"{v['gyro_x']:>7.2f}  {v['gyro_y']:>7.2f}  {v['gyro_z']:>7.2f}  {event_str:>8}")

        time.sleep(0.02)

    print(f"\n{'═' * 70}")
    print(f"  Pitch — Mean: {pitch_buf.mean:.2f}°, Std: {pitch_buf.std:.3f}°")
    print(f"  Accel magnitude — Mean: {accel_mag_buf.mean:.4f}g, Std: {accel_mag_buf.std:.4f}g")
    print(f"  Motion events: {motion_detector.anomaly_count}")
    print(f"  Estimated position: ({kf_pos_x.position:.3f}m, {kf_pos_y.position:.3f}m)")
    print(f"{'═' * 70}")


if __name__ == '__main__':
    main()
