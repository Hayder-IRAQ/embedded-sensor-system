#!/usr/bin/env python3
"""
Example: Power Monitoring with PID Control
=============================================
Monitors voltage/current with INA219, uses PID controller
to regulate a simulated load, and logs anomalies.

Usage:
    python examples/power_monitor.py
"""

import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.drivers.additional_sensors import INA219
from src.core.kalman_filter import KalmanFilter
from src.core.processing import PIDController, AnomalyDetector, RingBuffer


def main():
    print("=" * 60)
    print("  Power Monitor — INA219 + PID Control")
    print("=" * 60)

    ina = INA219(shunt_ohms=0.1, sensor_id="power_rail_5v")
    ina.initialize()

    kf_voltage = KalmanFilter(process_noise=0.001, measurement_noise=0.05)
    kf_current = KalmanFilter(process_noise=0.005, measurement_noise=0.02)

    # PID to regulate voltage at 5.0V
    pid = PIDController(kp=2.0, ki=0.5, kd=0.1, setpoint=5.0,
                         output_limits=(-1.0, 1.0))

    # Anomaly detection on power consumption
    power_anomaly = AnomalyDetector(method='combined', threshold=2.5)

    power_buf = RingBuffer(100)

    print(f"\n{'─' * 65}")
    print(f"  {'#':>4}  {'V_raw':>7}  {'V_filt':>7}  {'I(mA)':>7}  {'P(mW)':>8}  {'PID':>6}  {'Alert':>8}")
    print(f"{'─' * 65}")

    for i in range(40):
        reading = ina.read()
        v_raw = reading.values["voltage"]
        i_raw = reading.values["current_a"]

        v_filt = kf_voltage.update(v_raw)
        i_filt = kf_current.update(i_raw)
        power_mw = v_filt * abs(i_filt) * 1000

        pid_output = pid.update(v_filt, dt=0.1)

        event = power_anomaly.check(power_mw)
        alert = event.severity if event else ""

        power_buf.push(power_mw)

        print(f"  {i:>4}  {v_raw:>7.3f}  {v_filt:>7.3f}  {i_filt*1000:>7.2f}  "
              f"{power_mw:>8.2f}  {pid_output:>6.3f}  {alert:>8}")

        time.sleep(0.05)

    print(f"\n{'═' * 65}")
    print(f"  Power — Mean: {power_buf.mean:.2f} mW, Std: {power_buf.std:.3f} mW")
    print(f"  Anomalies: {power_anomaly.anomaly_count}")
    print(f"{'═' * 65}")


if __name__ == '__main__':
    main()
