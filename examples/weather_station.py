#!/usr/bin/env python3
"""
Example: Weather Station with Multi-Sensor Fusion
===================================================
Reads environmental data from BME280 + DHT22, applies Kalman filtering,
detects anomalies, logs to SQLite, and publishes via MQTT.

Usage:
    python examples/weather_station.py
"""

import sys, os, time, asyncio
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.drivers.bme280 import BME280
from src.drivers.additional_sensors import DHT22, BH1750
from src.core.kalman_filter import KalmanFilter
from src.core.sensor_fusion import SensorFusion
from src.core.processing import AnomalyDetector, RingBuffer
from src.middleware.middleware import DataLogger, EventBus, Watchdog


def main():
    print("=" * 60)
    print("  Weather Station — Multi-Sensor Fusion Demo")
    print("=" * 60)

    # Initialize sensors
    bme = BME280(sensor_id="indoor_bme280")
    dht = DHT22(sensor_id="outdoor_dht22")
    light = BH1750(sensor_id="ambient_light")

    for s in [bme, dht, light]:
        s.initialize()
        print(f"  [✓] {s.sensor_id} initialized ({s.sensor_type})")

    # Kalman filters for each measurement
    kf_temp = KalmanFilter(process_noise=0.01, measurement_noise=0.5, adaptive=True)
    kf_hum = KalmanFilter(process_noise=0.02, measurement_noise=1.0)

    # Sensor fusion for temperature
    fusion = SensorFusion(strategy='adaptive')
    fusion.add_source("bme280", weight=0.7, sensor=bme)
    fusion.add_source("dht22", weight=0.3, sensor=dht)

    # Anomaly detection
    anomaly = AnomalyDetector(method='zscore', threshold=3.0, window_size=50)

    # Data logging
    logger = DataLogger(path="./data", format="sqlite")

    # Event bus
    bus = EventBus()
    bus.on("anomaly", lambda t, d: print(f"  ⚠ ANOMALY: {d}"))
    bus.on("reading", lambda t, d: None)  # Could forward to MQTT

    # Watchdog
    wd = Watchdog(timeout_seconds=10.0)
    wd.register("bme280")
    wd.register("dht22")

    # Ring buffer for history
    temp_buffer = RingBuffer(capacity=100)

    print(f"\n{'─' * 60}")
    print(f"  {'Time':>8}  {'Raw °C':>8}  {'Filtered':>8}  {'Fused':>8}  {'Humidity':>8}  {'Lux':>8}")
    print(f"{'─' * 60}")

    # Main loop
    for i in range(30):
        # Read sensors
        bme_reading = bme.read()
        dht_reading = dht.read()
        light_reading = light.read()

        # Kalman filtering
        raw_temp = bme_reading.values["temperature"]
        filtered_temp = kf_temp.update(raw_temp)
        filtered_hum = kf_hum.update(bme_reading.values["humidity"])

        # Sensor fusion
        fused = fusion.read("temperature")

        # Anomaly detection
        anomaly_event = anomaly.check(raw_temp)
        if anomaly_event:
            bus.emit("anomaly", anomaly_event.to_dict())

        # Log data
        logger.log(bme_reading)
        logger.log(dht_reading)

        # Watchdog heartbeat
        wd.heartbeat("bme280")
        wd.heartbeat("dht22")

        # Buffer
        temp_buffer.push(filtered_temp)

        # Display
        lux = light_reading.values["illuminance"]
        print(f"  {i * 2:>5}s   {raw_temp:>7.2f}  {filtered_temp:>8.2f}  "
              f"{fused.value:>8.2f}  {filtered_hum:>7.1f}%  {lux:>7.1f}")

        time.sleep(0.05)

    # Summary
    print(f"\n{'═' * 60}")
    print(f"  Summary:")
    print(f"  Temperature — Mean: {temp_buffer.mean:.2f}°C, Std: {temp_buffer.std:.3f}°C")
    print(f"  Kalman gain: {kf_temp.K:.4f}")
    print(f"  Anomalies detected: {anomaly.anomaly_count}")
    print(f"  Data points logged: {logger.stats['entries']}")
    print(f"  Watchdog status: {wd.status}")
    print(f"{'═' * 60}")

    logger.close()


if __name__ == '__main__':
    main()
