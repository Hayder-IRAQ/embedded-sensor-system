"""
Test Suite — Embedded Sensor System
=====================================
Comprehensive unit and integration tests.
"""

import pytest
import numpy as np
import time
import sys
import os
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.drivers.base_sensor import SensorReading
from src.drivers.bme280 import BME280
from src.drivers.mpu6050 import MPU6050
from src.drivers.additional_sensors import ADS1115, GPSNEO6M, HCSR04, DHT22, BH1750, INA219
from src.core.kalman_filter import KalmanFilter, KalmanFilter2D
from src.core.sensor_fusion import SensorFusion
from src.core.processing import AnomalyDetector, PIDController, RingBuffer
from src.protocols.protocols import MQTTClient, HTTPAPIServer, ModbusRTU
from src.middleware.middleware import DataLogger, ConfigManager, EventBus, Watchdog
from src.utils.utils import FixedPoint, CRC, BitField, retry


# ════════════════════════════════════════════════
# Sensor Reading Tests
# ════════════════════════════════════════════════

class TestSensorReading:
    def test_creation(self):
        r = SensorReading(sensor_id="test", sensor_type="env",
                          values={"temp": 22.5, "humidity": 45.0}, unit="°C/%")
        assert r.primary_value == 22.5
        assert r.quality == 1.0

    def test_to_json(self):
        r = SensorReading(sensor_id="s1", sensor_type="env",
                          values={"temp": 20.0}, unit="°C")
        data = json.loads(r.to_json())
        assert data["sensor_id"] == "s1"
        assert data["values"]["temp"] == 20.0

    def test_iso_timestamp(self):
        r = SensorReading(sensor_id="s1", sensor_type="t", values={"v": 1}, unit="x")
        assert "T" in r.iso_timestamp


# ════════════════════════════════════════════════
# Sensor Driver Tests (Simulation Mode)
# ════════════════════════════════════════════════

class TestBME280:
    def test_init_and_read(self):
        sensor = BME280()
        sensor.initialize()
        reading = sensor.read()
        assert "temperature" in reading.values
        assert "humidity" in reading.values
        assert "pressure" in reading.values
        assert -40 < reading.values["temperature"] < 85

    def test_multiple_reads(self):
        sensor = BME280()
        sensor.initialize()
        readings = [sensor.read() for _ in range(10)]
        temps = [r.values["temperature"] for r in readings]
        assert len(set(temps)) > 1  # Values should vary slightly

    def test_health(self):
        sensor = BME280()
        sensor.initialize()
        sensor.read()
        h = sensor.health
        assert h["read_count"] == 1
        assert h["initialized"]

    def test_calibration(self):
        sensor = BME280()
        sensor.initialize()
        sensor.calibrate({"temperature": -0.5})
        r = sensor.read()
        assert r.values["temperature"] is not None


class TestMPU6050:
    def test_init_and_read(self):
        imu = MPU6050()
        imu.initialize()
        r = imu.read()
        assert "accel_x" in r.values
        assert "gyro_x" in r.values
        assert "pitch" in r.values
        assert "roll" in r.values

    def test_gravity_vector(self):
        imu = MPU6050()
        imu.initialize()
        r = imu.read()
        az = r.values["accel_z"]
        assert 0.8 < abs(az) < 1.2  # Should be ~1g


class TestAdditionalSensors:
    def test_ads1115(self):
        adc = ADS1115()
        adc.initialize()
        r = adc.read()
        assert "voltage" in r.values

    def test_gps(self):
        gps = GPSNEO6M()
        gps.initialize()
        r = gps.read()
        assert "latitude" in r.values
        assert "longitude" in r.values

    def test_hcsr04(self):
        us = HCSR04()
        us.initialize()
        r = us.read()
        assert "distance_cm" in r.values
        assert r.values["distance_cm"] > 0

    def test_dht22(self):
        dht = DHT22()
        dht.initialize()
        r = dht.read()
        assert "temperature" in r.values

    def test_bh1750(self):
        light = BH1750()
        light.initialize()
        r = light.read()
        assert "illuminance" in r.values

    def test_ina219(self):
        pwr = INA219()
        pwr.initialize()
        r = pwr.read()
        assert "voltage" in r.values
        assert "power_w" in r.values


# ════════════════════════════════════════════════
# Core Processing Tests
# ════════════════════════════════════════════════

class TestKalmanFilter:
    def test_converges_to_constant(self):
        kf = KalmanFilter(process_noise=0.01, measurement_noise=1.0)
        for _ in range(100):
            kf.update(10.0 + np.random.randn() * 0.5)
        assert abs(kf.x - 10.0) < 1.0

    def test_noise_reduction(self):
        kf = KalmanFilter(process_noise=0.01, measurement_noise=0.5)
        true_value = 25.0
        noisy = [true_value + np.random.randn() * 2 for _ in range(200)]
        filtered = kf.batch_filter(noisy)
        noise_before = np.std(noisy)
        noise_after = np.std([f - true_value for f in filtered[-50:]])
        assert noise_after < noise_before

    def test_adaptive_mode(self):
        kf = KalmanFilter(process_noise=0.01, measurement_noise=0.1, adaptive=True)
        for _ in range(100):
            kf.update(5.0 + np.random.randn() * 0.3)
        assert kf.R != 0.1  # Should have adapted

    def test_reset(self):
        kf = KalmanFilter()
        kf.update(100)
        kf.reset()
        assert kf.x == 0.0

    def test_2d_filter(self):
        kf2d = KalmanFilter2D(dt=0.1)
        for i in range(100):
            pos = i * 0.1 + np.random.randn() * 0.5
            est_pos, est_vel = kf2d.update(pos)
        assert abs(est_vel - 1.0) < 0.5  # Should estimate velocity ~1.0


class TestSensorFusion:
    def test_weighted_average(self):
        fusion = SensorFusion(strategy='weighted_average')
        s1, s2 = BME280(sensor_id="s1"), BME280(sensor_id="s2")
        s1.initialize(); s2.initialize()
        fusion.add_source("s1", weight=0.7, sensor=s1)
        fusion.add_source("s2", weight=0.3, sensor=s2)
        result = fusion.read("temperature")
        assert result.value != 0
        assert 0 < result.confidence <= 1

    def test_adaptive_fusion(self):
        fusion = SensorFusion(strategy='adaptive')
        for i in range(3):
            s = BME280(sensor_id=f"s{i}")
            s.initialize()
            fusion.add_source(f"s{i}", weight=1.0, sensor=s)
        result = fusion.read("temperature")
        assert result.value != 0

    def test_voting_fusion(self):
        fusion = SensorFusion(strategy='voting')
        for i in range(5):
            s = BME280(sensor_id=f"v{i}")
            s.initialize()
            fusion.add_source(f"v{i}", weight=1.0, sensor=s)
        result = fusion.read("temperature")
        assert result.confidence > 0


class TestAnomalyDetector:
    def test_detects_spike(self):
        det = AnomalyDetector(method='zscore', threshold=2.5, window_size=50)
        for _ in range(50):
            det.check(20.0 + np.random.randn() * 0.1)
        result = det.check(100.0)  # Big spike
        assert result is not None
        assert result.severity in ('warning', 'critical')

    def test_no_false_positives(self):
        det = AnomalyDetector(method='zscore', threshold=3.0, window_size=50)
        anomalies = 0
        for _ in range(200):
            r = det.check(20.0 + np.random.randn() * 0.1)
            if r: anomalies += 1
        assert anomalies < 10  # Very few false positives

    def test_iqr_method(self):
        det = AnomalyDetector(method='iqr', threshold=1.5)
        for _ in range(50):
            det.check(10 + np.random.randn() * 0.5)
        result = det.check(50)
        assert result is not None

    def test_rate_method(self):
        det = AnomalyDetector(method='rate', rate_limit=5.0)
        det.check(10.0, timestamp=1.0)
        for i in range(20):
            det.check(10.0 + i * 0.01, timestamp=1.0 + i * 0.1)
        result = det.check(100.0, timestamp=3.1)  # Sudden jump
        assert result is not None


class TestPIDController:
    def test_reaches_setpoint(self):
        pid = PIDController(kp=1.0, ki=0.5, kd=0.1, setpoint=100.0)
        value = 0.0
        for _ in range(500):
            output = pid.update(value, dt=0.01)
            value += output * 0.01
        assert abs(value - 100.0) < 20.0

    def test_output_limits(self):
        pid = PIDController(kp=100.0, setpoint=1000, output_limits=(-10, 10))
        output = pid.update(0.0, dt=0.01)
        assert -10 <= output <= 10


class TestRingBuffer:
    def test_basic(self):
        buf = RingBuffer(10)
        for i in range(5):
            buf.push(float(i))
        assert len(buf) == 5
        assert buf.mean == 2.0

    def test_overflow(self):
        buf = RingBuffer(5)
        for i in range(20):
            buf.push(float(i))
        assert len(buf) == 5
        assert buf.is_full

    def test_get_latest(self):
        buf = RingBuffer(100)
        for i in range(50):
            buf.push(float(i))
        latest = buf.get_latest(3)
        assert len(latest) == 3
        assert latest[-1] == 49.0


# ════════════════════════════════════════════════
# Protocol Tests
# ════════════════════════════════════════════════

class TestHTTPAPI:
    def test_list_sensors(self):
        api = HTTPAPIServer()
        api.register_sensor("bme", BME280())
        resp = api.handle_request("GET", "/api/sensors")
        assert "bme" in resp["sensors"]

    def test_read_sensor(self):
        s = BME280()
        s.initialize()
        api = HTTPAPIServer()
        api.register_sensor("bme", s)
        resp = api.handle_request("GET", "/api/sensors/bme")
        assert resp["status"] == 200
        assert "temperature" in resp["data"]["values"]

    def test_not_found(self):
        api = HTTPAPIServer()
        resp = api.handle_request("GET", "/api/sensors/nonexistent")
        assert resp["status"] == 404

    def test_health(self):
        api = HTTPAPIServer()
        resp = api.handle_request("GET", "/api/system/health")
        assert resp["status"] == "healthy"


class TestModbus:
    def test_read_registers_sim(self):
        mb = ModbusRTU()
        mb.connect()
        values = mb.read_holding_registers(1, 0, 4)
        assert len(values) == 4

    def test_crc16(self):
        data = b'\x01\x03\x00\x00\x00\x02'
        crc = ModbusRTU._crc16(data)
        assert len(crc) == 2


# ════════════════════════════════════════════════
# Middleware Tests
# ════════════════════════════════════════════════

class TestDataLogger:
    def test_sqlite_logging(self, tmp_path):
        logger = DataLogger(path=str(tmp_path), format="sqlite")
        s = BME280()
        s.initialize()
        r = s.read()
        assert logger.log(r)
        results = logger.query(s.sensor_id, limit=10)
        assert len(results) >= 1
        logger.close()

    def test_jsonl_logging(self, tmp_path):
        logger = DataLogger(path=str(tmp_path), format="jsonl")
        s = BME280()
        s.initialize()
        assert logger.log(s.read())
        assert (tmp_path / "sensor_data.jsonl").exists()


class TestConfigManager:
    def test_dot_notation(self):
        cfg = ConfigManager(defaults={"sensors": {"bme280": {"bus": 1}}})
        assert cfg.get("sensors.bme280.bus") == 1

    def test_set_and_get(self):
        cfg = ConfigManager()
        cfg.set("mqtt.broker", "192.168.1.1")
        assert cfg.get("mqtt.broker") == "192.168.1.1"

    def test_default(self):
        cfg = ConfigManager()
        assert cfg.get("nonexistent", 42) == 42

    def test_save_load(self, tmp_path):
        cfg = ConfigManager(defaults={"key": "value"})
        path = str(tmp_path / "config.json")
        cfg.save(path)
        cfg2 = ConfigManager(config_path=path)
        assert cfg2.get("key") == "value"


class TestEventBus:
    def test_emit_and_receive(self):
        bus = EventBus()
        received = []
        bus.on("sensor.read", lambda t, d: received.append(d))
        bus.emit("sensor.read", {"temp": 22})
        assert len(received) == 1
        assert received[0]["temp"] == 22

    def test_wildcard(self):
        bus = EventBus()
        all_events = []
        bus.on("*", lambda t, d: all_events.append(t))
        bus.emit("a"); bus.emit("b")
        assert len(all_events) == 2

    def test_off(self):
        bus = EventBus()
        calls = []
        handler = lambda t, d: calls.append(1)
        bus.on("x", handler)
        bus.emit("x")
        bus.off("x", handler)
        bus.emit("x")
        assert len(calls) == 1


class TestWatchdog:
    def test_healthy(self):
        wd = Watchdog(timeout_seconds=1.0)
        wd.register("sensor1")
        wd.heartbeat("sensor1")
        timed_out = wd.check()
        assert "sensor1" not in timed_out

    def test_timeout(self):
        wd = Watchdog(timeout_seconds=0.01)
        wd.register("sensor1")
        time.sleep(0.02)
        timed_out = wd.check()
        assert "sensor1" in timed_out


# ════════════════════════════════════════════════
# Utility Tests
# ════════════════════════════════════════════════

class TestFixedPoint:
    def test_basic_ops(self):
        a = FixedPoint(1.5)
        b = FixedPoint(2.25)
        assert abs((a + b).value - 3.75) < 0.001
        assert abs((a * b).value - 3.375) < 0.01

    def test_division(self):
        a = FixedPoint(10.0)
        b = FixedPoint(3.0)
        assert abs((a / b).value - 3.333) < 0.01

    def test_repr(self):
        f = FixedPoint(3.14)
        assert "Q16" in repr(f)


class TestCRC:
    def test_crc8(self):
        assert CRC.crc8(b'\x01\x02\x03') > 0

    def test_crc16(self):
        data = b'Hello'
        c = CRC.crc16(data)
        assert CRC.verify(data, c, 'crc16')

    def test_crc32(self):
        assert CRC.crc32(b'test') > 0


class TestBitField:
    def test_get_set_bit(self):
        bf = BitField(0x00)
        bf.set_bit(3, True)
        assert bf.get_bit(3)
        assert int(bf) == 0x08

    def test_get_set_bits(self):
        bf = BitField(0xFF)
        assert bf.get_bits(4, 4) == 0x0F
        bf.set_bits(0, 4, 0x05)
        assert bf.get_bits(0, 4) == 0x05


class TestRetry:
    def test_succeeds_eventually(self):
        call_count = [0]

        @retry(max_attempts=3, base_delay=0.01)
        def flaky():
            call_count[0] += 1
            if call_count[0] < 3:
                raise ValueError("not yet")
            return "ok"

        assert flaky() == "ok"
        assert call_count[0] == 3

    def test_raises_after_max(self):
        @retry(max_attempts=2, base_delay=0.01)
        def always_fail():
            raise RuntimeError("fail")

        with pytest.raises(RuntimeError):
            always_fail()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
