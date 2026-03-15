# ⚡ Embedded Sensor System — Python IoT Framework

[![Python](https://img.shields.io/badge/Python-3.9%2B-3776AB.svg?logo=python&logoColor=white)](https://python.org)
[![MicroPython](https://img.shields.io/badge/MicroPython-Compatible-2B2728.svg?logo=micropython)](https://micropython.org)
[![MQTT](https://img.shields.io/badge/MQTT-3.1.1%2F5.0-660066.svg)](https://mqtt.org)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![CI](https://img.shields.io/badge/CI-GitHub_Actions-2088FF.svg?logo=github-actions&logoColor=white)](.github/workflows/ci.yml)

A production-grade **embedded sensor data acquisition and processing framework** for IoT systems. Supports multi-sensor fusion, real-time filtering, MQTT/HTTP telemetry, OTA firmware updates, and a live monitoring dashboard — all in Python.

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Dashboard (Web UI)                     │
│              Real-time charts & alerts                   │
└──────────────────────┬──────────────────────────────────┘
                       │ WebSocket / REST
┌──────────────────────▼──────────────────────────────────┐
│                  Middleware Layer                         │
│   ┌──────────┐  ┌──────────┐  ┌───────────┐            │
│   │ MQTT     │  │ HTTP     │  │ Data      │            │
│   │ Broker   │  │ REST API │  │ Logger    │            │
│   └──────────┘  └──────────┘  └───────────┘            │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                  Processing Core                         │
│   ┌──────────┐  ┌──────────┐  ┌───────────┐            │
│   │ Kalman   │  │ Sensor   │  │ Anomaly   │            │
│   │ Filter   │  │ Fusion   │  │ Detector  │            │
│   └──────────┘  └──────────┘  └───────────┘            │
└──────────────────────┬──────────────────────────────────┘
                       │
┌──────────────────────▼──────────────────────────────────┐
│                  Hardware Abstraction                     │
│   ┌─────┐  ┌───────┐  ┌─────┐  ┌──────┐  ┌─────────┐  │
│   │BME280│  │MPU6050│  │ADS  │  │GPS   │  │Ultrasonic│  │
│   │T/H/P │  │IMU    │  │1115 │  │NEO-6M│  │HC-SR04  │  │
│   └─────┘  └───────┘  └─────┘  └──────┘  └─────────┘  │
└─────────────────────────────────────────────────────────┘
          I²C           SPI          UART        GPIO
```

## ✨ Features

| Category | Feature | Description |
|----------|---------|-------------|
| **Sensors** | 8+ drivers | BME280, MPU6050, ADS1115, GPS NEO-6M, HC-SR04, DHT22, BH1750, INA219 |
| **Processing** | Kalman filter | Adaptive noise estimation with configurable process/measurement noise |
| **Processing** | Sensor fusion | Multi-sensor weighted fusion with confidence scoring |
| **Processing** | Anomaly detection | Z-score & sliding window anomaly detection with alerts |
| **Protocols** | MQTT 3.1.1/5.0 | Async publish/subscribe with QoS levels, TLS support |
| **Protocols** | HTTP REST | RESTful API for sensor data retrieval and configuration |
| **Protocols** | Modbus RTU | Industrial protocol support for legacy equipment |
| **Middleware** | Data logger | SQLite + CSV + JSON logging with rotation and compression |
| **Middleware** | OTA updates | Over-the-air firmware update mechanism |
| **Middleware** | Watchdog | Hardware/software watchdog with auto-recovery |
| **Dashboard** | Web UI | Real-time monitoring with charts, alerts, and system health |
| **Testing** | 50+ tests | Unit, integration, and hardware simulation tests |

## 📁 Project Structure

```
embedded-sensor-system/
├── src/
│   ├── drivers/                # Hardware sensor drivers
│   │   ├── base_sensor.py      # Abstract sensor interface
│   │   ├── bme280.py           # Temperature/Humidity/Pressure
│   │   ├── mpu6050.py          # 6-axis IMU (Accel + Gyro)
│   │   ├── ads1115.py          # 16-bit ADC
│   │   ├── gps_neo6m.py        # GPS receiver
│   │   ├── hcsr04.py           # Ultrasonic distance
│   │   ├── dht22.py            # Temperature/Humidity
│   │   ├── bh1750.py           # Ambient light
│   │   └── ina219.py           # Current/Power monitor
│   ├── core/                   # Signal processing engine
│   │   ├── kalman_filter.py    # Kalman filter implementation
│   │   ├── sensor_fusion.py    # Multi-sensor data fusion
│   │   ├── anomaly_detector.py # Anomaly detection engine
│   │   ├── pid_controller.py   # PID control loop
│   │   └── ring_buffer.py      # Lock-free circular buffer
│   ├── protocols/              # Communication protocols
│   │   ├── mqtt_client.py      # MQTT pub/sub client
│   │   ├── http_api.py         # REST API server
│   │   └── modbus_rtu.py       # Modbus RTU master/slave
│   ├── middleware/              # System services
│   │   ├── data_logger.py      # Multi-format data logging
│   │   ├── ota_updater.py      # OTA firmware updates
│   │   ├── watchdog.py         # System watchdog
│   │   ├── config_manager.py   # YAML/JSON configuration
│   │   └── event_bus.py        # Pub/sub event system
│   └── utils/                  # Utilities
│       ├── fixed_point.py      # Fixed-point math (no FPU)
│       ├── crc.py              # CRC-8/16/32 checksums
│       ├── bitfield.py         # Register bit manipulation
│       └── retry.py            # Retry with backoff
├── tests/                      # Test suite
├── examples/                   # Usage examples
├── configs/                    # Configuration files
├── dashboard/                  # Web monitoring UI
├── scripts/                    # Deployment scripts
├── docs/                       # Documentation
├── .github/workflows/          # CI/CD
├── requirements.txt
├── setup.py
└── README.md
```

## 🚀 Quick Start

### Installation

```bash
git clone https://github.com/Hayder-IRAQ/embedded-sensor-system.git
cd embedded-sensor-system
pip install -r requirements.txt
pip install -e .
```

### Basic Usage — Read a BME280 Sensor

```python
from src.drivers.bme280 import BME280
from src.core.kalman_filter import KalmanFilter

# Initialize sensor on I2C bus
sensor = BME280(bus_id=1, address=0x76)

# Apply Kalman filtering for noise reduction
kf = KalmanFilter(process_noise=0.01, measurement_noise=0.1)

while True:
    reading = sensor.read()
    filtered = kf.update(reading.temperature)
    print(f"Temp: {filtered:.2f}°C | Humidity: {reading.humidity:.1f}%")
```

### Multi-Sensor Fusion

```python
from src.core.sensor_fusion import SensorFusion
from src.drivers.bme280 import BME280
from src.drivers.dht22 import DHT22

fusion = SensorFusion(strategy="weighted_average")
fusion.add_source("bme280", weight=0.7, sensor=BME280(bus_id=1))
fusion.add_source("dht22", weight=0.3, sensor=DHT22(pin=4))

result = fusion.read("temperature")
print(f"Fused temp: {result.value:.2f}°C (confidence: {result.confidence:.0%})")
```

### MQTT Telemetry

```python
from src.protocols.mqtt_client import MQTTClient
from src.drivers.bme280 import BME280

mqtt = MQTTClient(broker="mqtt.example.com", port=8883, tls=True)
sensor = BME280(bus_id=1)

async def publish_telemetry():
    await mqtt.connect()
    while True:
        data = sensor.read()
        await mqtt.publish("sensors/env", data.to_json(), qos=1)
        await asyncio.sleep(5)
```

### Run Tests

```bash
python -m pytest tests/ -v --tb=short
```

## 📊 Dashboard

Launch the real-time monitoring dashboard:

```bash
python dashboard/app.py
# Open http://localhost:8050
```

## 🔧 Configuration

```yaml
# configs/system.yaml
system:
  name: "weather-station-01"
  sample_rate_hz: 10
  log_level: INFO

sensors:
  bme280:
    bus: 1
    address: 0x76
    oversample: 16
  mpu6050:
    bus: 1
    address: 0x68
    accel_range: "4g"
    gyro_range: "500dps"

processing:
  kalman:
    process_noise: 0.01
    measurement_noise: 0.1
  anomaly:
    method: "zscore"
    threshold: 3.0
    window_size: 100

telemetry:
  mqtt:
    broker: "mqtt.example.com"
    port: 8883
    tls: true
    topic_prefix: "iot/sensors"
    qos: 1
```

## 🎯 Target Platforms

| Platform | Status | Notes |
|----------|--------|-------|
| Raspberry Pi 4/5 | ✅ Full support | Primary development target |
| Raspberry Pi Pico W | ✅ MicroPython | Via MicroPython compatibility layer |
| ESP32 | ✅ MicroPython | WiFi + BLE capable |
| BeagleBone Black | ✅ Full support | Industrial-grade |
| Jetson Nano | ✅ Full support | GPU-accelerated processing |
| Desktop (Simulated) | ✅ Full support | For development and testing |

## 📖 Documentation

- [Architecture Guide](docs/ARCHITECTURE.md)
- [API Reference](docs/API.md)
- [Sensor Wiring Guide](docs/WIRING.md)
- [Deployment Guide](docs/DEPLOYMENT.md)

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## 📄 License

MIT License — see [LICENSE](LICENSE) for details.
