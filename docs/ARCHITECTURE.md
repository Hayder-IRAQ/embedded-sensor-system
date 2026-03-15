# Architecture Guide

## System Overview

The Embedded Sensor System follows a layered architecture where each layer is independently testable and replaceable.

## Layers

### Hardware Abstraction Layer (HAL)

All sensor drivers inherit from `BaseSensor` which enforces a consistent interface: `initialize()` → `read()` → `SensorReading`. Every driver includes a simulation mode that activates automatically when hardware libraries are unavailable, enabling development and testing on any platform.

### Processing Core

The processing layer operates on `SensorReading` objects and produces refined outputs. The Kalman filter reduces measurement noise with configurable process/measurement covariance. The sensor fusion engine combines multiple sensors measuring the same quantity using weighted average, adaptive, Kalman-based, or voting strategies. The anomaly detector identifies outliers in real-time using Z-score, IQR, rate-of-change, or combined methods.

### Protocol Layer

Communication protocols handle data transport. The MQTT client provides async publish/subscribe with QoS and TLS. The HTTP API exposes sensor data via REST endpoints. Modbus RTU supports industrial equipment communication.

### Middleware

Cross-cutting services include multi-format data logging (SQLite, CSV, JSON Lines), hierarchical configuration management with dot-notation access, an in-process event bus for decoupled communication, and a software watchdog for liveness monitoring.

## Data Flow

```
Sensor Hardware → Driver.read() → SensorReading
    → KalmanFilter.update() → filtered value
    → AnomalyDetector.check() → alerts
    → SensorFusion.read() → fused reading
    → DataLogger.log() → persistent storage
    → MQTTClient.publish() → cloud telemetry
    → EventBus.emit() → internal subscribers
```

## Design Decisions

**Simulation by default**: Every driver falls back to realistic simulation when hardware is absent. This means the entire system runs on a laptop without any sensors connected.

**Dataclass-based readings**: `SensorReading` is an immutable dataclass with JSON serialization, making it easy to log, transmit, and test.

**Zero external dependencies for core**: The `src/core/` and `src/drivers/` modules require only NumPy. Hardware libraries, MQTT, and web frameworks are optional extras.
