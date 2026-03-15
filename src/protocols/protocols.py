"""
Communication Protocols — MQTT, HTTP REST, Modbus RTU
======================================================
"""

import json
import time
import asyncio
import struct
from typing import Any, Callable, Dict, List, Optional
from dataclasses import dataclass


# ════════════════════════════════════════════════
# MQTT Client
# ════════════════════════════════════════════════

class MQTTClient:
    """
    Async MQTT client wrapper with QoS, TLS, and auto-reconnect.

    Parameters
    ----------
    broker : str
        MQTT broker hostname.
    port : int
        Broker port (1883 plain, 8883 TLS).
    tls : bool
        Enable TLS encryption.
    client_id : str
        Unique client identifier.
    """

    def __init__(self, broker: str = "localhost", port: int = 1883,
                 tls: bool = False, client_id: Optional[str] = None,
                 username: Optional[str] = None, password: Optional[str] = None):
        self.broker = broker
        self.port = port
        self.tls = tls
        self.client_id = client_id or f"ess_{int(time.time())}"
        self.username = username
        self.password = password
        self._client = None
        self._connected = False
        self._subscriptions: Dict[str, List[Callable]] = {}
        self._message_count = 0
        self._sim_mode = True

    async def connect(self):
        """Connect to the MQTT broker."""
        try:
            import paho.mqtt.client as mqtt
            self._client = mqtt.Client(client_id=self.client_id,
                                        protocol=mqtt.MQTTv311)
            if self.username:
                self._client.username_pw_set(self.username, self.password)
            if self.tls:
                self._client.tls_set()

            self._client.on_connect = self._on_connect
            self._client.on_message = self._on_message
            self._client.on_disconnect = self._on_disconnect

            self._client.connect_async(self.broker, self.port)
            self._client.loop_start()
            self._sim_mode = False
            self._connected = True
        except (ImportError, OSError):
            self._sim_mode = True
            self._connected = True  # Simulated

    async def publish(self, topic: str, payload: Any, qos: int = 0,
                      retain: bool = False):
        """Publish a message to a topic."""
        if isinstance(payload, dict):
            payload = json.dumps(payload)
        elif not isinstance(payload, str):
            payload = str(payload)

        if self._sim_mode:
            self._message_count += 1
            return True

        result = self._client.publish(topic, payload, qos=qos, retain=retain)
        self._message_count += 1
        return result.rc == 0

    async def subscribe(self, topic: str, callback: Callable, qos: int = 0):
        """Subscribe to a topic with a callback handler."""
        if topic not in self._subscriptions:
            self._subscriptions[topic] = []
        self._subscriptions[topic].append(callback)

        if not self._sim_mode and self._client:
            self._client.subscribe(topic, qos)

    async def disconnect(self):
        if self._client and not self._sim_mode:
            self._client.loop_stop()
            self._client.disconnect()
        self._connected = False

    def _on_connect(self, client, userdata, flags, rc):
        self._connected = True
        for topic in self._subscriptions:
            client.subscribe(topic)

    def _on_message(self, client, userdata, msg):
        callbacks = self._subscriptions.get(msg.topic, [])
        for cb in callbacks:
            try:
                cb(msg.topic, msg.payload.decode())
            except Exception:
                pass

    def _on_disconnect(self, client, userdata, rc):
        self._connected = False

    @property
    def stats(self) -> Dict[str, Any]:
        return {
            "broker": self.broker,
            "connected": self._connected,
            "sim_mode": self._sim_mode,
            "messages_sent": self._message_count,
            "subscriptions": list(self._subscriptions.keys()),
        }


# ════════════════════════════════════════════════
# HTTP REST API
# ════════════════════════════════════════════════

class HTTPAPIServer:
    """
    Lightweight REST API server for sensor data.

    Endpoints:
        GET  /api/sensors           — List all sensors
        GET  /api/sensors/<id>      — Read specific sensor
        GET  /api/sensors/<id>/history — Sensor history
        POST /api/sensors/<id>/calibrate — Set calibration
        GET  /api/system/health     — System health
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8080):
        self.host = host
        self.port = port
        self._sensors: Dict[str, Any] = {}
        self._history: Dict[str, list] = {}

    def register_sensor(self, sensor_id: str, sensor):
        self._sensors[sensor_id] = sensor
        self._history[sensor_id] = []

    def get_routes(self) -> Dict[str, str]:
        return {
            "GET /api/sensors": "List all registered sensors",
            "GET /api/sensors/<id>": "Read latest from sensor",
            "GET /api/sensors/<id>/history": "Get reading history",
            "POST /api/sensors/<id>/calibrate": "Set calibration offsets",
            "GET /api/system/health": "System health and metrics",
        }

    def handle_request(self, method: str, path: str, body: Optional[dict] = None) -> dict:
        """Process an API request (framework-agnostic handler)."""
        if method == "GET" and path == "/api/sensors":
            return {"sensors": list(self._sensors.keys()), "count": len(self._sensors)}

        if method == "GET" and path.startswith("/api/sensors/"):
            parts = path.split("/")
            sensor_id = parts[3] if len(parts) > 3 else None
            if not sensor_id or sensor_id not in self._sensors:
                return {"error": "Sensor not found", "status": 404}

            if len(parts) > 4 and parts[4] == "history":
                return {"sensor_id": sensor_id,
                        "history": self._history.get(sensor_id, [])[-100:]}

            sensor = self._sensors[sensor_id]
            try:
                reading = sensor.read()
                self._history.setdefault(sensor_id, []).append(reading.to_dict())
                return {"status": 200, "data": reading.to_dict()}
            except Exception as e:
                return {"error": str(e), "status": 500}

        if method == "GET" and path == "/api/system/health":
            return {
                "status": "healthy",
                "sensors": {sid: s.health for sid, s in self._sensors.items()},
                "uptime": time.time(),
            }

        return {"error": "Not found", "status": 404}


# ════════════════════════════════════════════════
# Modbus RTU
# ════════════════════════════════════════════════

class ModbusRTU:
    """
    Modbus RTU master for industrial sensor communication.

    Supports function codes: 0x03 (Read Holding Registers),
    0x04 (Read Input Registers), 0x06 (Write Single Register).
    """

    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 9600,
                 timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._sim_mode = True

    def connect(self):
        try:
            import serial
            self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._sim_mode = False
        except (ImportError, OSError):
            self._sim_mode = True

    def read_holding_registers(self, slave_id: int, start: int, count: int) -> List[int]:
        """Read holding registers (FC 0x03)."""
        if self._sim_mode:
            import random
            return [random.randint(0, 65535) for _ in range(count)]

        frame = struct.pack('>BBHH', slave_id, 0x03, start, count)
        frame += self._crc16(frame)
        self._serial.write(frame)

        resp = self._serial.read(5 + count * 2)
        if len(resp) < 5 + count * 2:
            raise IOError("Modbus: incomplete response")

        values = []
        for i in range(count):
            val = struct.unpack('>H', resp[3 + i * 2: 5 + i * 2])[0]
            values.append(val)
        return values

    def write_register(self, slave_id: int, address: int, value: int):
        """Write single register (FC 0x06)."""
        if self._sim_mode:
            return True
        frame = struct.pack('>BBHH', slave_id, 0x06, address, value)
        frame += self._crc16(frame)
        self._serial.write(frame)
        resp = self._serial.read(8)
        return len(resp) == 8

    @staticmethod
    def _crc16(data: bytes) -> bytes:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return struct.pack('<H', crc)

    def disconnect(self):
        if self._serial:
            self._serial.close()
