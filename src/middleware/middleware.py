"""
Middleware Services — Logging, Config, Events, Watchdog
========================================================
"""

import os
import json
import time
import csv
import sqlite3
import threading
import logging
from typing import Any, Callable, Dict, List, Optional
from datetime import datetime, timezone
from pathlib import Path


# ════════════════════════════════════════════════
# Data Logger — Multi-Format Persistent Storage
# ════════════════════════════════════════════════

class DataLogger:
    """
    Multi-format sensor data logger with automatic rotation.

    Formats: SQLite (default), CSV, JSON Lines.

    Parameters
    ----------
    path : str
        Output directory for log files.
    format : str
        'sqlite', 'csv', or 'jsonl'.
    max_entries : int
        Max entries before rotation (0 = unlimited).
    """

    def __init__(self, path: str = "./data", format: str = "sqlite",
                 max_entries: int = 100000):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=True)
        self.format = format
        self.max_entries = max_entries
        self._count = 0
        self._db = None
        self._csv_writer = None
        self._csv_file = None
        self._lock = threading.Lock()

        if format == "sqlite":
            self._init_sqlite()
        elif format == "csv":
            self._init_csv()

    def _init_sqlite(self):
        db_path = self.path / "sensor_data.db"
        self._db = sqlite3.connect(str(db_path), check_same_thread=False)
        self._db.execute("""
            CREATE TABLE IF NOT EXISTS readings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                sensor_id TEXT NOT NULL,
                sensor_type TEXT,
                timestamp REAL NOT NULL,
                values_json TEXT NOT NULL,
                quality REAL DEFAULT 1.0
            )
        """)
        self._db.execute("CREATE INDEX IF NOT EXISTS idx_sensor_ts ON readings(sensor_id, timestamp)")
        self._db.commit()

    def _init_csv(self):
        csv_path = self.path / f"sensor_data_{int(time.time())}.csv"
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp', 'sensor_id', 'sensor_type', 'values', 'quality'])

    def log(self, reading) -> bool:
        """Log a SensorReading to the active store."""
        with self._lock:
            try:
                if self.format == "sqlite":
                    self._db.execute(
                        "INSERT INTO readings (sensor_id, sensor_type, timestamp, values_json, quality) "
                        "VALUES (?, ?, ?, ?, ?)",
                        (reading.sensor_id, reading.sensor_type, reading.timestamp,
                         json.dumps(reading.values), reading.quality)
                    )
                    if self._count % 100 == 0:
                        self._db.commit()

                elif self.format == "csv":
                    self._csv_writer.writerow([
                        reading.timestamp, reading.sensor_id, reading.sensor_type,
                        json.dumps(reading.values), reading.quality
                    ])

                elif self.format == "jsonl":
                    log_path = self.path / "sensor_data.jsonl"
                    with open(log_path, 'a') as f:
                        f.write(json.dumps(reading.to_dict()) + '\n')

                self._count += 1

                if self.max_entries > 0 and self._count >= self.max_entries:
                    self._rotate()

                return True
            except Exception:
                return False

    def query(self, sensor_id: str, start_time: Optional[float] = None,
              end_time: Optional[float] = None, limit: int = 1000) -> List[dict]:
        """Query logged data (SQLite only)."""
        if self.format != "sqlite" or not self._db:
            return []

        sql = "SELECT timestamp, values_json, quality FROM readings WHERE sensor_id = ?"
        params: list = [sensor_id]

        if start_time:
            sql += " AND timestamp >= ?"
            params.append(start_time)
        if end_time:
            sql += " AND timestamp <= ?"
            params.append(end_time)

        sql += " ORDER BY timestamp DESC LIMIT ?"
        params.append(limit)

        rows = self._db.execute(sql, params).fetchall()
        return [{"timestamp": r[0], "values": json.loads(r[1]), "quality": r[2]} for r in rows]

    def _rotate(self):
        if self.format == "sqlite" and self._db:
            self._db.commit()
        elif self.format == "csv" and self._csv_file:
            self._csv_file.close()
            self._init_csv()
        self._count = 0

    @property
    def stats(self) -> dict:
        return {"format": self.format, "entries": self._count, "path": str(self.path)}

    def close(self):
        if self._db:
            self._db.commit()
            self._db.close()
        if self._csv_file:
            self._csv_file.close()


# ════════════════════════════════════════════════
# Configuration Manager
# ════════════════════════════════════════════════

class ConfigManager:
    """
    Hierarchical configuration with YAML/JSON support and defaults.
    """

    def __init__(self, config_path: Optional[str] = None, defaults: Optional[dict] = None):
        self._config = defaults or {}
        if config_path:
            self.load(config_path)

    def load(self, path: str):
        """Load configuration from YAML or JSON file."""
        path = Path(path)
        if not path.exists():
            return

        with open(path) as f:
            if path.suffix in ('.yaml', '.yml'):
                try:
                    import yaml
                    data = yaml.safe_load(f)
                except ImportError:
                    return
            else:
                data = json.load(f)

        self._deep_merge(self._config, data)

    def get(self, key: str, default: Any = None) -> Any:
        """Get a value using dot notation: 'sensors.bme280.bus'"""
        keys = key.split('.')
        val = self._config
        for k in keys:
            if isinstance(val, dict) and k in val:
                val = val[k]
            else:
                return default
        return val

    def set(self, key: str, value: Any):
        """Set a value using dot notation."""
        keys = key.split('.')
        d = self._config
        for k in keys[:-1]:
            d = d.setdefault(k, {})
        d[keys[-1]] = value

    def save(self, path: str):
        """Save configuration to JSON file."""
        with open(path, 'w') as f:
            json.dump(self._config, f, indent=2)

    @staticmethod
    def _deep_merge(base: dict, override: dict):
        for key, val in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(val, dict):
                ConfigManager._deep_merge(base[key], val)
            else:
                base[key] = val

    def __getitem__(self, key):
        return self.get(key)

    def to_dict(self) -> dict:
        return self._config.copy()


# ════════════════════════════════════════════════
# Event Bus — Pub/Sub for Internal Communication
# ════════════════════════════════════════════════

class EventBus:
    """
    In-process publish/subscribe event system.

    Events decouple sensor drivers from processing and telemetry layers.
    """

    def __init__(self):
        self._handlers: Dict[str, List[Callable]] = {}
        self._event_count = 0

    def on(self, event_type: str, handler: Callable):
        """Register a handler for an event type."""
        self._handlers.setdefault(event_type, []).append(handler)

    def off(self, event_type: str, handler: Callable):
        """Unregister a handler."""
        if event_type in self._handlers:
            self._handlers[event_type] = [h for h in self._handlers[event_type] if h != handler]

    def emit(self, event_type: str, data: Any = None):
        """Emit an event to all registered handlers."""
        self._event_count += 1
        for handler in self._handlers.get(event_type, []):
            try:
                handler(event_type, data)
            except Exception as e:
                logging.error(f"EventBus handler error: {e}")

        # Wildcard handlers
        for handler in self._handlers.get('*', []):
            try:
                handler(event_type, data)
            except Exception:
                pass

    @property
    def stats(self) -> dict:
        return {
            "event_types": list(self._handlers.keys()),
            "total_handlers": sum(len(v) for v in self._handlers.values()),
            "events_emitted": self._event_count,
        }


# ════════════════════════════════════════════════
# Watchdog — System Health Monitor
# ════════════════════════════════════════════════

class Watchdog:
    """
    Software watchdog for system health monitoring.

    Monitors sensor liveness, system resources, and triggers
    recovery actions when thresholds are exceeded.
    """

    def __init__(self, timeout_seconds: float = 30.0, check_interval: float = 5.0):
        self.timeout = timeout_seconds
        self.check_interval = check_interval
        self._monitors: Dict[str, float] = {}  # name → last_heartbeat
        self._callbacks: Dict[str, Callable] = {}
        self._running = False

    def register(self, name: str, callback: Optional[Callable] = None):
        """Register a component to monitor."""
        self._monitors[name] = time.time()
        if callback:
            self._callbacks[name] = callback

    def heartbeat(self, name: str):
        """Signal that a component is alive."""
        self._monitors[name] = time.time()

    def check(self) -> List[str]:
        """Check all monitors. Returns list of timed-out component names."""
        now = time.time()
        timed_out = []
        for name, last_beat in self._monitors.items():
            if now - last_beat > self.timeout:
                timed_out.append(name)
                if name in self._callbacks:
                    try:
                        self._callbacks[name](name)
                    except Exception:
                        pass
        return timed_out

    @property
    def status(self) -> Dict[str, Any]:
        now = time.time()
        return {
            name: {
                "last_heartbeat": last,
                "age_seconds": round(now - last, 1),
                "healthy": (now - last) < self.timeout,
            }
            for name, last in self._monitors.items()
        }
