# Contributing to Embedded Sensor System

Thank you for your interest in contributing!

## Getting Started

```bash
git clone https://github.com/Hayder-IRAQ/embedded-sensor-system.git
cd embedded-sensor-system
pip install -r requirements.txt
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## Adding a New Sensor Driver

1. Create `src/drivers/your_sensor.py` inheriting from `BaseSensor`
2. Implement `_init_hardware()`, `_read_raw()`, `_parse()`
3. Include simulation mode fallback
4. Add tests in `tests/`
5. Update `src/drivers/__init__.py`

## Code Style

- PEP 8 with 100-char line limit
- Type hints on public methods
- NumPy-style docstrings
- All sensors must support simulation mode

## Pull Request Process

1. All tests must pass
2. Include tests for new features
3. Update documentation
4. Clear commit messages

## License

MIT — contributions are licensed under the same terms.
