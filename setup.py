from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="embedded-sensor-system",
    version="2.0.0",
    author="Hayder-IRAQ",
    description="Production IoT framework for embedded sensor data acquisition and processing",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Hayder-IRAQ/embedded-sensor-system",
    packages=find_packages(),
    python_requires=">=3.9",
    install_requires=["numpy>=1.20.0"],
    extras_require={
        "hardware": ["smbus2>=0.4", "RPi.GPIO>=0.7", "pyserial>=3.5", "adafruit-circuitpython-dht>=3.7"],
        "mqtt": ["paho-mqtt>=1.6"],
        "web": ["flask>=2.0", "flask-socketio>=5.0"],
        "dev": ["pytest>=7.0", "pyyaml>=6.0"],
        "full": ["smbus2", "paho-mqtt", "flask", "flask-socketio", "pyyaml", "matplotlib"],
    },
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Topic :: System :: Hardware",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
    ],
    keywords="iot embedded sensor mqtt kalman fpga raspberry-pi esp32 micropython",
)
