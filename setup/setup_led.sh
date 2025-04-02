#!/bin/bash

# Exit on error
set -e

echo "Installing LED dependencies..."

# Install system dependencies for SPI
sudo apt-get update
sudo apt-get install -y python3-spidev

# Enable SPI interface
sudo raspi-config nonint do_spi 0

# Install required Python packages
python3 -m pip install Pi5Neo spidev

echo "Applying patch to Pi5Neo library..."

# Find pi5neo.py in the virtual environment and replace time.sleep(0.1) with time.sleep(0.01)
PI5NEO_PATH=$(find ~/ -type f -name 'pi5neo.py' 2>/dev/null | grep -E '/(\.venv|venv)/lib')

if [ -f "$PI5NEO_PATH" ]; then
    sed -i 's/time\.sleep(0\.1)/time.sleep(0.01)/g' "$PI5NEO_PATH"
    echo "Patched $PI5NEO_PATH successfully."
else
    echo "pi5neo.py not found in virtual environment."
fi

echo "LED setup complete!"


