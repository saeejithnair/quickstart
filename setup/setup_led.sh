#!/bin/bash

# Exit on error
set -e

# Important Notes:
# - Use MOSI pin (GPIO10) on PI5 for LED strip
# - LED strips are directional - check input/output direction
# - Pi5Neo library is used instead of Adafruit NeoPixel for PI5 compatibility

echo "Installing LED dependencies..."

# Install system dependencies for SPI and IPC
sudo apt-get update
sudo apt-get install -y python3-sysv-ipc

# Enable SPI interface
sudo raspi-config nonint do_spi 0

# Update pip
pip3 install --upgrade pip

# Install required Python packages
pip3 install spidev Pi5Neo

echo "Applying patch to Pi5Neo library..."

# Search for pi5neo.py in the home directory virtual environment
PI5NEO_PATH=~/.venv/lib/python3.11/site-packages/pi5neo/pi5neo.py

if [ -f "$PI5NEO_PATH" ]; then
    sed -i 's/time\.sleep(0\.1)/time.sleep(0.01)/g' "$PI5NEO_PATH"
    echo "Patched $PI5NEO_PATH successfully."
else
    echo "Error: pi5neo.py not found at $PI5NEO_PATH. Please ensure pi5neo package is installed."
fi

echo "LED setup complete!"

echo -e "\nIMPORTANT NOTES:"
echo "1. Connect LED strip to MOSI pin (GPIO10) on PI5"
echo "2. Check LED strip direction - they have specific input/output ends"
echo "3. The Pi5Neo library is used as Adafruit NeoPixel is not PI5 compatible"
