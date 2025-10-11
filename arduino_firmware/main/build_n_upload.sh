#!/bin/bash

PORT="/dev/cu.usbmodem1201"
FQBN="arduino:mbed_nano:nano33ble"

echo "🔨 Compiling SensorTest..."
arduino-cli compile --fqbn $FQBN .

if [ $? -eq 0 ]; then
    echo "Compilation successful!"
    echo "Uploading to board..."
    arduino-cli upload -p $PORT --fqbn $FQBN .
    
    if [ $? -eq 0 ]; then
        echo "Upload successful!"
        echo "Opening serial monitor..."
        arduino-cli monitor -p $PORT
    else
        echo "Upload failed!"
    fi
else
    echo "Compilation failed!"
fi