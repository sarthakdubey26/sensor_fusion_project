#!/bin/bash

echo "================================"
echo "Starting Data Collection"
echo "================================"
echo ""
echo "This will record GPS, IMU, and Camera data"
echo "Walk your path, then press Ctrl+C to stop"
echo ""

mkdir -p data

FILENAME="data/recording_$(date +%Y%m%d_%H%M%S)"

echo "Recording to: $FILENAME"
echo ""
echo "Starting sensors in 3 seconds..."
sleep 3

ros2 bag record -o $FILENAME /gps/fix /imu/data /camera/image_raw

echo ""
echo "Recording saved!"
echo "File: $FILENAME"
