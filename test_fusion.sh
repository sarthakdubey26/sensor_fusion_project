#!/bin/bash

echo "================================"
echo "Testing Fusion System"
echo "================================"
echo ""

echo "Starting GPS driver..."
gnome-terminal -- bash -c "cd ~/sensor_fusion; python3 drivers/gps_driver.py; exec bash" &
sleep 2

echo "Starting IMU driver..."
gnome-terminal -- bash -c "cd ~/sensor_fusion; python3 drivers/imu_driver.py; exec bash" &
sleep 2

echo "Starting Fusion driver..."
gnome-terminal -- bash -c "cd ~/sensor_fusion; python3 drivers/fusion_driver.py; exec bash" &
sleep 2

echo ""
echo "All drivers running!"
echo "Check the terminal windows"
echo ""
echo "Test will run for 10 seconds..."
sleep 10

echo ""
echo "Stopping all drivers..."
pkill -f gps_driver.py
pkill -f imu_driver.py
pkill -f fusion_driver.py

echo ""
echo "Test complete!"
echo ""
echo "Check data file:"
ls -lh data/fusion_*.csv 2>/dev/null
echo ""
echo "View data:"
cat data/fusion_*.csv 2>/dev/null | head -10
