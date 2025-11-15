#!/bin/bash

echo "Starting all sensor drivers..."

gnome-terminal -- bash -c "python3 drivers/gps_driver.py; exec bash" &
sleep 1

gnome-terminal -- bash -c "python3 drivers/imu_driver.py; exec bash" &
sleep 1

gnome-terminal -- bash -c "python3 drivers/camera_driver.py; exec bash" &

echo "All drivers started in separate terminals"
echo "Check each terminal for status"
