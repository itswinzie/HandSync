#!/bin/bash
# ============================================================
# MasterPi Auto Startup Script
# Letak di: /home/pi/start_ros2.sh
# ============================================================

echo "[$(date)] Starting MasterPi ROS2..."

# 1. Stop masterpi.service supaya tak conflict serial port
echo "[$(date)] Stopping masterpi.service..."
systemctl stop masterpi.service
sleep 2

# 2. Start docker container
echo "[$(date)] Starting Docker container..."
docker start ros2_masterpi
sleep 3

# 3. Run subscriber dalam container (background)
echo "[$(date)] Starting subscriber..."
docker exec -d ros2_masterpi bash -c "
    source /opt/ros/humble/setup.bash &&
    export ROS_DOMAIN_ID=42 &&
    python3 /root/masterpi_subscriber.py
"
sleep 2

# 4. Run camera publisher dalam container (background)
echo "[$(date)] Starting camera publisher..."
docker exec -d ros2_masterpi bash -c "
    source /opt/ros/humble/setup.bash &&
    export ROS_DOMAIN_ID=42 &&
    python3 /root/masterpi_camera_pub.py
"

echo "[$(date)] MasterPi ROS2 started!"
echo "[$(date)] Subscriber + Camera Publisher running in background"
