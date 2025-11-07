#!/bin/bash

# Star Nav Onboard Startup Script
# Quick launch script for onboard deployment

set -e

# Get script directory
SCRIPT_DIR=$(dirname "$(realpath "$0")")
WORKSPACE_ROOT="$(realpath "$SCRIPT_DIR/..")"

# Star Nav Onboard Environment
export PLATFORM_TYPE="onboard"
export VK180_HOST=10.42.0.64
export VK180_HOSTNAME=compulab-imx8mp-01
export DEVICE=/dev/ttyUSB0
export FCU_BAUD=921600
export IMAGE_TAG=local
export IMAGE_NAME=1nrobotics/star-nav
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export ROS_IP=localhost
export VEHICLE_ID=1
export DRONE_ID=1
export USE_RVIZ=false
export TAKEOFF_HEIGHT=1.5
export MAX_VEL=1.0
export MAX_ACC=2.0
export MAVROS_RESPAWN=true

echo "Star Nav onboard environment loaded"

# Check for serial device
if [ ! -e "$DEVICE" ]; then
    echo "ERROR: Serial device $DEVICE not found!"
    echo "Available devices:"
    ls -l /dev/tty{USB,ACM}* 2>/dev/null || echo "  No USB serial devices found"
    exit 1
fi

echo "Starting Star Nav with device: $DEVICE"
cd "$WORKSPACE_ROOT"
./docker/local-deploy.sh
