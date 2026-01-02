#!/bin/bash

# Star Nav Onboard Startup Script
# Quick launch script for onboard deployment

set -e

# Get script directory
SCRIPT_DIR=$(dirname "$(realpath "$0")")
WORKSPACE_ROOT="$(realpath "$SCRIPT_DIR/..")"

# VK180 SSH configuration
VK180_HOST=10.42.0.64
VK180_USER=compulab
VK180_PASS=compulab

# PX4 Serial device
DEVICE=/dev/ttyUSB0
FCU_BAUD=921600

# Docker image
IMAGE_TAG=local
IMAGE_NAME=1nrobotics/star-nav

echo "Star Nav onboard environment loaded"

# Set system time
# echo "[*] Setting system time..."
# sudo date -s "2025-12-22 22:50:00"

# Check VK180 connectivity
echo "[*] Checking VK180 connectivity..."
if ! ping -c 1 -W 2 $VK180_HOST &> /dev/null; then
    echo "ERROR: VK180 device at $VK180_HOST is not reachable!"
    echo "Make sure VK180 is powered on and connected to the network"
    exit 1
fi
echo "    VK180 is reachable at $VK180_HOST"

# Start VK180 camera driver and VIO in tmux sessions
echo "[*] Starting VK180 camera driver and VIO..."

# Check if sshpass is installed for password authentication
if ! command -v sshpass &> /dev/null; then
    echo "ERROR: sshpass not installed (needed for automated SSH login)"
    echo "Install with: sudo apt-get install sshpass"
    exit 1
fi

# Start camera driver in tmux session
sshpass -p "$VK180_PASS" ssh -o StrictHostKeyChecking=no $VK180_USER@$VK180_HOST "tmux kill-session -t vk_camera 2>/dev/null || true"
sshpass -p "$VK180_PASS" ssh -o StrictHostKeyChecking=no $VK180_USER@$VK180_HOST "tmux new-session -d -s vk_camera '/opt/vilota/bin/vk_camera_driver /opt/vilota/configs/camera_driver/vk180-pro_light_rectified.json'"
echo "    Camera driver started in tmux session 'vk_camera'"

# Wait a moment for camera driver to initialize
sleep 2

# Start VIO in tmux session
sshpass -p "$VK180_PASS" ssh -o StrictHostKeyChecking=no $VK180_USER@$VK180_HOST "tmux kill-session -t vk_vio 2>/dev/null || true"
sshpass -p "$VK180_PASS" ssh -o StrictHostKeyChecking=no $VK180_USER@$VK180_HOST "tmux new-session -d -s vk_vio '/opt/vilota/bin/vk_vio /opt/vilota/configs/vio/vk180-pro_fast_rectified.json'"
echo "    VIO started in tmux session 'vk_vio'"

# Wait for VIO to initialize
sleep 2

echo "[*] VK180 services started successfully"
echo "    To view camera driver: sshpass -p '$VK180_PASS' ssh $VK180_USER@$VK180_HOST -t 'tmux attach -t vk_camera'"
echo "    To view VIO: sshpass -p '$VK180_PASS' ssh $VK180_USER@$VK180_HOST -t 'tmux attach -t vk_vio'"

# Check for serial device
if [ ! -e "$DEVICE" ]; then
    echo "ERROR: Serial device $DEVICE not found!"
    echo "Available devices:"
    ls -l /dev/tty{USB,ACM}* 2>/dev/null || echo "  No USB serial devices found"
    exit 1
fi

echo "Starting Star Nav with device: $DEVICE"
cd "$WORKSPACE_ROOT"
# ./docker/local-deploy.sh
