#!/bin/bash

# Star Nav Remote Visualization Deployment Script
# Deployment script that allows RViz on remote computer

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Remote Visualization Deployment${NC}"
echo -e "${YELLOW}⚠️  Configuring for remote RViz access${NC}"

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Get onboard computer's IP address
ONBOARD_IP=${ONBOARD_IP:-$(hostname -I | awk '{print $1}')}
echo "Detected onboard IP: ${ONBOARD_IP}"
echo "Remote computer should connect to: http://${ONBOARD_IP}:11311"

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-local}
export IMAGE_NAME=${IMAGE_NAME:-1nrobotics/star-nav}
export DRONE_ID=${DRONE_ID:-0}
export VEHICLE_ID=${VEHICLE_ID:-1}
export DISPLAY=${DISPLAY}

# Repository root (default: one level up from this script's directory)
SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_ROOT=${REPO_ROOT:-$(realpath "$SCRIPT_DIR/..")}

# Stop any running containers
echo "Stopping existing containers..."
docker stop star_nav 2>/dev/null || true
docker rm star_nav 2>/dev/null || true

# Check if image exists, if not build it
if ! docker images | grep -q "${IMAGE_NAME}"; then
    echo "Image not found. Building..."
    echo "Using repository root: ${REPO_ROOT}"
    cd "${REPO_ROOT}"
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
fi

# Start the service
echo "Starting Star Nav service..."

# Detect serial device to pass through to container (override with DEVICE env)
DEVICE=${DEVICE:-/dev/ttyUSB0}
DOCKER_DEVICE_ARGS=""
DOCKER_LAUNCH_ARGS=""
if [ -e "${DEVICE}" ]; then
    echo "Found serial device on host: ${DEVICE} — passing through to container"
    DOCKER_DEVICE_ARGS="--device ${DEVICE}:${DEVICE}"
    FCU_BAUD=${FCU_BAUD:-921600}
    DOCKER_LAUNCH_ARGS="fcu_url:=${DEVICE}:${FCU_BAUD}"
else
    echo "No serial device '${DEVICE}' found on host — starting without passthrough (use DEVICE env to override)"
fi

docker run -d \
    --name star_nav \
    --network host \
    --privileged \
    ${DOCKER_DEVICE_ARGS} \
    -v "${REPO_ROOT}/px4ctrl":/catkin_ws/src/px4ctrl:rw \
    -v "${REPO_ROOT}/planner":/catkin_ws/src/planner:rw \
    -v "${REPO_ROOT}/vk180-ros":/catkin_ws/src/vk180-ros:rw \
    -v "${REPO_ROOT}/utils":/catkin_ws/src/utils:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e ROS_MASTER_URI=http://${ONBOARD_IP}:11311 \
    -e ROS_HOSTNAME=${ONBOARD_IP} \
    -e ROS_IP=${ONBOARD_IP} \
    -e VEHICLE_ID=${VEHICLE_ID} \
    -e DISPLAY=${DISPLAY} \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_autonomous_flight.launch use_rviz:=false ${DOCKER_LAUNCH_ARGS}"

echo ""
echo -e "${GREEN}✅ Remote visualization deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo -e "${BLUE}=== Remote Computer Setup Instructions ===${NC}"
echo ""
echo "On your remote computer (the one with the display), run these commands:"
echo ""
echo -e "${YELLOW}# Install ROS Noetic (if not already installed)${NC}"
echo "sudo apt-get install ros-noetic-desktop-full"
echo ""
echo -e "${YELLOW}# Set environment variables (add to ~/.bashrc)${NC}"
echo "export ROS_MASTER_URI=http://${ONBOARD_IP}:11311"
echo "export ROS_HOSTNAME=\$(hostname -I | awk '{print \$1}')"
echo "export ROS_IP=\${ROS_HOSTNAME}"
echo ""
echo -e "${YELLOW}# Source ROS and test connection${NC}"
echo "source /opt/ros/noetic/setup.bash"
echo "rostopic list"
echo ""
echo -e "${YELLOW}# Launch RViz${NC}"
echo "rosrun rviz rviz -d /path/to/vk180_planner_visualization.rviz"
echo ""
echo -e "${BLUE}=== Network Requirements ===${NC}"
echo "- Ensure both computers are on the same network"
echo "- Onboard computer IP: ${ONBOARD_IP}"
echo "- Firewall must allow port 11311 (ROS Master)"
echo "- Test connectivity: ping ${ONBOARD_IP}"
echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav         # View logs"
echo "  docker exec -it star_nav bash   # Access container"
echo "  docker stop star_nav            # Stop service"
echo "  docker rm star_nav              # Remove container"
