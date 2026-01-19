#!/bin/bash

# Star Nav Local Docker Deployment Script
# Quick script for local development deployment

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Local Deployment${NC}"

# Check if remote visualization is needed
ENABLE_REMOTE_VIZ=${ENABLE_REMOTE_VIZ:-false}

# Configure ROS networking
if [ "$ENABLE_REMOTE_VIZ" = "true" ]; then
    # Get onboard computer's IP address for remote visualization
    ONBOARD_IP=${ONBOARD_IP:-$(hostname -I | awk '{print $1}')}
    ROS_MASTER_URI="http://${ONBOARD_IP}:11311"
    ROS_HOSTNAME="${ONBOARD_IP}"
    ROS_IP="${ONBOARD_IP}"
    echo -e "${BLUE}Remote visualization ENABLED${NC}"
    echo "Onboard IP: ${ONBOARD_IP}"
    echo "Remote computer should connect to: ${ROS_MASTER_URI}"
else
    # Local only - use localhost
    ROS_MASTER_URI="http://localhost:11311"
    ROS_HOSTNAME="localhost"
    ROS_IP="localhost"
    echo -e "${BLUE}Local mode - remote visualization DISABLED${NC}"
fi

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-local}
export IMAGE_NAME=${IMAGE_NAME:-1nrobotics/star-nav}
export DISPLAY=${DISPLAY}
export REBUILD_IMAGE=${REBUILD_IMAGE:-false}

# Repository root (default: one level up from this script's directory)
SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_ROOT=${REPO_ROOT:-$(realpath "$SCRIPT_DIR/..")}

# Stop any running containers
echo "Stopping existing containers..."
docker stop star_nav 2>/dev/null || true
docker rm star_nav 2>/dev/null || true

# Handle image rebuild if requested
if [ "$REBUILD_IMAGE" = "true" ]; then
    echo -e "${BLUE}Rebuild requested - removing existing image...${NC}"
    docker rmi ${IMAGE_NAME}:${IMAGE_TAG} 2>/dev/null || true
fi

# Check if image exists, if not build it
if ! docker images | grep -q "${IMAGE_NAME}"; then
    echo "Image not found. Building..."
    echo "Using repository root: ${REPO_ROOT}"
    cd "${REPO_ROOT}"
    docker build -t ${IMAGE_NAME}:${IMAGE_TAG} .
elif [ "$REBUILD_IMAGE" = "true" ]; then
    echo "Rebuilding image..."
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
    -e ROS_MASTER_URI=${ROS_MASTER_URI} \
    -e ROS_HOSTNAME=${ROS_HOSTNAME} \
    -e ROS_IP=${ROS_IP} \
    -e DISPLAY=${DISPLAY} \
    -e DISABLE_ROS1_EOL_WARNINGS=1 \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash -c "source /catkin_ws/devel/setup.bash && tail -f /dev/null"
    # bash -c "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_autonomous_flight.launch use_rviz:=true ${DOCKER_LAUNCH_ARGS}"

echo ""
echo -e "${GREEN}✅ Local deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav         # View logs"
echo "  docker exec -it star_nav bash   # Access container"
echo "  docker stop star_nav            # Stop service"
echo "  docker rm star_nav              # Remove container"
