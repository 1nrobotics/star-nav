#!/bin/bash

# Star Nav Release Deployment Script
# Deployment script for production/release builds (no source code mounts)

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Release Deployment${NC}"
echo -e "${YELLOW}⚠️  Using release build (no source code volume mounts)${NC}"

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set default environment variables
export IMAGE_TAG=${IMAGE_TAG:-release}
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
if ! docker images | grep -q "${IMAGE_NAME}.*${IMAGE_TAG}"; then
    echo "Release image not found. Building..."
    echo "Using repository root: ${REPO_ROOT}"
    cd "${REPO_ROOT}"
    docker build --build-arg RELEASE_BUILD=true -t ${IMAGE_NAME}:${IMAGE_TAG} .
    echo -e "${GREEN}✓ Build complete - source code removed from image${NC}"
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
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e ROS_HOSTNAME=localhost \
    -e ROS_IP=localhost \
    -e VEHICLE_ID=${VEHICLE_ID} \
    -e DISPLAY=${DISPLAY} \
    ${IMAGE_NAME}:${IMAGE_TAG} \
    bash -c "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_autonomous_flight.launch use_rviz:=false ${DOCKER_LAUNCH_ARGS}"

echo ""
echo -e "${GREEN}✅ Release deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav         # View logs"
echo "  docker exec -it star_nav bash   # Access container"
echo "  docker stop star_nav            # Stop service"
echo "  docker rm star_nav              # Remove container"
echo ""
echo -e "${YELLOW}Note: This is a release build - source code is NOT mounted from host${NC}"
