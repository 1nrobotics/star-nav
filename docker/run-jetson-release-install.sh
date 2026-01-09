#!/bin/bash

# Star Nav Jetson Release Runner
# Builds and runs runtime Docker image from build_context_release

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚁 Star Nav - Jetson Release Runner${NC}"

# Get script directory and repo root
SCRIPT_DIR=$(dirname "$(realpath "$0")")
REPO_ROOT=$(realpath "$SCRIPT_DIR/..")
BUILD_CONTEXT_DIR="$REPO_ROOT/build_context_release"

# Docker image settings
IMAGE_NAME="${IMAGE_NAME:-1nrobotics/star-nav}"
IMAGE_TAG="${IMAGE_TAG:-runtime}"
FULL_IMAGE="${IMAGE_NAME}:${IMAGE_TAG}"

# Check if build context exists
if [ ! -d "$BUILD_CONTEXT_DIR" ]; then
    echo -e "${RED}Error: Build context not found at $BUILD_CONTEXT_DIR${NC}"
    echo "Please run './scripts/build_release_docker.sh' first to create the build context."
    exit 1
fi

# Build the runtime image
echo "Building runtime image from build context..."
cd "$BUILD_CONTEXT_DIR"
if [ ! -f "Dockerfile.runtime" ]; then
    echo -e "${RED}Error: Dockerfile.runtime not found in build context${NC}"
    exit 1
fi

docker build -t "${FULL_IMAGE}" -f Dockerfile.runtime .
echo -e "${GREEN}✓ Runtime image built successfully${NC}"

# Check if remote visualization is needed
ENABLE_REMOTE_VIZ=${ENABLE_REMOTE_VIZ:-true}
ONBOARD_IP=192.168.10.200

# Configure ROS networking
if [ "$ENABLE_REMOTE_VIZ" = "true" ]; then
    # Get onboard computer's IP address for remote visualization
    ONBOARD_IP=${ONBOARD_IP:-$(hostname -I | awk '{print $1}')}
    ROS_MASTER_URI="http://${ONBOARD_IP}:11311"
    ROS_HOSTNAME="${ONBOARD_IP}"
    ROS_IP="${ONBOARD_IP}"
    ROS_ENV_ARGS="-e ROS_MASTER_URI=${ROS_MASTER_URI} -e ROS_HOSTNAME=${ROS_HOSTNAME} -e ROS_IP=${ROS_IP}"
    echo -e "${BLUE}Remote visualization ENABLED${NC}"
    echo "Onboard IP: ${ONBOARD_IP}"
    echo "Remote computer should connect to: ${ROS_MASTER_URI}"
else
    # Local only - let roslaunch start its own master
    ROS_ENV_ARGS=""
    echo -e "${BLUE}Local mode - remote visualization DISABLED${NC}"
    echo "ROS master will be started by roslaunch"
fi

# Enable X11 forwarding for GUI applications
xhost +local:docker 2>/dev/null || true

# Set DISPLAY
export DISPLAY=${DISPLAY}

# Stop any running containers
echo "Stopping existing containers..."
docker stop star_nav 2>/dev/null || true
docker rm star_nav 2>/dev/null || true

# Start the service
echo "Starting Star Nav service..."

# Detect serial device to pass through to container (override with DEVICE env)
DEVICE=${DEVICE:-/dev/ttyUSB0}
DOCKER_DEVICE_ARGS=""
if [ -e "${DEVICE}" ]; then
    echo "Found serial device on host: ${DEVICE} — passing through to container"
    DOCKER_DEVICE_ARGS="--device ${DEVICE}:${DEVICE}"
else
    echo "No serial device '${DEVICE}' found on host — starting without passthrough (use DEVICE env to override)"
fi

docker run -d \
    --name star_nav \
    --network host \
    --privileged \
    ${DOCKER_DEVICE_ARGS} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ${ROS_ENV_ARGS} \
    -e DISPLAY=${DISPLAY} \
    -e DISABLE_ROS1_EOL_WARNINGS=1 \
    ${FULL_IMAGE} \
    bash -c "source /catkin_ws/install/setup.bash && tail -f /dev/null"

echo ""
echo -e "${GREEN}✅ Jetson release deployment completed!${NC}"
echo ""
echo "Container status:"
docker ps | grep star_nav

echo ""
echo "Useful commands:"
echo "  docker logs -f star_nav                    # View logs"
echo "  docker exec -it star_nav bash              # Access container (auto-sources setup)"
echo "  docker stop star_nav                       # Stop service"
echo "  docker rm star_nav                         # Remove container"
echo ""
echo "To launch exploration:"
echo "  docker exec -it star_nav bash -c 'roslaunch vk180_ros vk180_exploration.launch'"
echo ""
echo "To launch autonomous flight:"
echo "  docker exec -it star_nav bash -c 'roslaunch vk180_ros vk180_autonomous_flight.launch'"
echo ""
echo "Environment variables:"
echo "  ENABLE_REMOTE_VIZ=true/false    # Enable remote ROS visualization (default: true)"
echo "  DEVICE=/dev/ttyUSB0             # Serial device for PX4 (default: /dev/ttyUSB0)"
echo "  IMAGE_NAME=1nrobotics/star-nav  # Docker image name (default: 1nrobotics/star-nav)"
echo "  IMAGE_TAG=runtime               # Docker image tag (default: runtime)"
echo ""
echo -e "${YELLOW}Running runtime image: ${FULL_IMAGE}${NC}"
echo -e "${YELLOW}Built from: ${BUILD_CONTEXT_DIR}${NC}"
