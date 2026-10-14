#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-star_nav}"
USE_RVIZ="${USE_RVIZ:-false}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running."
  echo "Start it first with: ENABLE_VK180_SDK=true ./docker/local-deploy.sh"
  exit 1
fi

docker exec -it "${CONTAINER_NAME}" bash -lc \
  "source /catkin_ws/devel/setup.bash && roslaunch vk180_ros vk180_exploration_flight.launch use_rviz:=${USE_RVIZ} $*"
