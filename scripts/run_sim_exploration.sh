#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-star_nav}"
USE_RVIZ="${USE_RVIZ:-true}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running."
  echo "Start it first with: ./docker/local-deploy.sh"
  exit 1
fi

docker exec -it "${CONTAINER_NAME}" bash -lc \
  "source /catkin_ws/devel/setup.bash && roslaunch star_nav_bringup exploration_sim.launch use_rviz:=${USE_RVIZ} $*"
