#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-star_nav}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running."
  exit 1
fi

docker exec "${CONTAINER_NAME}" bash -lc \
  "source /catkin_ws/devel/setup.bash && rostopic pub -1 /exploration/trigger geometry_msgs/PoseStamped '{}'"
