#!/usr/bin/env bash
# Launches the exploration simulation in the dev container.
#
#   ./scripts/run_sim_exploration.sh              # kinematic simulator
#   SIM=mujoco ./scripts/run_sim_exploration.sh   # MuJoCo simulator (viewer on :8080)
#   USE_RVIZ=false ./scripts/run_sim_exploration.sh
#
# Extra arguments are passed to roslaunch, e.g. exploration_width:=12.

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-star_nav}"
USE_RVIZ="${USE_RVIZ:-true}"
SIM="${SIM:-kinematic}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
  echo "Container '${CONTAINER_NAME}' is not running."
  echo "Start it first with: ./docker/local-deploy.sh"
  exit 1
fi

case "${SIM}" in
  kinematic)
    launch="exploration_sim.launch"
    ;;
  mujoco)
    cd "$(dirname "${BASH_SOURCE[0]}")/.."
    docker compose up -d --build mujoco
    echo "MuJoCo viewer: http://localhost:${MUJOCO_VIEWER_PORT:-8080}"
    launch="exploration_mujoco.launch start_sim:=false"
    ;;
  *)
    echo "Unknown SIM='${SIM}' (use kinematic or mujoco)." >&2
    exit 1
    ;;
esac

docker exec -it "${CONTAINER_NAME}" bash -lc \
  "source /catkin_ws/devel/setup.bash && roslaunch star_nav_bringup ${launch} use_rviz:=${USE_RVIZ} $*"
