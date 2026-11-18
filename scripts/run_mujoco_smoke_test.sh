#!/usr/bin/env bash
# Headless end-to-end test with the MuJoCo simulator, in Docker.
#
# Starts a fresh simulator container, runs the exploration mission against it
# and checks waypoints, coverage, that the quadrotor never collided, and the
# sensor data and planner map against the true world.
#
#   ./scripts/run_mujoco_smoke_test.sh                # depth camera
#   SENSOR=lidar ./scripts/run_mujoco_smoke_test.sh   # 360-degree lidar

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

SENSOR="${SENSOR:-depth}"
export MUJOCO_ARGS="--sensor ${SENSOR} ${MUJOCO_ARGS:-}"
export MUJOCO_TEST_ARGS="sensor:=${SENSOR} ${MUJOCO_TEST_ARGS:-}"

docker compose build mujoco test-mujoco
docker compose up -d --force-recreate mujoco
status=0
docker compose run --rm test-mujoco || status=$?
if [[ ${status} -ne 0 ]]; then
  docker compose logs --tail 50 mujoco
fi
docker compose stop mujoco
exit ${status}
