#!/usr/bin/env bash
# Headless end-to-end test with the MuJoCo simulator, in Docker.
#
# Starts a fresh simulator container, runs the exploration mission against it
# and checks waypoints, coverage and that the quadrotor never collided.

set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

docker compose build mujoco test-mujoco
docker compose up -d --force-recreate mujoco
status=0
docker compose run --rm test-mujoco || status=$?
if [[ ${status} -ne 0 ]]; then
  docker compose logs --tail 50 mujoco
fi
docker compose stop mujoco
exit ${status}
