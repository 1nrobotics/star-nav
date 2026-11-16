#!/usr/bin/env bash
# Starts the standalone MuJoCo simulator (mujoco_sim) from roslaunch.
#
# The simulator needs Python >= 3.11 with mujoco, numpy and pyzmq; it does not
# use ROS. The interpreter is taken from MUJOCO_SIM_PYTHON, else from the
# virtual environment created by scripts/setup_mujoco_env.sh.

set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "${here}/../.." && pwd)"
python="${MUJOCO_SIM_PYTHON:-${repo}/mujoco_sim/.venv/bin/python}"

if [[ ! -x "${python}" ]]; then
  echo "[mujoco_sim] Python not found at ${python}." >&2
  echo "[mujoco_sim] Run ./scripts/setup_mujoco_env.sh or set MUJOCO_SIM_PYTHON." >&2
  exit 1
fi

# Drop the __name:= / __log:= arguments added by roslaunch.
args=()
for a in "$@"; do
  [[ "${a}" == __*:=* ]] || args+=("${a}")
done

cd "${repo}/mujoco_sim"
exec "${python}" -u -m star_nav_mujoco "${args[@]}"
