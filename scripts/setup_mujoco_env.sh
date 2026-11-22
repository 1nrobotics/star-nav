#!/usr/bin/env bash
# Creates mujoco_sim/.venv with MuJoCo and the simulator's dependencies.
#
# The simulator needs Python >= 3.11 (MuJoCo 4). ROS Noetic keeps using the
# system Python 3.8; the two only talk over ZeroMQ.
#
#   ./scripts/setup_mujoco_env.sh                 # pick a Python >= 3.11
#   PYTHON=/path/to/python3.12 ./scripts/setup_mujoco_env.sh
#   ./scripts/setup_mujoco_env.sh --lite3         # also the Lite3 quadruped:
#                                                 # onnxruntime, and its model
#                                                 # and policy (downloaded, BSD-3)
#   ./scripts/setup_mujoco_env.sh --t1            # also the T1 humanoid: its
#                                                 # model and policy
#                                                 # (downloaded, Apache-2.0)
#   ./scripts/setup_mujoco_env.sh --mid360        # also Livox's Mid-360 beam
#                                                 # table (downloaded, MIT)

set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${repo}/mujoco_sim/.venv"
extras="test"
lite3=false
t1=false
mid360=false
for a in "$@"; do
  case "${a}" in
    --lite3) lite3=true; extras="${extras},lite3" ;;
    --t1) t1=true ;;
    --mid360) mid360=true ;;
    *) echo "unknown option: ${a}" >&2; exit 2 ;;
  esac
done

find_python() {
  local c
  for c in "${PYTHON:-}" python3.13 python3.12 python3.11 python3; do
    [[ -n "${c}" ]] || continue
    if command -v "${c}" >/dev/null 2>&1 &&
       "${c}" -c 'import sys; sys.exit(sys.version_info < (3, 11))' 2>/dev/null; then
      command -v "${c}"
      return 0
    fi
  done
  return 1
}

if ! python="$(find_python)"; then
  echo "No Python >= 3.11 found. Install one (e.g. 'sudo apt install python3.11 python3.11-venv'" >&2
  echo "on Ubuntu with the deadsnakes PPA, or use 'uv python install 3.12') and set PYTHON=..." >&2
  exit 1
fi

echo "Using ${python} ($("${python}" --version))"
"${python}" -m venv "${venv}"
"${venv}/bin/pip" install --quiet --upgrade pip
"${venv}/bin/pip" install --quiet -e "${repo}/mujoco_sim[${extras}]"
"${venv}/bin/python" -c 'import mujoco, zmq; print("mujoco", mujoco.__version__, "pyzmq", zmq.__version__)'
if [[ "${lite3}" == true ]]; then
  "${venv}/bin/python" -m star_nav_mujoco --fetch-lite3
fi
if [[ "${t1}" == true ]]; then
  "${venv}/bin/python" -m star_nav_mujoco --fetch-t1
fi
if [[ "${mid360}" == true ]]; then
  "${venv}/bin/python" -m star_nav_mujoco --fetch-mid360
fi
echo "Ready: ${venv}"
