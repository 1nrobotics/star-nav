#!/usr/bin/env bash
# Creates mujoco_sim/.venv with MuJoCo and the simulator's dependencies.
#
# The simulator needs Python >= 3.11 (MuJoCo 4). ROS Noetic keeps using the
# system Python 3.8; the two only talk over ZeroMQ.
#
#   ./scripts/setup_mujoco_env.sh                 # pick a Python >= 3.11
#   PYTHON=/path/to/python3.12 ./scripts/setup_mujoco_env.sh
#   ./scripts/setup_mujoco_env.sh --microduck     # also the Microduck biped
#
# --microduck adds onnxruntime and the BAM servo model, and downloads the
# Microduck model and walking policy (not in this repository; the meshes are
# CC BY-NC-SA 4.0, see mujoco_sim/README.md) to ~/.cache/star_nav/microduck.

set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
venv="${repo}/mujoco_sim/.venv"

microduck=0
for arg in "$@"; do
  case "${arg}" in
    --microduck) microduck=1 ;;
    *) echo "Unknown option: ${arg}" >&2; exit 2 ;;
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
"${venv}/bin/pip" install --quiet -e "${repo}/mujoco_sim[test]"
"${venv}/bin/python" -c 'import mujoco, zmq; print("mujoco", mujoco.__version__, "pyzmq", zmq.__version__)'

if (( microduck )); then
  "${venv}/bin/pip" install --quiet -e "${repo}/mujoco_sim[test,microduck]"
  # BAM pins requires-python < 3.13 but is pure Python; install the commit the
  # walking policy was trained with.
  bam="$("${venv}/bin/python" -c 'from star_nav_mujoco.microduck import BAM_REQUIREMENT; print(BAM_REQUIREMENT)')"
  "${venv}/bin/pip" install --quiet --no-deps --ignore-requires-python "${bam}"
  "${venv}/bin/python" -m star_nav_mujoco --fetch-microduck
fi
echo "Ready: ${venv}"
