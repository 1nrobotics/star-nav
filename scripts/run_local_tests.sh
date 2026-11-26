#!/usr/bin/env bash
# Runs the project's tests on this machine (native ROS Noetic, no Docker).
#
# These are the checks the sim-smoke GitHub workflow ran on pull requests,
# plus the walking robots, which need downloaded files and never ran there.
# Run it before pushing; it exits non-zero when a step fails.
#
#   ./scripts/run_local_tests.sh                 # everything
#   ./scripts/run_local_tests.sh lint pytest     # only these steps
#
# Steps:
#   lint     bash -n on every shell script
#   build    catkin_make
#   unit     planner unit tests (plan_env)
#   launch   planner interface: the planner each robot's launch file starts
#   pytest   simulator unit tests (mujoco_sim/tests)
#   sim      sim_smoke.test: quadrotor, lightweight simulator
#   mujoco   mujoco_smoke.test: quadrotor in MuJoCo, depth camera, then lidar
#   lite3    lite3_smoke.test: Lite3 in the house (the CMU planner)
#   t1       t1_smoke.test: T1 in the house (the CMU planner)
#   frontier frontier map unit tests, then frontier_smoke.test: the Lite3
#            maps the house without waypoints
#
# Needs: the catkin workspace this repository is the src/ of, and
# ./scripts/setup_mujoco_env.sh (pytest, mujoco, lite3, t1). lite3 and t1 are
# skipped when the robot is not downloaded (setup_mujoco_env.sh --lite3 --t1)
# or the CMU planner's paths are not (fetch_cmu_paths.sh). Logs:
# ${STAR_NAV_TEST_LOG_DIR:-/tmp/star_nav_tests}.

set -uo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws="$(cd "${repo}/.." && pwd)"
logs="${STAR_NAV_TEST_LOG_DIR:-/tmp/star_nav_tests}"
cache="${HOME}/.cache/star_nav"
python="${MUJOCO_SIM_PYTHON:-${repo}/mujoco_sim/.venv/bin/python}"
mkdir -p "${logs}"

steps=("$@")
if [[ ${#steps[@]} -eq 0 ]]; then
  steps=(lint build unit launch pytest sim mujoco lite3 t1 frontier)
fi

results=()
failed=0

# record <name> <status> [note]
record() {
  results+=("$(printf '%-22s %s %s' "$1" "$2" "${3:-}")")
  if [[ "$2" == FAIL ]]; then failed=1; fi
}

# run <name> <command...>: runs it with its output in ${logs}/<name>.log
run() {
  local name="$1"
  shift
  echo "== ${name}: $*"
  if "$@" >"${logs}/${name}.log" 2>&1; then
    record "${name}" PASS
  else
    record "${name}" FAIL "(${logs}/${name}.log)"
    tail -n 30 "${logs}/${name}.log"
  fi
}

lint() {
  local f status=0
  while IFS= read -r f; do
    bash -n "${repo}/${f}" || status=1
  done < <(git -C "${repo}" ls-files --cached --others --exclude-standard '*.sh')
  return ${status}
}

ros_ready=0
ros_setup() {
  if [[ ${ros_ready} -eq 1 ]]; then return 0; fi
  if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    echo "ROS Noetic not found (/opt/ros/noetic)." >&2
    return 1
  fi
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  if [[ -f "${ws}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${ws}/devel/setup.bash"
  fi
  set -u
  if ! python3 -c 'import rospkg' 2>/dev/null; then
    echo "python3 ($(command -v python3)) cannot import rospkg: ROS needs the" >&2
    echo "system Python. Leave the conda or virtual environment first." >&2
    return 1
  fi
  ros_ready=1
}

# smoke <name> <test file> [args...]
smoke() {
  local name="$1"
  shift
  ros_setup || { record "${name}" FAIL "(ROS environment)"; return; }
  ROS_LOG_DIR="${logs}/${name}.ros" run "${name}" rostest star_nav_bringup "$@"
}

# ground <robot>: its smoke test (it walks with the CMU planner)
ground() {
  local robot="$1"
  if [[ ! -d "${cache}/${robot}" ]]; then
    record "${robot}" SKIP "(not downloaded: ./scripts/setup_mujoco_env.sh --${robot})"
  elif [[ ! -f "${STAR_NAV_CMU_PATHS_DIR:-${cache}/cmu_paths}/correspondences.txt" ]]; then
    record "${robot}" SKIP "(no paths: ./scripts/fetch_cmu_paths.sh)"
  else
    smoke "${robot}" "${robot}_smoke.test"
  fi
}

for step in "${steps[@]}"; do
  case "${step}" in
    lint)
      run lint lint
      ;;
    build)
      if ros_setup; then
        run build bash -c "cd '${ws}' && catkin_make"
        ros_ready=0
      else
        record build FAIL "(ROS environment)"
      fi
      ;;
    unit)
      if ros_setup; then
        run unit bash -c "cd '${ws}' && catkin_make run_tests_plan_env && catkin_test_results build/test_results/plan_env"
      else
        record unit FAIL "(ROS environment)"
      fi
      ;;
    launch)
      if ros_setup; then
        run launch python3 -B "${repo}/star_nav_bringup/test/test_planner_interface.py"
      else
        record launch FAIL "(ROS environment)"
      fi
      ;;
    pytest)
      if [[ -x "${python}" ]]; then
        run pytest "${python}" -m pytest "${repo}/mujoco_sim/tests" -q
      else
        record pytest FAIL "(no ${python}: ./scripts/setup_mujoco_env.sh)"
      fi
      ;;
    sim)
      smoke sim sim_smoke.test
      ;;
    mujoco)
      smoke mujoco-depth mujoco_smoke.test
      smoke mujoco-lidar mujoco_smoke.test sensor:=lidar
      ;;
    lite3)
      ground lite3
      ;;
    t1)
      ground t1
      ;;
    frontier)
      run frontier-unit python3 -B "${repo}/star_nav_bringup/test/test_frontier_map.py"
      if [[ ! -d "${cache}/lite3" ]]; then
        record frontier SKIP "(not downloaded: ./scripts/setup_mujoco_env.sh --lite3)"
      elif [[ ! -f "${STAR_NAV_CMU_PATHS_DIR:-${cache}/cmu_paths}/correspondences.txt" ]]; then
        record frontier SKIP "(no paths: ./scripts/fetch_cmu_paths.sh)"
      else
        smoke frontier frontier_smoke.test
      fi
      ;;
    *)
      echo "Unknown step: ${step}" >&2
      record "${step}" FAIL "(unknown step)"
      ;;
  esac
done

echo
echo "== Summary"
printf '%s\n' "${results[@]}"
exit ${failed}
