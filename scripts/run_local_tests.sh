#!/usr/bin/env bash
# Runs the project's tests on this machine (native ROS Noetic, no Docker).
#
# These are the checks the sim-smoke GitHub workflow ran on pull requests,
# plus the walking robots, which need downloaded files and never ran there.
# Run it before pushing; it exits non-zero when a step fails.
#
#   ./scripts/run_local_tests.sh                     # everything
#   ./scripts/run_local_tests.sh common aerial       # groups
#   ./scripts/run_local_tests.sh lint pytest         # only these steps
#   ./scripts/run_local_tests.sh --repeat 10 t1      # how often a mission fails
#
# The steps are grouped by the kind of robot they test (docs/TESTING.md has
# the procedure and what each step must show to pass):
#
# common: every robot
#   lint     bash -n on every shell script
#   build    catkin_make
#   launch   planner interface: the planner each robot's launch file starts
#   pytest   simulator unit tests (mujoco_sim/tests)
# aerial: the quadrotor, flying with EGO-Planner
#   unit     EGO's map unit tests (plan_env)
#   sim      sim_smoke.test: lightweight simulator
#   mujoco   mujoco_smoke.test in MuJoCo: depth camera (mujoco-depth), then
#            lidar (mujoco-lidar); either name runs that one alone
#   mujoco-frontier  mujoco_frontier_smoke.test: the quadrotor maps the
#            area around its start without waypoints
# legged: the Lite3 and the T1, walking with the CMU planner
#   lite3    lite3_smoke.test: Lite3 in the house
#   t1       t1_smoke.test: T1 in the house
#   frontier frontier map unit tests, then frontier_smoke.test: the Lite3
#            maps the house without waypoints
#
# --repeat N runs each mission test N times and counts the passes, to measure
# a failure that does not happen in every run. The other steps run once.
#
# Needs: the catkin workspace this repository is the src/ of, and
# ./scripts/setup_mujoco_env.sh (pytest, mujoco, legged). The legged missions
# are skipped when the robot is not downloaded (setup_mujoco_env.sh --lite3
# --t1) or the CMU planner's paths are not (fetch_cmu_paths.sh). Logs:
# ${STAR_NAV_TEST_LOG_DIR:-/tmp/star_nav_tests}.

set -uo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ws="$(cd "${repo}/.." && pwd)"
logs="${STAR_NAV_TEST_LOG_DIR:-/tmp/star_nav_tests}"
cache="${HOME}/.cache/star_nav"
python="${MUJOCO_SIM_PYTHON:-${repo}/mujoco_sim/.venv/bin/python}"
mkdir -p "${logs}"

common=(lint build launch pytest)
aerial=(unit sim mujoco mujoco-frontier)
legged=(lite3 t1 frontier)

repeat=1
steps=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --repeat)
      repeat="${2:-}"
      if [[ ! "${repeat}" =~ ^[1-9][0-9]*$ ]]; then
        echo "--repeat needs a number of runs." >&2
        exit 2
      fi
      shift
      ;;
    -h|--help)
      sed -n '2,/^$/p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    common) steps+=("${common[@]}") ;;
    aerial) steps+=("${aerial[@]}") ;;
    legged) steps+=("${legged[@]}") ;;
    *) steps+=("$1") ;;
  esac
  shift
done
if [[ ${#steps[@]} -eq 0 ]]; then
  steps=("${common[@]}" "${aerial[@]}" "${legged[@]}")
fi

# group_of <step>: the kind of robot it tests
group_of() {
  case "$1" in
    unit|sim|mujoco|mujoco-depth|mujoco-lidar|mujoco-frontier) echo aerial ;;
    lite3|t1|frontier) echo legged ;;
    *) echo common ;;
  esac
}

group=common
results=()
result_groups=()
failed=0

# record <name> <status> [note]
record() {
  results+=("$(printf '%-22s %s %s' "$1" "$2" "${3:-}")")
  result_groups+=("${group}")
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

# fall_in <log>: the simulator's line about a walking robot's fall, if any
fall_in() {
  grep -m1 -o '[a-z0-9]* fell at t=[0-9.]*' "$1" 2>/dev/null
}

# smoke <name> <test file> [args...]: a mission test, ${repeat} times
smoke() {
  local name="$1" i log passed=0 falls=0 fall note
  shift
  ros_setup || { record "${name}" FAIL "(ROS environment)"; return; }
  for ((i = 1; i <= repeat; i++)); do
    log="${logs}/${name}.log"
    if [[ ${repeat} -gt 1 ]]; then
      log="${logs}/${name}.${i}.log"
      echo "== ${name}, run ${i} of ${repeat}: rostest star_nav_bringup $*"
    else
      echo "== ${name}: rostest star_nav_bringup $*"
    fi
    if ROS_LOG_DIR="${logs}/${name}.ros" rostest star_nav_bringup "$@" >"${log}" 2>&1; then
      passed=$((passed + 1))
    else
      tail -n 30 "${log}"
      fall="$(fall_in "${log}")"
      if [[ -n "${fall}" ]]; then falls=$((falls + 1)); fi
    fi
  done
  if [[ ${repeat} -gt 1 ]]; then
    note="${passed}/${repeat} passed"
    if [[ ${falls} -gt 0 ]]; then note+=", ${falls} with a fall"; fi
    if [[ ${passed} -eq ${repeat} ]]; then
      record "${name}" PASS "(${note})"
    else
      record "${name}" FAIL "(${note}; ${logs}/${name}.*.log)"
    fi
  elif [[ ${passed} -eq 1 ]]; then
    record "${name}" PASS
  else
    record "${name}" FAIL "(${fall:+${fall}; }${log})"
  fi
}

# downloaded <name> <robot>: whether a legged mission can run; if not, it is
# recorded as skipped
downloaded() {
  if [[ ! -d "${cache}/$2" ]]; then
    record "$1" SKIP "(not downloaded: ./scripts/setup_mujoco_env.sh --$2)"
    return 1
  fi
  if [[ ! -f "${STAR_NAV_CMU_PATHS_DIR:-${cache}/cmu_paths}/correspondences.txt" ]]; then
    record "$1" SKIP "(no paths: ./scripts/fetch_cmu_paths.sh)"
    return 1
  fi
}

for step in "${steps[@]}"; do
  group="$(group_of "${step}")"
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
    mujoco-depth)
      smoke mujoco-depth mujoco_smoke.test
      ;;
    mujoco-lidar)
      smoke mujoco-lidar mujoco_smoke.test sensor:=lidar
      ;;
    mujoco-frontier)
      smoke mujoco-frontier mujoco_frontier_smoke.test
      ;;
    lite3|t1)
      if downloaded "${step}" "${step}"; then
        smoke "${step}" "${step}_smoke.test"
      fi
      ;;
    frontier)
      run frontier-unit python3 -B "${repo}/star_nav_bringup/test/test_frontier_map.py"
      if downloaded frontier lite3; then
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
for group in common aerial legged; do
  shown=0
  for i in "${!results[@]}"; do
    if [[ "${result_groups[$i]}" != "${group}" ]]; then continue; fi
    if [[ ${shown} -eq 0 ]]; then echo "${group}"; shown=1; fi
    echo "  ${results[$i]}"
  done
done
exit ${failed}
