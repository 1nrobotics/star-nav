# Testing

How Star Nav is tested: the two kinds of robot and what each one runs, the
procedure, what every step must show to pass, and how a failure that does not
happen in every run is judged.

Tests run on your machine, with one script:

```bash
./scripts/run_local_tests.sh
```

Nothing runs on GitHub when you push ([CONTRIBUTING.md](../CONTRIBUTING.md)).

## The two kinds of robot

Every robot is of one type, and the type decides its planner
([PLANNER_INTERFACE.md](PLANNER_INTERFACE.md)). Code, tests, failures and
notes belong to one type or to both:

| | `aerial` | `legged` |
|---|---|---|
| Robots | quadrotor | Lite3 quadruped, T1 humanoid |
| Planner | EGO-Planner (`planner/ego/`) | CMU local planner, ground version (`planner/cmu/`) |
| Planner's nodes | `ego_planner_node`, `traj_server` | `terrainAnalysis`, `localPlanner`, `pathFollower`, `cmu_waypoint_router` |
| Command | position setpoints | velocity |
| Mission | `exploration_manager` (waypoints), or `frontier_explorer.py` (`explore:=frontier`, MuJoCo) | `exploration_manager` (waypoints), or `frontier_explorer.py` (`explore:=frontier`) |
| Simulator | lightweight (`uav_simulator/`), or MuJoCo | MuJoCo |
| Obstacle sensor | depth camera or lidar | lidar |
| Launch files | `exploration_sim.launch`, `exploration_mujoco.launch` | `exploration_lite3.launch`, `exploration_t1.launch` (both include `exploration_ground.launch`) |
| Status page | [QUADROTOR.md](QUADROTOR.md) | [QUADRUPED.md](QUADRUPED.md), [HUMANOID.md](HUMANOID.md) |
| Test group | `aerial` | `legged` |

A legged robot's launch file starts no EGO node, and the quadrotor's no CMU
node; the `launch` step checks this. So a fault in EGO can only fail the
`aerial` group, and a fault in the CMU planner or in a walking policy only
the `legged` group.

Both types share the exploration manager, the frontier explorer
(`frontier_explorer.py`, `frontier_map.py`), the planner interface
(`launch/planner.xml`, `planning_stack.xml`), the MuJoCo simulator and its
bridge (`mujoco_sim/`, `mujoco_bridge/`), `odom_visualization` and the mission
tests themselves (`star_nav_bringup/test/sim_smoke_test.py`,
`frontier_smoke_test.py`).

## Steps

| Group | Step | What runs | Time |
|---|---|---|---|
| `common` | `lint` | `bash -n` on every shell script | seconds |
| | `build` | `catkin_make` | seconds when little changed |
| | `launch` | `test_planner_interface.py`: the planner each robot's launch file starts, and that it is given the robot's sizes (`config/robots/<robot>.yaml`) | seconds |
| | `pytest` | `mujoco_sim/tests`: the simulator, every robot; that each robot's description file is current | 20 s |
| `aerial` | `unit` | EGO's map unit tests (`plan_env`) | seconds |
| | `sim` | `sim_smoke.test`: the mission in the lightweight simulator | 1 min |
| | `mujoco` | `mujoco_smoke.test` in MuJoCo, twice: depth camera (`mujoco-depth`), lidar (`mujoco-lidar`) | 1 min each |
| | `mujoco-frontier` | `mujoco_frontier_smoke.test`: the quadrotor maps the area round its start without waypoints | 40 s |
| `legged` | `lite3` | `lite3_smoke.test`: the Lite3 walks the mission through the house | 3 min |
| | `t1` | `t1_smoke.test`: the T1 walks the mission through the house | 3 min |
| | `frontier` | `test_frontier_map.py` (`frontier-unit`: the map, and the explorer's rule for a robot that stands still), then `frontier_smoke.test`: the Lite3 maps the house without waypoints | 2 min |

The times are from one machine (2026-11-26, `mujoco-frontier` 2026-11-27);
everything takes about a quarter of an hour there.

```bash
./scripts/run_local_tests.sh                    # everything
./scripts/run_local_tests.sh common aerial      # groups
./scripts/run_local_tests.sh lint mujoco-lidar  # single steps
./scripts/run_local_tests.sh --repeat 10 t1     # a mission, 10 times
./scripts/run_local_tests.sh --help
```

The summary lists the steps under their group. Logs are in
`/tmp/star_nav_tests` (`STAR_NAV_TEST_LOG_DIR`), one file per step.

## Procedure

1. **Once**: build the workspace, `./scripts/setup_mujoco_env.sh --lite3 --t1`
   and `./scripts/fetch_cmu_paths.sh`. Leave any conda or virtual
   environment: ROS needs the system Python.
2. **While you work**: run `common` and the group of the code you change.

   | Change in | Run |
   |---|---|
   | `planner/ego/`, `launch/planners/ego.xml`, `exploration_sim.launch`, `exploration_mujoco.launch`, `uav_simulator/` (not `odom_visualization`), `px4ctrl/` | `common aerial` |
   | `planner/cmu/`, `launch/planners/cmu.xml`, `scripts/planners/cmu/`, `exploration_ground.launch`, `exploration_lite3.launch`, `exploration_t1.launch` | `common legged` |
   | `planner/exploration_manager/`, `frontier_explorer.py`, `frontier_map.py`, `launch/planner.xml`, `planning_stack.xml`, `mujoco_sim/`, `mujoco_bridge/`, `star_nav_bringup/test/`, `odom_visualization`, `utils/` | everything |
   | Shell scripts | `lint` |
   | Documentation, `site/` | nothing; `launch` when launch commands changed |

3. **Before you push**: run everything. The push is good when no step says
   `FAIL` and nothing of the type you changed says `SKIP`.
4. **A step failed**: read its log and decide what it is (see
   [Failures that do not happen every time](#failures-that-do-not-happen-every-time)).
   Do not run it again until it passes without knowing why it failed.
5. **In the pull request**: paste the summary. For a step that was run
   again, give both results and the reason.

Without a native ROS installation, Docker covers the quadrotor only:
`./scripts/run_smoke_test.sh` (`sim`) and `./scripts/run_mujoco_smoke_test.sh`
(`mujoco-depth`; `SENSOR=lidar` for `mujoco-lidar`). `mujoco-frontier` and
the legged group need the native workspace.

## What a step must show

| Result | Meaning |
|---|---|
| `PASS` | Every check below held. With `--repeat N`: in all N runs. |
| `FAIL` | A check did not hold, or the step could not run (no ROS, no simulator environment). |
| `SKIP` | A legged mission whose robot or path set is not downloaded. Not a pass. |

`lint`, `build`, `unit`, `launch`, `pytest` and `frontier-unit` pass when
every script parses, the build succeeds and every test case passes.

### The waypoint mission

`sim`, `mujoco-depth`, `mujoco-lidar`, `lite3` and `t1` run the same mission
and the same test (`sim_smoke_test.py`): 12 waypoints over 9 m x 9 m, lines
3 m apart. Times are simulated time in MuJoCo.

| Check | `sim` | `mujoco-depth`, `mujoco-lidar` | `lite3`, `t1` |
|---|---|---|---|
| The mission reaches `FINISH` within | 180 s | 180 s | 420 s |
| Waypoints started | 12 of 12 | 12 of 12 | 12 of 12 |
| Waypoints that timed out | 0 (30 s each, reached within 0.5 m) | 0 (30 s, 0.5 m) | 0 (60 s, 0.3 m) |
| Path's extent in x and in y | 80 % of the area's side (7.2 m) | 7.2 m | 7.2 m |
| Contacts with obstacles (`/mujoco/contacts`) | not measured | 0 | 0 |
| Highest point of the body | | | house: Lite3 0.5 m, T1 0.8 m (it got onto the 0.21 m platform) |
| EGO's map against the world | | see below | |

Timeouts are read from `/rosout` (rostest starts no `rosout` node, so
`/rosout_agg` stays empty). For a walking robot, the ground and its feet on
the ramp, the platform and the steps are not contacts. A fall is not a check of its own: the robot
stops, so the waypoints after it time out. The script reads the simulator's
line about the fall from the log and puts it in the summary
(`t1 FAIL (t1 fell at t=76.18; ...)`).

The quadrotor in MuJoCo is also checked against the simulator's obstacles
(`map_accuracy.py`; [SENSORS.md](SENSORS.md#how-accuracy-is-checked)):

| Check | `mujoco-depth` | `mujoco-lidar` |
|---|---|---|
| Occupied voxels of the inflated map farther than 0.45 m from any obstacle | 1 % at most | 1 % at most |
| Lidar points farther than 2 cm from any surface | | 0.1 % at most |
| Pillars within 2.5 m of the quadrotor that are in the map | | 95 % at least |

### Frontier exploration

No waypoints are given; both steps run the same test
(`frontier_smoke_test.py`). `frontier` (legged) runs `frontier_smoke.test`:
the Lite3 maps the house. `mujoco-frontier` (aerial) runs
`mujoco_frontier_smoke.test`: the quadrotor maps the pillar forest round its
start.

| Check | `frontier` | `mujoco-frontier` |
|---|---|---|
| The mission reaches `FINISH` within | 600 s | 240 s |
| Frontiers given up on a timeout, way home not finished, explorer's time limit reached | none | none |
| Share of the area that is known | 95 % at least, of the floor inside the walls (10.8 m x 10.8 m) | 95 % at least, of the exploration area with its margin (12 m x 12 m) |
| Distance from the start at the end | 0.6 m at most | 0.6 m at most |
| Contacts with obstacles | 0 | 0 |

The start is where the robot is just before the trigger. (Until 2026-11-27
the test took it when it first saw `EXEC_FRONTIER`, up to a second later: a
walking robot has hardly moved by then, the quadrotor has flown 0.8 m.)

### Changing a check

The numbers are parameters of the `.test` files and defaults of the two test
scripts. A change that loosens one is a change of what the project promises:
say so in the pull request, with the measurement behind it, and update the
tables above.

## Failures that do not happen every time

A mission runs a planner, a simulator and, for the walking robots, a learned
policy; some faults show in one run of ten. They are recorded here with the
sign they leave, so that a failed step can be told apart from a new fault.

| Type | Steps | Fault | Sign in the log | How often | State |
|---|---|---|---|---|---|
| `legged` | `t1` | The T1 falls going down the house's 7 cm steps | `t1 fell at t=...`, then every later waypoint times out | 17 of 120 runs of the simulator alone; 2 of 10 runs of `t1` (both about 75 s into the mission) | Open; [HUMANOID.md](HUMANOID.md#limitations) |
| `legged` | `lite3` | The Lite3 touches the north end of the corridor's west wall with a front leg, turning round next to it on the way to the last waypoint | `Contact at t=...: lite3/FR_FOOT... / world:geom7(box size=[0.05 2.5  0.5 ]) at [0.65 1.8 ...]` about 130 s into the mission; all 12 waypoints are still reached, and the test fails with `the robot collided with an obstacle` | 1 of 11 runs of `lite3` (2026-11-27) | Open; [QUADRUPED.md](QUADRUPED.md#limitations) |
| `legged` | `frontier` | The Lite3 touches the cabinet with a leg, turning round next to it when the next frontier lies behind it | `Contact at t=...: world:geom13(box size=[0.2 0.5 0.4]) / lite3/FR_SHANK... at [ 3.7 -2.5 ...]` about 43 s into the mission; the map is still finished, and the test fails with `the robot collided with an obstacle` | 1 of 76 runs of `frontier` (2026-11-27); 1 of 54 runs of that change of goal alone, from a script | Open; the same cause as the row above; [QUADRUPED.md](QUADRUPED.md#limitations) |
| `legged` | `frontier` | A frontier is given up: on the way to it the Lite3 stood still for 6 s, or turned on the spot and was taken to stand still | `[frontier] TIMEOUT: gave up frontier (...)` once, 7 s after that frontier was chosen; the map is still finished, and the test fails with `exploration reported: [...]` | 1 of 56 runs of `frontier` (2026-11-27); not in 50 further runs with a bag. Since the explorer does not take a turn for standing still (2026-11-27): 0 of 20 | Open: the cause of that run is not known, and 168 runs without it are needed; [QUADRUPED.md](QUADRUPED.md#limitations) |

**Judging a failed step**

1. No sign from the table in the log: it is a new fault. Find the cause; do
   not push.
2. A sign from the table, and the change touches nothing that type of robot
   runs (first table of this page): run the step once more. If it passes,
   push, and report both runs in the pull request. If it fails again, treat
   it as a new fault.
3. A sign from the table, and the change does touch that type: one more run
   proves nothing. Measure the rate (`--repeat`) and compare it with the
   table's.

**Measuring a rate**

```bash
./scripts/run_local_tests.sh --repeat 20 t1
```

runs the mission 20 times, keeps each run's log (`t1.1.log` ...) and reports
`17/20 passed, 3 with a fall`. When the fault shows early in the mission, a
shorter scenario gives more runs in the same time; describe it in the pull
request so that it can be repeated.

**Claiming a fix**

A fault that showed in `k` of `n` runs is taken as fixed when at least
`3 n / k` runs of the same scenario with the fix show none. (With no failure
in `N` runs, the rate is below `3 / N` with 95 % confidence.) For 17 of 120
that is 22 runs. EGO's hang at the mission's first corner
([QUADROTOR.md](QUADROTOR.md#planner-faults)) showed in 3 of 39 runs: 39
were needed, and 80 were made. Then remove the row from the table and
record the fix on the robot's status page.

**A new fault** that cannot be fixed at once gets a row: its type, the steps
it fails, the sign in the log and the measured rate.
