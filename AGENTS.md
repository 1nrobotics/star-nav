# AGENTS.md

This repository is being prepared as an open-source branch. Any coding agent
working here should optimize for a clean public contributor experience.

## Priorities

1. Keep the simulation workflow easy to discover and easy to run.
2. Keep `main` hardware-independent; sensor-specific code belongs on
   `hardware/*` branches (see docs/HARDWARE.md).
3. Preserve and clarify upstream licensing instead of simplifying it away.
4. Prefer small, reviewable cleanup steps over broad rewrites.

## Repository Rules

- Do not remove upstream license files from subpackages.
- Do not assume the whole repo is MIT: this repo contains mixed licenses,
  including GPLv3 components from the planning/control stack.
- Keep root documentation public-facing and task-oriented.
- When changing launch or Docker behavior, verify the simulation path first.
- Do not add proprietary SDK or sensor-specific dependencies to `main`.
- Keep the two robot types apart: `aerial` (the quadrotor, EGO-Planner) and
  `legged` (Lite3 and T1, the CMU planner) share no planner code. Say which
  type a change, a test result or a fault concerns, and record it on that
  type's page ([docs/QUADROTOR.md](docs/QUADROTOR.md);
  [docs/QUADRUPED.md](docs/QUADRUPED.md), [docs/HUMANOID.md](docs/HUMANOID.md)).

## Recommended Workflow

1. Read [README.md](README.md) first.
2. If the change affects onboarding, update:
   - [docs/SIMULATION.md](docs/SIMULATION.md)
   - [docs/HARDWARE.md](docs/HARDWARE.md)
   - [CONTRIBUTING.md](CONTRIBUTING.md)
3. If the change affects licensing, update:
   - [LICENSE](LICENSE)
   - [docs/LICENSES.md](docs/LICENSES.md)
4. If the change affects scripted entrypoints, prefer updating the helper
   scripts in `/scripts` so docs stay short.
5. If the change affects a test, a check's limit or a test step, update
   [docs/TESTING.md](docs/TESTING.md).

## Validation Expectations

Tests run locally, not on GitHub, with `./scripts/run_local_tests.sh`. Its
steps are in three groups; [docs/TESTING.md](docs/TESTING.md) has the
procedure and what each step must show to pass.

- `common` (every change): shell scripts pass `bash -n`; the workspace
  builds; `python3 -B star_nav_bringup/test/test_planner_interface.py`
  passes (keep the contract of
  [docs/PLANNER_INTERFACE.md](docs/PLANNER_INTERFACE.md)); `pytest` in
  `mujoco_sim/` passes.
- `aerial` (the quadrotor, EGO-Planner): `rostest star_nav_bringup
  sim_smoke.test` (or `docker compose run --rm test`) and
  `rostest star_nav_bringup mujoco_smoke.test`, with the depth camera and
  with `sensor:=lidar` (or `./scripts/run_mujoco_smoke_test.sh`), must pass.
- `legged` (Lite3 and T1, the CMU planner): `lite3_smoke.test`,
  `t1_smoke.test` and `frontier_smoke.test` must pass.
- While working, run `common` and the group of the code you change. Push
  only when the whole script passes.
- A mission that fails in one run: read its log before running it again.
  Run it again only when the log shows a fault listed in
  [docs/TESTING.md](docs/TESTING.md#failures-that-do-not-happen-every-time)
  and the change touches nothing that type of robot runs; report both runs.
- A fix for such a fault is shown by repeated runs
  (`./scripts/run_local_tests.sh --repeat N <step>`), not by one pass.
- MuJoCo simulator or bridge changes: keep
  `mujoco_sim/star_nav_mujoco/protocol.py` and `mujoco_bridge/src/protocol.h`
  in sync.
- XML/package metadata: keep package names and launch file references accurate.
- Documentation: make simulation commands copy-pasteable.
- Keep `mujoco_sim/` free of ROS imports; ROS-specific code belongs in
  `mujoco_bridge/`.
