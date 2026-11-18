# Open-Source Branch Plan

This document tracks the repository changes needed to make `star-nav` usable as
a public-facing codebase.

## Goals

- Provide a clear, simulation-first quick start.
- Keep `main` hardware-independent; sensor-specific integration lives on
  `hardware/*` branches.
- Clean up repository hygiene issues that make the repo look internal-only.
- Clarify mixed licensing inherited from upstream planning/control components.

## Phase 1: Repository Foundation

- [x] Audit repository structure, launch entrypoints, and licensing signals.
- [x] Add agent collaboration guidance in `AGENTS.md`.
- [x] Add root repository hygiene files (`.gitignore`, `.editorconfig`,
      `.clang-format`).
- [x] Add root-level license and mixed-license documentation.

## Phase 2: Documentation Refresh

- [x] Rewrite the root README around simulation-first onboarding.
- [x] Add dedicated simulation documentation.
- [x] Add dedicated hardware deployment documentation.
- [x] Add contribution guidance for external collaborators.

## Phase 3: Simulation Usability

- [x] Add simple helper scripts for launching simulation and triggering
      exploration.
- [x] Make the Docker-based simulation path work without proprietary SDKs.
- [x] Move VK180 hardware integration to the `hardware/vk180` branch and the
      simulation launch files to `star_nav_bringup`.
- [x] Add a headless end-to-end simulation smoke test
      (`rostest star_nav_bringup sim_smoke.test`) runnable through
      `docker compose run --rm test`.

## Phase 4: Metadata and Cleanup

- [x] Fix package license metadata for the main planning/simulation path where
      upstream GPLv3 provenance is clear.
- [x] Remove tracked editor backups, Python bytecode, and nested package build
      outputs.
- [x] Resolve the remaining `TODO` license placeholders and add `LICENSE`
      files to original MIT packages (see docs/LICENSES.md).
- [ ] Continue normalizing README quality inside subpackages with stale or
      conflicting instructions.

## Phase 5: MuJoCo Simulator and Release Prep

- [x] MuJoCo 4 simulator without ROS (`mujoco_sim/`) and a thin ROS bridge
      (`mujoco_bridge/`), with unit tests, `mujoco_smoke.test` and CI jobs.
- [x] Project page (`site/`) and GitHub Pages workflow.
- [x] Community files: issue and PR templates, `CODE_OF_CONDUCT.md`,
      `SECURITY.md`, `CITATION.cff`.
- [x] Attach the demo video to the `site-assets` release (site/README.md).
- [x] Make the repository public, then enable Pages (Source: GitHub Actions)
      and private vulnerability reporting.

## Phase 6: Sensors

- [x] 360-degree lidar in the MuJoCo simulator as an alternative obstacle
      sensor, with accuracy checks of the sensor data and the planner map
      (docs/SENSORS.md).
- [x] Point cloud ray casting clears obstacles that moved away: distant
      returns, sensor ray origin, exact voxel traversal
      (planner/plan_env/POINTCLOUD_RAYCASTING_FIX.md).
- [ ] Lidar beams with no return as free space (per-sensor option).
- [ ] Moving obstacle in the MuJoCo world with a map clearing-time check.

## Known Follow-Ups

- Hardware branches depend on proprietary vendor tooling and cannot be fully
  validated in a public environment.
- The `sim-smoke` GitHub Actions workflow runs both smoke tests and the
  MuJoCo unit tests on every pull request.
