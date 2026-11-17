# Licensing Notes

This repository is a mixed-license distribution.

## Repository-Level Summary

- The planning and control stack includes GPLv3 upstream components from
  EGO-Planner / Fast-Planner and `px4ctrl`.
- Some utility and message packages remain BSD, LGPLv3, or MIT.
- As distributed in this repository, the overall branch should be treated
  conservatively as GPLv3-compatible software with per-package exceptions
  preserved where they already exist.

The root [LICENSE](../LICENSE) file therefore publishes the
repository under GPLv3 terms for redistribution of this combined codebase.

## Important Subpackage Notes

Every package declares its license in `package.xml` (or `pyproject.toml`);
packages with their own license file are marked.

| Package | License | Origin |
| --- | --- | --- |
| `planner/bspline_opt`, `path_searching`, `plan_env`, `plan_manage`, `traj_utils` | GPLv3 | EGO-Planner / Fast-Planner |
| `planner/rosmsg_tcp_bridge` | GPLv3 | EGO-Planner |
| `planner/exploration_manager` | MIT (`LICENSE`) | original |
| `px4ctrl` | GPLv3 (`LICENSE`) | upstream px4ctrl |
| `uav_simulator/local_sensing`, `map_generator`, `mockamap`, `fake_drone` | GPLv3 | EGO-Planner simulator |
| `uav_simulator/odom_visualization`, `so3_control`, `so3_quadrotor_simulator` | BSD | upstream simulator |
| `utils/DecompROS/*` | BSD-3-Clause (`utils/DecompROS/LICENSE`) | DecompROS |
| `utils/catkin_simple`, `utils/quadrotor_msgs` | BSD | upstream |
| `utils/cmake_utils`, `pose_utils`, `rviz_plugins`, `uav_utils` | LGPLv3 | upstream |
| `star_nav_bringup`, `rviz_gcs_plugin` | MIT (`LICENSE`) | original |
| `mujoco_sim`, `mujoco_bridge` | MIT (`LICENSE`) | original |

Notes:

- `mujoco_sim` and `mujoco_bridge` use these third-party libraries, installed
  as dependencies and not vendored: MuJoCo (Apache-2.0), NumPy
  (BSD-3-Clause), pyzmq (BSD-3-Clause) and libzmq (MPL-2.0).
- The MuJoCo quadrotor controller implements the published geometric
  tracking controller of Lee, Leok and McClamroch (2010); no code was copied.
- The optional Microduck robot (`--robot microduck`) is not in this
  repository. `scripts/setup_mujoco_env.sh --microduck` downloads, for local
  use, Pollen Robotics' model from `pollen-robotics/microduck_rl` (MJCF
  Apache-2.0; meshes **CC BY-NC-SA 4.0, non-commercial**) and the
  `velstand.onnx` walking policy from `pollen-robotics/microduck-policies`
  (Apache-2.0), and installs onnxruntime (MIT) and Rhoban's BAM actuator
  model (Apache-2.0). Do not commit or redistribute the meshes outside the
  terms of their licence. Details: [mujoco_sim/README.md](../mujoco_sim/README.md#microduck).
- Several imported packages inherited `TODO` license placeholders in
  `package.xml`. They are aligned with their upstream: GPLv3 for the
  EGO-Planner packages (`rosmsg_tcp_bridge`, `fake_drone`, and the planning
  path), BSD-3-Clause for DecompROS, whose license file ships with it.

## Practical Guidance

- Keep upstream `LICENSE` files in place when moving or reusing code.
- Preserve more permissive package licenses where they are explicitly declared.
- When importing new upstream code, copy its license file or license notice into
  the repository at the same time.
