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
- The Lite3 quadruped model (MJCF, meshes) and walking policy are
  DeepRoboticsLab's (BSD-3-Clause). They are downloaded on request
  (`--fetch-lite3`) at pinned commits and not redistributed here;
  `star_nav_mujoco/lite3.py` reimplements their policy runner's interface.
- The T1 humanoid model (MJCF, meshes) and walking policy (`T1.pt`) are
  Booster Robotics' booster_gym (Apache-2.0). They are downloaded on request
  (`--fetch-t1`) at a pinned commit and not redistributed here;
  `star_nav_mujoco/t1.py` reads the policy's weights and reimplements its
  observation and PD control from `play_mujoco.py`.
- The Beni model (MJCF, meshes, textures) and policy (`policy.onnx`) are
  Mondo Robotics' open-beni (MIT). They are downloaded on request
  (`--fetch-beni`) at a pinned commit and not redistributed here;
  `star_nav_mujoco/beni.py` reads the policy's weights and reimplements its
  observation and motor model from the training code.
- The scene textures (`star_nav_mujoco/textures.py`) are generated with
  NumPy at run time; no image files are used.
- The MuJoCo quadrotor controller implements the published geometric
  tracking controller of Lee, Leok and McClamroch (2010); no code was copied.
- Several imported packages inherited `TODO` license placeholders in
  `package.xml`. They are aligned with their upstream: GPLv3 for the
  EGO-Planner packages (`rosmsg_tcp_bridge`, `fake_drone`, and the planning
  path), BSD-3-Clause for DecompROS, whose license file ships with it.

## Practical Guidance

- Keep upstream `LICENSE` files in place when moving or reusing code.
- Preserve more permissive package licenses where they are explicitly declared.
- When importing new upstream code, copy its license file or license notice into
  the repository at the same time.
