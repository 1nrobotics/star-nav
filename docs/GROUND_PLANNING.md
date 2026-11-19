# Planning for Ground Robots: Survey Notes

Star Nav plans with EGO-Planner, a quadrotor planner, and uses it for the
Lite3 quadruped and the T1 humanoid too. These notes collect what others have
done with EGO-Planner on the ground, how off-road wheeled robots are planned
for, and how the CMU ground autonomy stack differs from EGO. They end with
what this means for Star Nav. Written November 2026; claims that were not
checked against a paper or the code are marked as such.

## Summary

- **The problem is known.** Used on a ground robot, EGO-Planner's 3D
  optimization bends trajectories up and over low obstacles. SCAN-Planner
  (2026) reports exactly this for its EGO-Planner-3D baseline; it is also
  why the T1 brushes obstacles in `sensor:=depth` mode
  ([HUMANOID.md](HUMANOID.md)).
- **Two fixes are in use:** plan in 2D (a fixed height), or penalize or
  suppress vertical motion. Either needs the map to put an obstacle's whole
  height into the planning plane; Star Nav's terrain filter does this for the
  lidar.
- **Off-road wheeled robots** are planned for in four ways: terrain analysis
  plus a local planner (CMU), SE(2) optimization over terrain (ZJU FAST Lab),
  learned traversability plus a classic planner, and end-to-end learned
  planners (YOPO-Rally, iPlanner/ViPlanner).
- **The CMU stack** perceives in 2.5D and plans in 2D, by picking one of 343
  precomputed paths. It has no notion of time or dynamics, which a separate
  path follower adds. It is simple, deterministic and cheap, which suits slow
  walking robots.
- **Next steps for Star Nav:** a 2D mode for EGO (with a height-band
  projection for the depth camera), then the CMU planner as a second backend
  for the walking robots, and an MPC in place of the proportional `Tracker`.

## 1. EGO-Planner on ground robots

| Work | Robot | What they did |
|---|---|---|
| Chen et al., *Sensors* 24(22):7306, 2024 (NUS) | Lite3 (Gazebo), RealSense D435i | EGO-Planner reduced to 2D, to "limit unnecessary Z-axis movement"; D* global path; the CoM trajectory goes to MPC + WBC. Box obstacles only, despite the title's rough terrain. |
| SCAN-Planner, Zheng et al., arXiv 2606.19555, 2026 (SJTU) | Unitree Go2 | Uses EGO-Planner-2D and -3D as baselines. 3D: deforms upward over obstacles, infeasible for a walking robot. 2D: stable in planar clutter but treats stairs as obstacles. Both use an isotropic inflation radius. Its own planner: a yaw-aware twin-cylinder footprint, A\* with suppressed z gradient, height-regularized B-spline optimization, a sliding 3D map. |
| RAEM, Yuan et al., arXiv 2608.25366, 2026 | quadruped, multi-floor | Exploration through stairwells with a tomography map and a 3D grid. A search summary said it adds a z penalty to EGO-Planner and a rolling local map; **not verified** (the abstract does not mention EGO-Planner). |
| EGO-Planner V2.1 | aerial + ground swarm | Extends EGO-Planner V2 to ground robots in a heterogeneous swarm. **Not verified** (page not accessible). |
| Dangko/ego-planner-for-ground-robot (GitHub, no paper) | simulated unicycle | Ground segmentation (patchwork), the rest of the cloud projected to a 2D grid, EGO in 2D, MPC (qpOASES) tracking. CMU's vehicle simulator, indoor. |

SCAN-Planner's simulation benchmark (dense obstacles, 50 trials):

| Method | Success | Collision |
|---|---|---|
| EGO-Planner-2D | 0.96 | 0.04 |
| EGO-Planner-3D | 0.76 | 0.24 |
| SCAN-Planner | 1.00 | 0.00 |

## 2. Off-road wheeled robots

| Family | Examples | Strengths | Weaknesses |
|---|---|---|---|
| Terrain analysis + local planner | CMU autonomy stack (terrain analysis, local planner, FAR, TARE); NATURE stack | Mature, explainable, widely deployed | Geometry only (tall grass = wall); dense vegetation and narrow passages (FFI's Milrem Themis report) |
| SE(2) optimization over terrain | Uneven Planner (arXiv 2309.06115, BSD-3), SEB-Naver (arXiv 2503.02412, GPL-3.0), terrain safety corridors + CBF (*Drones* 2026) | Vehicle kinematics and attitude (pitch, roll) in the optimization; smooth trajectories | Needs an accurate terrain map; heavier computation |
| Learned traversability + classic planner | Wild Visual Navigation (ETH), CMU TartanDrive work, Trailblazer | Tells pushable grass from obstacles | Needs data; self-supervision pipelines |
| End-to-end learned planner | YOPO-Rally, iPlanner / ViPlanner | Depth camera only, low latency, no explicit map | Less explainable; generalization depends on the simulator |

**YOPO-Rally** (Cao, Lu et al., arXiv 2505.18714, 2025; Tianjin University)
extends YOPO, a learned one-stage quadrotor planner (RA-L 2024), to forests:

- Input: a depth image, the current velocity and the goal direction.
- Output: several candidate trajectories (Hermite curves) with costs; the
  best goes to an MPC.
- Training: behaviour cloning on expert trajectories, which a traversability
  module computes from cost maps in randomized forests of its Unity
  simulator, YOPO-Sim. Deployed without fine-tuning.
- Code: ROS 2 Humble; the README states **no license**, so its code cannot
  be used here.

**iPlanner / ViPlanner** (ETH; ViPlanner at ICRA 2024) learn a depth (plus
semantics, for ViPlanner) planner in simulation by "imperative learning":
the loss is a differentiable traversability cost, not expert demonstrations.
Tested on ANYmal; ROS Noetic package; iPlanner is MIT.

YOPO-Rally is a second example of a drone planner moved to the ground, and
its strength (depth only) is where Star Nav's walking robots are weakest.

## 3. The CMU ground autonomy stack

From the code of `autonomous_exploration_development_environment` (noetic
branch).

### Terrain analysis (`terrainAnalysis.cpp`): 2.5D

- A 51 × 51 grid of 0.2 m cells around the vehicle. A cell's ground is the
  25 % quantile of its point heights (`quantileZ`), or its lowest point.
- Each point's height above its cell's ground goes into the point's
  `intensity`. Only points from 0 to `vehicleHeight` (1.5 m) above the
  ground are kept, so the vehicle passes under branches and tables.
- A rolling 21 × 21 m voxel map; points decay after 2 s beyond 4 m from the
  vehicle (`decayTime`, `noDecayDis`); the map can be cleared on request.

### Local planner (`localPlanner.cpp`): 2D

- A library of 343 paths in 7 groups, rotated to 36 directions (10° apart);
  reverse driving optional (`twoWayDrive`).
- Collision checks use x and y only: obstacle points are rotated by yaw into
  the vehicle frame (roll and pitch are read but not used).
- A point more than `obstacleHeightThre` (0.2 m) above the ground blocks the
  paths through its cell; points between 0.1 and 0.2 m can add a cost
  (`useCost`, off by default). No explicit slope handling: a ramp is free
  because its ground rises with it.
- Unblocked paths are scored by direction to the goal and turning; the best
  group wins. With no free path, paths are shortened (`pathScale` 1.0 →
  0.75) and the range reduced.

### Offline path generation (`paths/path_generator.m`)

No search or optimization: a hand-designed, three-level tree of curves.

- Each path is defined in polar form: heading θ(r) over distance r = 0…3 m,
  interpolated with MATLAB's `spline` between knots and sampled every
  0.01 m (x = r cos θ, y = r sin θ).

  | Segment | Distance | Heading | Branches |
  |---|---|---|---|
  | 1 | 0 → 1 m | 0 → shift1 ∈ −27° … +27°, step 9° | 7 |
  | 2 | 1 → 2 m | shift1 ± 17.55°, step 5.85° | 7 |
  | 3 | 2 → 3 m | shift2 ± 11.41°, step 3.80° | 7 |

- 7 × 7 × 7 = 343 paths. The 49 paths sharing a first segment form a group:
  the planner commits to the first metre only and re-selects next frame
  (receding horizon).
- **Correspondences:** a 0.02 m grid of 161 × 451 cells ahead of the
  vehicle; for each cell, `rangesearch` lists the paths that pass within
  `searchRadius` = 0.45 m (the vehicle's half-width plus a margin). Online,
  an obstacle point is looked up by cell, and each path through it gets a
  count. This makes a check a table lookup, independent of geometry.
- This is a motion-primitive (path-set) method. It goes back, as far as we
  recall (not checked), to Zhang et al.'s Falco (Journal of Field Robotics),
  which scores paths by collision likelihood.

### Path follower and global planners

- The path follower turns the chosen path into `/cmd_vel` like a
  differential-drive vehicle (turn, then drive), with speed, yaw-rate and
  acceleration limits.
- **FAR Planner** (route planning): extracts obstacle polygons and keeps a
  visibility graph, extended globally as the robot moves. The paper's
  multi-layer polygons are an extension for aerial vehicles.
- **TARE** (exploration) uses the same terrain analysis.
- The Go2 version (`autonomy_stack_go2`) uses the Go2's L1 lidar with
  Point-LIO, on ROS 2 Foxy; it asks for obstacles higher than 0.3 m
  because of the L1's noise. A "Ground Autonomy Stack" by Vector Robotics
  is said to support the Unitree G1; **not verified** (repository not
  accessible).

## 4. CMU local planner vs EGO-Planner

| | CMU local planner | EGO-Planner |
|---|---|---|
| Method | Selection from a precomputed path library | Gradient-based optimization (L-BFGS) of B-spline control points |
| Space | 2D (vehicle frame) | 3D |
| Output | A geometric path, no timing | A timed trajectory (position, velocity, acceleration) |
| Dynamics | None; the path follower sets the speed | Velocity, acceleration and jerk limits in the cost |
| Obstacles | Raw points looked up in a grid → path table | Inflated occupancy grid, "rebound" (A\* around a colliding segment gives push-out directions) |
| Distance field | Not needed | Not needed (EGO's point) |
| Cost per cycle | Very small: lookups and counts | Larger: iterations, A\* |
| Failure | All paths blocked: shorten, then stop | Local minima, no convergence: replan |
| Determinism | High | Lower (initial guess, iterations) |

In practice: EGO gives smooth, fast trajectories and squeezes through
arbitrary gaps; CMU is conservative and predictable, reacts every frame,
and runs on weak computers. For robots walking at 0.5 m/s the CMU approach
loses little.

## 5. Path library vs MPC

Both are receding-horizon: pick among candidates, execute the start, repeat.
The difference is where the candidates come from and whether a model is used.

| | CMU path library | MPC (sampling, e.g. MPPI/CEM/DWA, or optimization) |
|---|---|---|
| Candidates | 343 fixed shapes, offline | Control sequences rolled out through a model, online |
| Model | None | Unicycle, bicycle, or dynamics |
| Time, speed | Not in the path | Part of the trajectory |
| Constraints | Collision only; curvature implicit | Explicit: limits, non-holonomy, roll, slip |
| Cost | Nearly binary (blocked or not) + heading score | Continuous weighted sum |
| Output | A path for a separate follower | Control commands directly |
| Frame to frame | Independent; may switch groups | Warm-started from the last solution |
| Compute | Tiny | Hundreds to thousands of rollouts (often GPU), or an NLP |

DWA is in between: it samples (v, ω) and rolls out a constant-velocity
model. Many off-road stacks combine the two: a planner for the path, an MPC
to track it (Uneven Planner, SEB-Naver, YOPO-Rally).

For walking robots, an MPC would use a reduced model: the velocity command
to actual velocity as a first-order lag, plus unicycle or omnidirectional
kinematics. That captures the policy's response lag, which limits Star Nav's
tracking today (T1: 0.42 m/s reached at a 0.5 m/s command).

## 6. Making EGO-Planner 2D

Setting the height constant is not enough on its own. In this repository:

1. **A\*** (`planner/path_searching/src/dyn_a_star.cpp`, the `dz` loop):
   search the planning layer only (`dz = 0`). This matters most: EGO's
   rebound uses A\* around a colliding segment to get push-out directions;
   an A\* path over the obstacle gives mostly vertical directions, which
   cannot push the trajectory out sideways.
2. **Optimizer** (`planner/bspline_opt/src/bspline_optimizer.cpp`,
   `combineCostRebound` and `combineCostRefine`): zero the z row of the
   gradient. With all control points starting at the planning height, L-BFGS
   then never moves z. Project the push-out directions (`cps_.direction`)
   onto the plane and normalize them.
3. **Start and goal** at the planning height (the exploration manager does
   this already).

**The map must match.** A single slice at the planning height misses low
obstacles: at the T1's 0.67 m, a 0.3 m stump with 0.35 m inflation reaches
only 0.65 m. The plane must contain everything from a step's height to the
robot's height. The terrain filter does this for the lidar (columns); for
the depth camera, the grid map needs a height-band projection.

| | Virtual ceiling | 2D mode |
|---|---|---|
| Change | ~10 lines in `grid_map` (on `feature/exploration`) | A\*, optimizer, a parameter |
| How | Space above the ceiling counts as occupied | No z motion by construction |
| Guarantee | The band between ceiling and plan height remains | Constant z |
| Low obstacles | Need the band projection / terrain filter | Same |
| Drones | Useful too | Ground robots only |

## 7. The CMU stack on a humanoid

The interface fits: CMU outputs a velocity command, and humanoid policies
take (vx, vy, ωz). The work is in parameters and the robot's peculiarities.
For the T1 in this simulator:

- **Inputs:** `/registered_scan` (world-frame cloud) and `/state_estimation`
  (sensor pose); the bridge publishes both under other names.
- **Terrain analysis:** `vehicleHeight` ≈ 1.25 m (the table is an obstacle);
  `obstacleHeightThre` ≈ 0.05 m (the T1 cannot climb 7 cm steps); check
  `minRelZ`/`maxRelZ` for a lidar 1.2 m up.
- **Local planner:** footprint ≈ 0.3 × 0.54 m (arms); regenerate the
  correspondences with `searchRadius` ≈ 0.35 m; `twoWayDrive` off (walking
  backwards is slow and unsteady).
- **Path follower:** turn-then-walk suits a humanoid (sideways only
  0.26 m/s); limit speed, yaw rate and acceleration.
- **Simulator:** a velocity-command mode that bypasses `Tracker`
  (`protocol.py` and `protocol.h` in sync).
- **Humanoid-specific:** the head lidar sways with the gait, so the cloud
  needs the full 6-DoF pose; filter the arms from the scan on hardware; one
  obstacle threshold cannot express "down 7 cm yes, up no".
- **Licensing:** the CMU repositories are believed to be BSD; check each
  before importing and update [LICENSES.md](LICENSES.md).

## 8. What this means for Star Nav

1. **2D mode for EGO** (`planning_2d`, on for walking robots) and a
   height-band projection in the grid map, so `sensor:=depth` works; keep the
   virtual ceiling for drones.
2. **CMU local planner as a second backend** for walking robots
   (`exploration_legged.launch planner:=ego|cmu`), compared on the same
   missions.
3. **MPC tracking** in place of the proportional `Tracker`, with a lagged
   velocity model of the walking policy.
4. **A rover** (differential drive) in MuJoCo with outdoor terrain; EGO +
   terrain filter for gentle ground, an SE(2) planner for rough ground.
5. The simulator (procedural forests, depth, lidar, ground truth) could
   generate training data for a YOPO- or iPlanner-style planner.

## References

- EGO-Planner: Zhou et al., RA-L 2021, [arXiv 2008.08835](https://arxiv.org/abs/2008.08835)
- Chen et al., Path Planning and Motion Control of Robot Dog Through Rough Terrain Based on Vision Navigation, [Sensors 2024](https://pmc.ncbi.nlm.nih.gov/articles/PMC11598469/)
- SCAN-Planner, [arXiv 2606.19555](https://arxiv.org/abs/2606.19555)
- RAEM, [arXiv 2608.25366](https://arxiv.org/abs/2608.25366)
- EGO-Planner V2.1, [ResearchGate](https://www.researchgate.net/publication/385880528_EGO-Planner_V21_Swarm_of_Heterogeneous_Mobile_Robots_in_Habitat)
- [Dangko/ego-planner-for-ground-robot](https://github.com/Dangko/ego-planner-for-ground-robot)
- Uneven Planner, [arXiv 2309.06115](https://arxiv.org/abs/2309.06115), [code](https://github.com/ZJU-FAST-Lab/uneven_planner)
- SEB-Naver, [arXiv 2503.02412](https://arxiv.org/abs/2503.02412)
- Terrain safety corridors and CBF for UGVs, [Drones 2026](https://doi.org/10.3390/drones10060440)
- DDR-opt, [arXiv 2409.07924](https://arxiv.org/abs/2409.07924); Dftpav, [arXiv 2208.13160](https://arxiv.org/abs/2208.13160)
- YOPO-Rally, [arXiv 2505.18714](https://arxiv.org/abs/2505.18714), [code](https://github.com/TJU-Aerial-Robotics/YOPO-Rally); YOPO, [code](https://github.com/TJU-Aerial-Robotics/YOPO)
- ViPlanner, [arXiv 2310.00982](https://arxiv.org/abs/2310.00982); [iPlanner](https://github.com/leggedrobotics/iPlanner)
- Wild Visual Navigation, [arXiv 2305.08510](https://arxiv.org/abs/2305.08510); Trailblazer, [arXiv 2505.09739](https://arxiv.org/abs/2505.09739); Verti-Bench, [arXiv 2502.11426](https://arxiv.org/abs/2502.11426)
- CMU code: [localPlanner.cpp](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/local_planner/src/localPlanner.cpp), [terrainAnalysis.cpp](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/terrain_analysis/src/terrainAnalysis.cpp), [path_generator.m](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/blob/noetic/src/local_planner/paths/path_generator.m); [autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2); [open-source list](https://frc.ri.cmu.edu/~zhangji/others.html)
- FAR Planner, [arXiv 2110.09460](https://arxiv.org/abs/2110.09460); Falco, [CMU RI](https://ri.cmu.edu/?p=125590)
- FFI, [Development of off-road autonomy for UGVs](https://www.ffi.no/en/publications-archive/development-of-off-road-autonomy-for-unmanned-ground-vehicles)
