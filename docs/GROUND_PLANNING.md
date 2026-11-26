# Planning for Ground Robots: Survey Notes

Star Nav flies the quadrotor with EGO-Planner and first used it for the
Lite3 quadruped and the T1 humanoid too; they now walk with the CMU local
planner only (section 10). These notes collect what others have
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
  height into the planning plane; Star Nav's terrain filter did this for the
  lidar while EGO moved the walking robots (section 10).
- **Off-road wheeled robots** are planned for in four ways: terrain analysis
  plus a local planner (CMU), SE(2) optimization over terrain (ZJU FAST Lab),
  learned traversability plus a classic planner, and end-to-end learned
  planners (YOPO-Rally, iPlanner/ViPlanner).
- **The CMU stack** perceives in 2.5D and plans in 2D, by picking one of 343
  precomputed paths. It has no notion of time or dynamics, which a separate
  path follower adds. It is simple, deterministic and cheap, which suits slow
  walking robots.
- **MPPI** samples control sequences, rolls them out through a model and
  averages them weighted by cost. It takes any cost and model and outputs
  commands directly; it needs a reference path to choose a side around an
  obstacle, and more compute than a path library.
- **In Star Nav:** the CMU planner steers the walking robots (section 10).
  Open: a 2D mode for EGO (with a height-band projection for the depth
  camera) and an MPPI tracker in place of the proportional `Tracker`.

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

## 6. MPPI

Model Predictive Path Integral control (Williams, Theodorou et al., Georgia
Tech, 2016–2018) is the most widely used sampling MPC in ground robotics:
Nav2's default-recommended controller, the AutoRally cars, and many off-road
stacks. It does not take gradients or solve an optimization problem; it
samples, rolls out and averages.

### Algorithm

Horizon T steps of dt, K samples, nominal control sequence U = (u₀ … u_{T−1}),
noise covariance Σ, temperature λ. Every control cycle:

```
1. sample   ε^k ~ N(0, Σ) for k = 1..K;  v^k_t = clip(u_t + ε^k_t)
2. roll out x^k_{t+1} = f(x^k_t, v^k_t) from the current state
3. cost     S_k = φ(x^k_T) + Σ_t [ q(x^k_t) + γ u_tᵀ Σ⁻¹ ε^k_t ]
4. weight   w_k = exp(−(S_k − min S) / λ) / Σ_j exp(−(S_j − min S) / λ)
5. update   u_t ← u_t + Σ_k w_k ε^k_t
6. smooth   filter U (optional, e.g. Savitzky–Golay)
7. execute  u₀
8. shift    U left by one step (warm start for the next cycle)
```

The last term of step 3 is a control cost from the derivation; implementations
make γ small or drop it. λ sets how selective the average is: as λ → 0 it
takes the best sample, as λ grows it averages all of them.

**Theory, briefly.** For a system with noisy controls, the optimal
distribution of control sequences is q*(V) ∝ p(V) exp(−S(V)/λ), the sampling
distribution reweighted by the cost. MPPI minimizes the KL divergence from its
own (Gaussian) control distribution to q*; the solution is the mean of q*,
which steps 4–5 estimate by importance sampling. CEM differs in keeping only
the best (elite) samples and refitting the variance; MPPI weights all samples
softly and usually keeps Σ fixed.

### Strengths and problems

| Strength | Why it matters |
|---|---|
| Cost need not be differentiable | Occupancy grids, traversability maps, learned costs, hard collision checks |
| Model need not be differentiable or linear | Kinematics, slip dynamics, learned (neural) dynamics |
| Parallel | Rollouts are independent: thousands on a CPU with vectorization, tens of thousands on a GPU |
| Warm start | One iteration per cycle is usually enough; controls stay continuous |
| Planning and control in one | Outputs velocity commands that both avoid and track |

| Problem | Cause | Usual remedy |
|---|---|---|
| Chattering | Averaged noise does not cancel | Smooth U (Nav2); Smooth-MPPI samples the control's derivative |
| Stuck in front of an obstacle | All samples near it; the average sits in the middle | More noise or samples; Log-MPPI (heavier-tailed sampling); a reference path |
| Left or right | Samples stay near the nominal sequence | Several nominal sequences; seed samples from the global path |
| Tuning | λ, Σ and cost weights interact | Normalize costs; set Σ from the robot's acceleration, then λ |
| No hard constraints | The average can leave the limits | Clip samples and the result; collision as a very large cost |
| Compute | K × T model steps | Vectorize; a simple model; a short horizon |

### Nav2's implementation

`nav2_mppi_controller` (C++, xtensor, CPU) is the most complete open
implementation: differential-drive, omnidirectional and Ackermann models, and
a cost made of switchable critics:

| Critic | Penalizes |
|---|---|
| Constraint | Speed and acceleration beyond the limits |
| Cost / Obstacles | Costmap: collision, closeness to obstacles |
| PathAlign, PathFollow, PathAngle | Leaving the global path, not progressing along it, heading off it |
| Goal, GoalAngle | Distance to the goal and its heading, near the goal |
| PreferForward | Driving backwards |
| Twirling | Spinning in place (omnidirectional robots) |

Its defaults are of the order of 1000–2000 samples, 56 steps of 0.05 s
(2.8 s), λ ≈ 0.3, one iteration per cycle at 20–30 Hz (from memory; not
verified against the current release). The split, a global path plus MPPI
tracking and avoiding, matches Star Nav's EGO reference plus tracker.

**Off-road.** AutoRally drove MPPI with a learned vehicle model near the
limits on dirt. Learned traversability costs (TartanDrive, TerrainNet-style
maps) plug straight into q(x), where a non-differentiable cost is an
advantage. Risk-aware variants replace the expected cost by a quantile
(CVaR) on uncertain terrain.

### MPPI vs the path library

Both generate candidates, score them and pick; they differ in where
candidates come from, how the result is chosen, and what comes out.

| | CMU path library | MPPI |
|---|---|---|
| Candidates | 343 offline splines, rotated to 36 headings | K control sequences sampled each cycle around the last solution |
| Candidate space | Geometric paths, no time or speed | Control sequences rolled out to state-time trajectories |
| Dynamics | None; feasibility built into the path shapes | In the rollout (kinematics, a lag, a learned model) |
| Collision check | Lookup: voxel → paths through it, one lookup per obstacle point | Map query at every step of every rollout (K × T) |
| Cost | Mostly binary collision, plus heading and terrain | Any weighted sum |
| Selection | Discrete: path scores summed per group, best group | Soft weighted average; may lie outside the samples |
| Frame to frame | Stateless | Warm-started |
| Output | A path for a separate follower | Velocity commands |
| Speed | Decoupled, set by the follower | Optimized |
| Determinism | Deterministic, reproducible | Stochastic |
| Resolution | Limited by the library | Continuous, accuracy set by K |
| Compute | Well under a millisecond | Milliseconds on a CPU for hundreds to thousands of samples |

Two behaviours differ most:

1. **An obstacle straight ahead.** Samples passing left and right both score
   well, and samples through the middle score badly. MPPI's average can land
   in the middle; warm starting with a small λ usually collapses it to one
   side, but nothing guarantees it. The path library chooses one group, and
   summing scores per group asks "which direction has the most free
   continuations", a robust choice. This is why MPPI is usually guided by a
   global path, which decides the side.
2. **Dynamics.** The path library guarantees geometric feasibility only;
   whether the robot keeps up is the follower's problem. CMU's pure-pursuit
   follower is enough for wheels but ignores a lag like the T1's (0.42 m/s
   at a 0.5 m/s command). MPPI rolls the lag out, so its plan is what this
   robot will actually do with these commands.

They are two ends of one family. MPPI with a fixed sample set, λ → 0, no
warm start and a collision-only cost is a path library. The two also
combine: seed some of MPPI's samples with library paths or fixed manoeuvres
(left, right, turn in place) and sample the rest randomly. Biased-MPPI
(Trevisan and Alonso-Mora, RA-L 2024) generalizes this by drawing samples
from ancillary controllers.

### For Star Nav

| Option | Replaces | For | Against |
|---|---|---|---|
| Path library (CMU) | EGO and `Tracker`, plus a follower | Very fast, deterministic, proven on ground robots; its 2D check fixes the depth camera case | Largest change; EGO's tuning and tests redone; no lag model; offline data files |
| MPPI tracker | `Tracker` only | Small change, same interface; compensates the walking policies' lag; checks a 2D map, which also covers the depth camera case | Heavier compute; tuning; relies on EGO to pick the side |
| MPPI seeded with manoeuvres | `Tracker` only | As above, less stuck in the middle | A little more code |

The MPPI tracker comes first: it fits the EGO-reference-plus-tracker split,
stays in `mujoco_sim` (no ROS), and EGO already decides the side, which
removes MPPI's main weakness. A sketch (not implemented):

- **Model:** omnidirectional kinematics with a first-order velocity lag.
  State (x, y, ψ, vx, vy, ω), body-frame command (vx, vy, ω);
  v ← v + (u − v) dt/τ, τ ≈ 0.2–0.3 s from a step response, optionally a
  speed gain (≈ 0.84 for the T1).
- **Cost:** distance to the EGO reference at the matching time and its
  heading; the planner's inflated occupancy (or the terrain filter's map),
  collision as a large cost; limits; sideways and backward penalties.
- **Start:** K = 512, T = 30 × 0.05 s (1.5 s), λ = 0.3; Σ from the
  achievable change per step (dog: 0.3 m/s in vx, vy and 0.5 rad/s;
  T1: smaller vy).
- **Implementation:** NumPy, K × T × 3 arrays; milliseconds per cycle on a
  CPU, enough for 50 Hz; no new dependency.
- **Checks:** tracking error and speed reached against `Tracker` on flat
  ground, then `lite3_smoke` and `t1_smoke`, then
  `t1_smoke.test world:=forest sensor:=depth` (does the 2D map check stop
  the contacts?).

## 7. Making EGO-Planner 2D

Setting the height constant is not enough on its own. In this repository:

1. **A\*** (`planner/ego/path_searching/src/dyn_a_star.cpp`, the `dz` loop):
   search the planning layer only (`dz = 0`). This matters most: EGO's
   rebound uses A\* around a colliding segment to get push-out directions;
   an A\* path over the obstacle gives mostly vertical directions, which
   cannot push the trajectory out sideways.
2. **Optimizer** (`planner/ego/bspline_opt/src/bspline_optimizer.cpp`,
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

## 8. The CMU stack on a humanoid

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
  (`protocol.py` and `protocol.h` in sync). Done: section 10.
- **Humanoid-specific:** the head lidar sways with the gait, so the cloud
  needs the full 6-DoF pose; filter the arms from the scan on hardware; one
  obstacle threshold cannot express "down 7 cm yes, up no".
- **Licensing:** the CMU repositories are believed to be BSD; check each
  before importing and update [LICENSES.md](LICENSES.md). Checked for the
  two imported packages: BSD in `package.xml`, no license file.

## 9. What this means for Star Nav

Written while EGO moved the walking robots. They now walk with the CMU
planner only (section 10), so items 1, 3 and 4 would start from that, or
bring EGO back for ground robots first.

1. **2D mode for EGO** (`planning_2d`, on for walking robots) and a
   height-band projection in the grid map, so `sensor:=depth` works; keep the
   virtual ceiling for drones.
2. **CMU local planner as a second backend** for walking robots
   (`exploration_ground.launch planner:=ego|cmu`), compared on the same
   missions. Done: section 10.
3. **MPPI tracking** in place of the proportional `Tracker`, with a lagged
   velocity model of the walking policy (section 6; not started).
4. **A rover** (differential drive) in MuJoCo with outdoor terrain; EGO +
   terrain filter for gentle ground, an SE(2) planner for rough ground.
5. The simulator (procedural forests, depth, lidar, ground truth) could
   generate training data for a YOPO- or iPlanner-style planner.

## 10. The CMU planner in Star Nav

The CMU planner is the planner of a legged robot, the Lite3 and the T1:
`star_nav_bringup/launch/planners/cmu.xml`. The quadrotor keeps EGO: the CMU
planner is 2D and outputs a ground vehicle's velocity. Both sit behind one
interface: [PLANNER_INTERFACE.md](PLANNER_INTERFACE.md).

Until November 2026 the walking robots could run EGO as well
(`planner:=ego`): EGO planned at the height of the robot's body, a terrain
filter (`terrain_filter.py`) turned the lidar's scans into obstacle columns
so that ramps and steps of 9 cm or less stayed free, and the simulator
steered the robot to EGO's position setpoints. The measurements below
compare the two on the same missions. That path and the filter were removed
with the planner interface (the T1 first, then the Lite3), so each kind of
robot has one planner; the `ego` rows cannot be repeated from the launch
files.

```
lidar cloud (world frame) ─► terrainAnalysis ─► /terrain_map ─► localPlanner ─► /path
exploration_manager goal ─► cmu_waypoint_router.py ─► /way_point ─┘               │
                                 ▲ /terrain_map                              pathFollower
                                                                                  │ /cmd_vel
                              mujoco_bridge ─► simulator ─► walking policy ◄──────┘
```

- **CMU code, unmodified:** `planner/cmu/terrain_analysis` and
  `planner/cmu/local_planner` (`localPlanner`, `pathFollower`) from
  autonomous_exploration_development_environment, branch `noetic`, commit
  `bf0cba7` (BSD). Its topics (`/state_estimation`, `/registered_scan`) are
  remapped to the bridge's odometry and lidar cloud. The path set is
  upstream's (343 paths, 0.45 m clearance, about 3 m long), downloaded by
  `scripts/fetch_cmu_paths.sh`; it was not regenerated for these robots.
- **Velocity commands:** the bridge forwards `/cmd_vel` to the simulator
  (protocol version 2, topic `vel`); a ground robot's walking policy gets it
  within `tracking.Holonomic`'s limits, in place of the tracker's output.
  Without a command for 0.5 s the robot stands.
- **Waypoint router** (`star_nav_bringup/scripts/planners/cmu/cmu_waypoint_router.py`,
  Star Nav's own): the local planner looks 3 m ahead and heads straight for
  its waypoint, so the mission's goals, up to 9 m away and behind walls,
  leave it facing a wall. Measured in the house: without the router the
  Lite3 reached 11 of 12 waypoints and timed out on the last one, 1.4 m away
  on the other side of the wall between the rooms. The router stands in for
  CMU's global planners (FAR planner, TARE): it keeps a 0.1 m grid of the
  terrain map's obstacles (remembered beyond 3.5 m from the robot: static
  worlds only), searches it with A* and gives the local planner the farthest
  point of the route, up to 3 m along it, that is in a straight free line.
- **Parameters** (`planners/cmu.xml`; the rest are upstream's defaults):

  | Parameter | Value | Why |
  |---|---|---|
  | `maxSpeed`, `autonomySpeed` | the robot's `max_vel` (0.6, 0.5 m/s) | |
  | `maxYawRate` | 40°/s | upstream 80; a cautious choice, not tuned |
  | `autonomyMode` | true | no joystick |
  | `twoWayDrive` | false | walk forwards |
  | `vehicleHeight` | the robot's `robot_height` (0.55, 1.25 m) | higher returns are overhead |
  | `obstacleHeightThre` | 0.15 m | the 7 cm steps and the 12° ramp are ground, the 0.21 m platform edge is not |
  | `clearDyObs` | false | static world; upstream's setting assumes a VLP-16's field of view |
  | `pathScale` / `minPathScale` | 1.0 / 0.5 | clearance 0.45 m, down to 0.23 m where nothing else is free |
  | `dirDiffThre` | 0.2 rad | upstream 0.1; walks while within this of the path's direction |

- **Limits.** The lidar only (`sensor:=depth` is refused). Static
  worlds only. The walking robots' tests do not check a map against the
  world (the quadrotor's check reads EGO's map). One obstacle threshold for both robots: the
  T1 is allowed onto 7 cm steps going up, which it does not always manage
  ([HUMANOID.md](HUMANOID.md)). The router has no unit tests yet.

Measured (house, lidar, default mission, `lite3_smoke.test` and
`t1_smoke.test`; one run per row on 2026-11-23, so the times are indications,
not a benchmark):

| Robot | Planner | Waypoints | Contacts | Falls | Mission time (simulated) |
|---|---|---|---|---|---|
| Lite3 | `cmu` | 12/12 | 0 | 0 | 153 s |
| Lite3 | `ego` | 12/12 | 0 | 0 | 194 s |
| T1 | `cmu` | 12/12 | 0 | 0 | 174 s |
| T1 | `ego` | 12/12 | 0 | 0 | 234 s |

Both robots crossed the ramp, the platform and the steps with either
planner.

Lite3 in more scenes (same mission, same 0.6 m/s limit; one run per row on
2026-11-23, the default house twice). Time, path, speed and standing time
are from the odometry between the first waypoint and the last movement;
clearance is the horizontal distance from the torso's centre to the nearest
obstacle surface:

| Scene | Planner | Result | Time (s) | Path (m) | Mean speed (m/s) | Standing | Min clearance (m) |
|---|---|---|---|---|---|---|---|
| House | `cmu` | pass | 152, 146 | 64.2, 61.8 | 0.42 | 13%, 11% | 0.32, 0.36 |
| | `ego` | pass | 192, 186 | 65.8, 64.8 | 0.34, 0.35 | 2% | 0.41, 0.40 |
| House, `lidar:=mid360` | `cmu` | pass | 153 | 64.1 | 0.42 | 11% | 0.37 |
| | `ego` | pass | 193 | 64.1 | 0.33 | 3% | 0.39 |
| House, 2 cm lidar range noise | `cmu` | pass | 139 | 61.5 | 0.44 | 10% | 0.31 |
| | `ego` | fail: waypoint 8 timed out | 238 | 68.4 | 0.29 | 17% | 0.43 |
| Forest, seed 1 | `cmu` | pass | 132 | 60.2 | 0.46 | 9% | 0.54 |
| | `ego` | pass | 166 | 57.5 | 0.35 | 1% | 0.48 |
| Forest, seed 2 | `cmu` | pass | 128 | 58.3 | 0.46 | 7% | 0.57 |
| | `ego` | pass | 172 | 58.3 | 0.34 | 2% | 0.64 |
| Forest, seed 3 | `cmu` | pass | 123 | 59.1 | 0.48 | 4% | 0.66 |
| | `ego` | pass | 172 | 57.6 | 0.33 | 2% | 0.59 |
| Forest, 400 pillars | `cmu` | pass | 171 | 68.2 | 0.40 | 14% | 0.32 |
| | `ego` | fail: 15 contact steps | 184 | 61.1 | 0.33 | 3% | 0.28 |

In these runs the CMU planner finished 7% to 42% sooner and passed all
eight; it also stands still more (4% to 14% of the time, not investigated),
passes closer to the furniture in the house, and detours more in the dense
forest. EGO's two failures were each seen once: its 400-pillar run was
repeated once for the video below and passed without a contact (196 s
against 161 s for the CMU planner), and the noise run was not repeated. So
the failures may be chance. No fall in any run.

Side by side, at twice the speed, from one more run each at `344c2b8`:
[house](https://github.com/1nrobotics/star-nav/releases/download/site-assets/lite3-house-cmu-vs-ego-344c2b8.mp4)
(155 s against 181 s),
[forest with 400 pillars](https://github.com/1nrobotics/star-nav/releases/download/site-assets/lite3-dense-forest-cmu-vs-ego-344c2b8.mp4).

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
- MPPI: Williams et al., Information-Theoretic MPC, T-RO 2018, [arXiv 1707.02342](https://arxiv.org/abs/1707.02342); Log-MPPI, [arXiv 2203.16599](https://arxiv.org/abs/2203.16599); Smooth-MPPI, [arXiv 2112.09988](https://arxiv.org/abs/2112.09988); Biased-MPPI, [arXiv 2401.09241](https://arxiv.org/abs/2401.09241); [nav2_mppi_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_mppi_controller)
- FAR Planner, [arXiv 2110.09460](https://arxiv.org/abs/2110.09460); Falco, [CMU RI](https://ri.cmu.edu/?p=125590)
- FFI, [Development of off-road autonomy for UGVs](https://www.ffi.no/en/publications-archive/development-of-off-road-autonomy-for-unmanned-ground-vehicles)
