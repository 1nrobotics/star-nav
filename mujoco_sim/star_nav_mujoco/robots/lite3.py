"""DeepRobotics Lite3 quadruped walking with its upstream velocity policy.

The robot model and the policy come from DeepRobotics and are downloaded on
first use (``star-nav-mujoco --fetch lite3``); they are not part of this
repository. Both are BSD-3-Clause:

* MJCF and meshes: github.com/DeepRoboticsLab/deep_robotics_model
  (``Lite3/Lite3_mjcf``).
* ``policy.onnx``: github.com/DeepRoboticsLab/Lite3_rl_deploy
  (``policy/ppo``), with the observation, action scaling and PD gains of its
  ONNX runner (``run_policy/lite3_test_policy_runner_onnx.hpp``).

The policy follows a body-frame velocity command (forward, sideways, turn
rate), so ``tracking.Holonomic`` steers the robot in any direction towards
the planner's setpoint and turns it to the planner's yaw.
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

from star_nav_mujoco import assets
from star_nav_mujoco import locomotion
from star_nav_mujoco import robots
from star_nav_mujoco import scene
from star_nav_mujoco.assets import MissingAssets

# Pinned upstream revisions.
MODEL_REPO = 'DeepRoboticsLab/deep_robotics_model'
MODEL_COMMIT = 'b452a1f404d37155400242f18e454fed8bcc51b8'
MODEL_DIR = 'Lite3/Lite3_mjcf'
MODEL_XML = 'mjcf/Lite3.xml'
MESHES = ('hip.STL', 'shank.STL', 'thigh.STL', 'torso.STL')
POLICY_REPO = 'DeepRoboticsLab/Lite3_rl_deploy'
POLICY_COMMIT = '1c4a8f14c9c531903fa5a26783e3f9b99d1039d9'
POLICY_PATH = 'policy/ppo/policy.onnx'
POLICY_FILE = 'policy.onnx'
POLICY_SHA256 = 'efa6581d314d8c0442881453d39fc082c09b28bbaa3029d328fcd61fa25b57c7'

PREFIX = 'lite3/'
TRUNK = PREFIX + 'TORSO'
CAMERA = PREFIX + 'depth'
STAND_HEIGHT = 0.34  # torso height when standing (m)
START_HEIGHT = STAND_HEIGHT
TIMESTEP = 0.002  # physics; PD torques every step
DECIMATION = 6  # policy every 12 ms (83 Hz), as the upstream runner

# Joint order of the robot and the policy (same), and the standing pose the
# scaled actions are added to.
JOINTS = ('FL_HipX_joint', 'FL_HipY_joint', 'FL_Knee_joint',
          'FR_HipX_joint', 'FR_HipY_joint', 'FR_Knee_joint',
          'HL_HipX_joint', 'HL_HipY_joint', 'HL_Knee_joint',
          'HR_HipX_joint', 'HR_HipY_joint', 'HR_Knee_joint')
DEFAULT_POSE = np.array([0.0, -0.65, 1.3] * 4, np.float32)
ACTION_SCALE = np.array([0.125, 0.25, 0.25] * 4, np.float32)
KP, KD = 30.0, 1.0
TORQUE_LIMIT = 30.0  # N m, the model's ctrlrange (torques are clipped to it)
MAX_CMD = 0.8  # the runner clips each command to this (m/s, rad/s)
OMEGA_SCALE, JOINT_VEL_SCALE = 0.25, 0.05

# Sensors on the torso (body frame: x forward, z up): the depth camera at the
# front, the lidar on top.
CAMERA_POS = (0.29, 0.0, 0.02)
LIDAR_MOUNT = (0.0, 0.0, 0.10)
# Standing, with the lidar: length, width, height above the ground (m)
SIZE = (0.61, 0.39, 0.44)
# Collision geoms of the legs: touching the ground, a ramp or a step with
# these is walking, not a collision (see Simulator._touching).
SUPPORT_GEOMS = ('FOOT_collision', 'SHANK_collision')
HOUSE_WALL_HEIGHT = 1.0  # m; the robot sees and is seen over the walls


def asset_dir() -> pathlib.Path:
  return assets.directory('lite3')


def check_assets(root: pathlib.Path | None = None) -> pathlib.Path:
  root = root or asset_dir()
  missing = [p for p in (root / MODEL_XML, root / POLICY_FILE) if not p.exists()]
  if missing:
    raise MissingAssets(
        f'Lite3 files not found in {root}. Download them once with '
        '"star-nav-mujoco --fetch lite3" (see mujoco_sim/README.md).')
  try:
    import onnxruntime  # noqa: F401  pylint: disable=g-import-not-at-top,unused-import
  except ImportError as e:
    raise MissingAssets(
        'onnxruntime is not installed; run ./scripts/setup_mujoco_env.sh '
        '--lite3 (see mujoco_sim/README.md).') from e
  return root


def fetch(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned model, meshes and policy into ``root``."""
  root = root or asset_dir()
  raw = f'{assets.github(MODEL_REPO, MODEL_COMMIT)}/{MODEL_DIR}'
  log(f'Downloading the Lite3 model ({MODEL_REPO}@{MODEL_COMMIT[:7]}) to {root}')
  assets.download(f'{raw}/{MODEL_XML}', root / MODEL_XML)
  for name in MESHES:
    assets.download(f'{raw}/meshes/{name}', root / 'meshes' / name)
  log(f'Downloading {POLICY_PATH} ({POLICY_REPO}@{POLICY_COMMIT[:7]})')
  assets.download(
      f'{assets.github(POLICY_REPO, POLICY_COMMIT)}/{POLICY_PATH}',
      root / POLICY_FILE, POLICY_SHA256)
  (root / 'SOURCE.md').write_text(
      'Downloaded by star_nav_mujoco.robots.lite3.fetch; not redistributed.\n\n'
      f'- {MODEL_XML}, meshes/: https://github.com/{MODEL_REPO}/tree/'
      f'{MODEL_COMMIT}/{MODEL_DIR} (BSD-3-Clause, DeepRoboticsLab)\n'
      f'- {POLICY_FILE}: https://github.com/{POLICY_REPO}/blob/{POLICY_COMMIT}/'
      f'{POLICY_PATH} (BSD-3-Clause, DeepRoboticsLab)\n')
  log(f'Lite3: model, {len(MESHES)} meshes and the policy are in {root}')
  return root

# The upstream meshes are one flat grey. Per part (the visual geoms'
# <part>_visual names): rgba, specular, shininess. The STL meshes have no
# texture coordinates, so these are plain materials.
APPEARANCE = {
    'lite3_torso': ([0.17, 0.18, 0.20, 1.0], 0.5, 0.6),  # dark shell
    'lite3_hip': ([0.70, 0.72, 0.75, 1.0], 0.8, 0.8),  # bare motor housings
    'lite3_thigh': ([0.22, 0.23, 0.26, 1.0], 0.5, 0.6),
    'lite3_shank': ([0.60, 0.62, 0.65, 1.0], 0.7, 0.7),
    'lite3_foot': ([0.06, 0.06, 0.06, 1.0], 0.1, 0.1),  # rubber
}


def attach(spec: mujoco.MjSpec, start: tuple[float, float], root: pathlib.Path,
           lidar: bool = False) -> None:
  """Adds the Lite3, standing at ``start`` (x, y), to ``spec``.

  Adds the depth camera (``scene.CAM_*``) at the front of the torso and, with
  ``lidar``, a (visual-only) lidar housing below ``LIDAR_MOUNT``. Its
  collision geoms move to ``scene.ROBOT_GROUP`` so the onboard sensors do not
  see the robot's legs; its visual geoms are in ``scene.MARKER_GROUP`` already.
  All names get the ``lite3/`` prefix.
  """
  child = mujoco.MjSpec.from_file(str(root / MODEL_XML))
  torso = child.body('TORSO')
  torso.add_camera(name='depth', pos=CAMERA_POS, xyaxes=[0, -1, 0, 0, 0, 1],
                   fovy=scene.CAM_FOVY,
                   resolution=[scene.CAM_WIDTH, scene.CAM_HEIGHT])
  if lidar:
    torso.add_geom(name='lidar', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   pos=LIDAR_MOUNT[:2] + (LIDAR_MOUNT[2] - 0.03,),
                   size=[0.035, 0.02, 0], rgba=[0.2, 0.25, 0.3, 1],
                   contype=0, conaffinity=0, group=scene.MARKER_GROUP)
  for name, (rgba, specular, shininess) in APPEARANCE.items():
    child.add_material(name=name, rgba=rgba, specular=specular,
                       shininess=shininess)
  for geom in child.geoms:
    if geom.conaffinity or geom.contype:
      geom.group = scene.ROBOT_GROUP
    elif geom.name.endswith('_visual'):
      part = geom.name[:-len('_visual')].rsplit('_', 1)[-1]
      geom.material = 'lite3_' + part.lower()
  frame = spec.worldbody.add_frame(pos=[start[0], start[1], 0.0])
  frame.attach_body(torso, PREFIX, '')


class Locomotion(locomotion.JointPolicy):
  """The upstream velocity policy (ONNX), with the runner's observation."""

  title = 'Lite3'
  trunk = TRUNK
  actuators = tuple(PREFIX + j + '_ctrl' for j in JOINTS)
  default_pose = DEFAULT_POSE
  action_scale = ACTION_SCALE
  kp, kd = KP, KD
  decimation = DECIMATION

  def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
               root: pathlib.Path | None = None):
    import onnxruntime  # pylint: disable=g-import-not-at-top

    opts = onnxruntime.SessionOptions()
    opts.intra_op_num_threads = 1
    self._policy = onnxruntime.InferenceSession(
        str((root or asset_dir()) / POLICY_FILE), opts,
        providers=['CPUExecutionProvider'])
    super().__init__(model, data)

  def observation(self) -> np.ndarray:
    return np.concatenate([
        self.omega() * OMEGA_SCALE, self.gravity(),
        np.clip(self.twist, -MAX_CMD, MAX_CMD), self.joint_pos(),
        self.joint_vel() * JOINT_VEL_SCALE, self.last]).astype(np.float32)

  def act(self, obs: np.ndarray) -> np.ndarray:
    return self._policy.run(None, {'obs': obs[None]})[0][0]


class Lite3(robots.GroundRobot):
  name = 'lite3'
  title = 'Lite3'
  type = 'legged'
  body = TRUNK
  camera = CAMERA
  timestep = TIMESTEP
  size = SIZE
  stand_height = STAND_HEIGHT
  start_height = START_HEIGHT
  support_geoms = SUPPORT_GEOMS
  lidar_mount = LIDAR_MOUNT
  lidar_vfov = (-30.0, 52.0)  # the floor from 0.75 m ahead, and the walls
  house_wall_height = HOUSE_WALL_HEIGHT
  check_assets = staticmethod(check_assets)
  fetch = staticmethod(fetch)
  attach = staticmethod(attach)
  locomotion = Locomotion


ROBOT = Lite3()
