"""DeepRobotics Lite3 quadruped walking with its upstream velocity policy.

The robot model and the policy come from DeepRobotics and are downloaded on
first use (``star-nav-mujoco --fetch-lite3``); they are not part of this
repository. Both are BSD-3-Clause:

* MJCF and meshes: github.com/DeepRoboticsLab/deep_robotics_model
  (``Lite3/Lite3_mjcf``).
* ``policy.onnx``: github.com/DeepRoboticsLab/Lite3_rl_deploy
  (``policy/ppo``), with the observation, action scaling and PD gains of its
  ONNX runner (``run_policy/lite3_test_policy_runner_onnx.hpp``).

The policy follows a body-frame velocity command (forward, sideways, turn
rate), so ``Tracker`` steers the robot omnidirectionally towards the
planner's setpoint and turns it to the planner's yaw.
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import os
import pathlib
import urllib.request

import mujoco
import numpy as np

from star_nav_mujoco import scene

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
TORQUE_LIMIT = 30.0  # N m, the model's ctrlrange
MAX_CMD = 0.8  # the runner clips each command to this (m/s, rad/s)
OMEGA_SCALE, JOINT_VEL_SCALE = 0.25, 0.05

# Sensors on the torso (body frame: x forward, z up): the depth camera at the
# front, the lidar on top.
CAMERA_POS = (0.29, 0.0, 0.02)
LIDAR_MOUNT = (0.0, 0.0, 0.10)


class MissingAssets(RuntimeError):
  pass


def asset_dir() -> pathlib.Path:
  env = os.environ.get('STAR_NAV_LITE3_DIR')
  if env:
    return pathlib.Path(env).expanduser()
  return pathlib.Path('~/.cache/star_nav/lite3').expanduser()


def check_assets(root: pathlib.Path | None = None) -> pathlib.Path:
  root = root or asset_dir()
  missing = [p for p in (root / MODEL_XML, root / POLICY_FILE) if not p.exists()]
  if missing:
    raise MissingAssets(
        f'Lite3 files not found in {root}. Download them once with '
        '"star-nav-mujoco --fetch-lite3" (see mujoco_sim/README.md).')
  try:
    import onnxruntime  # noqa: F401  pylint: disable=g-import-not-at-top,unused-import
  except ImportError as e:
    raise MissingAssets(
        'onnxruntime is not installed; run ./scripts/setup_mujoco_env.sh '
        '--lite3 (see mujoco_sim/README.md).') from e
  return root


def _download(url: str, dest: pathlib.Path) -> None:
  dest.parent.mkdir(parents=True, exist_ok=True)
  tmp = dest.with_suffix(dest.suffix + '.part')
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, 'wb') as f:
    while chunk := r.read(1 << 20):
      f.write(chunk)
  tmp.replace(dest)


def fetch(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned model, meshes and policy into ``root``."""
  root = root or asset_dir()
  raw = f'https://raw.githubusercontent.com/{MODEL_REPO}/{MODEL_COMMIT}/{MODEL_DIR}'
  log(f'Downloading the Lite3 model ({MODEL_REPO}@{MODEL_COMMIT[:7]}) to {root}')
  _download(f'{raw}/{MODEL_XML}', root / MODEL_XML)
  for name in MESHES:
    _download(f'{raw}/meshes/{name}', root / 'meshes' / name)
  log(f'Downloading {POLICY_PATH} ({POLICY_REPO}@{POLICY_COMMIT[:7]})')
  policy = root / POLICY_FILE
  _download(f'https://raw.githubusercontent.com/{POLICY_REPO}/{POLICY_COMMIT}/{POLICY_PATH}',
            policy)
  digest = hashlib.sha256(policy.read_bytes()).hexdigest()
  if digest != POLICY_SHA256:
    policy.unlink()
    raise RuntimeError(f'{POLICY_FILE}: unexpected sha256 {digest}')
  (root / 'SOURCE.md').write_text(
      'Downloaded by star_nav_mujoco.lite3.fetch; not redistributed.\n\n'
      f'- {MODEL_XML}, meshes/: https://github.com/{MODEL_REPO}/tree/'
      f'{MODEL_COMMIT}/{MODEL_DIR} (BSD-3-Clause, DeepRoboticsLab)\n'
      f'- {POLICY_FILE}: https://github.com/{POLICY_REPO}/blob/{POLICY_COMMIT}/'
      f'{POLICY_PATH} (BSD-3-Clause, DeepRoboticsLab)\n')
  log(f'Lite3: model, {len(MESHES)} meshes and the policy are in {root}')
  return root


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
  for geom in child.geoms:
    if geom.conaffinity or geom.contype:
      geom.group = scene.ROBOT_GROUP
  frame = spec.worldbody.add_frame(pos=[start[0], start[1], 0.0])
  frame.attach_body(torso, PREFIX, '')


class Walker:
  """Runs the velocity policy and the joint PD controller."""

  def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
               root: pathlib.Path | None = None):
    import onnxruntime  # pylint: disable=g-import-not-at-top

    root = root or asset_dir()
    self.model, self.data = model, data
    acts = [PREFIX + j + '_ctrl' for j in JOINTS]
    if [model.actuator(i).name for i in range(model.nu)] != acts:
      raise ValueError('unexpected Lite3 actuator order')
    jids = model.actuator_trnid[:, 0]
    self._qpos = model.jnt_qposadr[jids]
    self._qvel = model.jnt_dofadr[jids]
    self._trunk = model.body(TRUNK).id
    self._root_qvel = model.jnt_dofadr[model.body_jntadr[self._trunk]]

    opts = onnxruntime.SessionOptions()
    opts.intra_op_num_threads = 1
    self._policy = onnxruntime.InferenceSession(
        str(root / POLICY_FILE), opts, providers=['CPUExecutionProvider'])
    self.twist = np.zeros(3, np.float32)
    self.reset()

  def reset(self) -> None:
    self.data.qpos[self._qpos] = DEFAULT_POSE
    self._last = np.zeros(len(JOINTS), np.float32)
    self._target = DEFAULT_POSE.copy()
    self._steps = 0

  def observation(self) -> np.ndarray:
    d = self.data
    q = d.xquat[self._trunk]
    gravity = np.empty(3)
    mujoco.mju_rotVecQuat(gravity, np.array([0.0, 0.0, -1.0]),
                          np.array([q[0], -q[1], -q[2], -q[3]]))
    omega = d.qvel[self._root_qvel + 3:self._root_qvel + 6]  # body frame
    return np.concatenate([
        omega * OMEGA_SCALE, gravity, np.clip(self.twist, -MAX_CMD, MAX_CMD),
        d.qpos[self._qpos] - DEFAULT_POSE, d.qvel[self._qvel] * JOINT_VEL_SCALE,
        self._last]).astype(np.float32)

  @property
  def policy_due(self) -> bool:
    """True if the next ``before_step`` runs the policy (set ``twist`` now)."""
    return self._steps % DECIMATION == 0

  def before_step(self) -> None:
    """Call before every mj_step: policy every DECIMATION steps, PD always."""
    d = self.data
    if self._steps % DECIMATION == 0:
      action = self._policy.run(None, {'obs': self.observation()[None]})[0][0]
      self._last = action.astype(np.float32)
      self._target = DEFAULT_POSE + self._last * ACTION_SCALE
    self._steps += 1
    torque = KP * (self._target - d.qpos[self._qpos]) - KD * d.qvel[self._qvel]
    d.ctrl[:] = np.clip(torque, -TORQUE_LIMIT, TORQUE_LIMIT)


@dataclasses.dataclass
class Tracker:
  """Turns a position setpoint (and yaw) into a body-frame velocity command.

  Measured on flat ground (policy at 83 Hz): forward 0.5 -> 0.49 m/s,
  sideways 0.3 -> 0.24 m/s, turn 0.5 -> 0.45 rad/s, standing still on a zero
  command. It walks in any direction, so it follows the setpoint's velocity
  plus a position correction, and turns towards the planner's yaw.
  """
  k_pos: float = 1.5  # 1/s, position error to velocity
  k_yaw: float = 1.5  # 1/s, yaw error to turn rate
  max_speed: float = 0.8  # m/s, the policy's command limit
  max_turn: float = 0.8  # rad/s
  hold: float = 0.03  # m: stand within this of a resting setpoint
  hold_yaw: float = math.radians(3)

  def twist(self, sp_pos, sp_vel, sp_yaw, pos, yaw) -> np.ndarray:
    err = np.asarray(sp_pos[:2], float) - pos[:2]
    v = np.asarray(sp_vel[:2], float)
    vel = v + self.k_pos * err
    if float(np.hypot(*v)) < 0.02 and float(np.hypot(*err)) < self.hold:
      vel = np.zeros(2)
    speed = float(np.hypot(*vel))
    if speed > self.max_speed:
      vel *= self.max_speed / speed
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, side = c * vel[0] + s * vel[1], -s * vel[0] + c * vel[1]
    yaw_err = math.remainder(sp_yaw - yaw, 2.0 * math.pi)
    turn = 0.0 if abs(yaw_err) < self.hold_yaw else float(
        np.clip(self.k_yaw * yaw_err, -self.max_turn, self.max_turn))
    return np.array([fwd, side, turn], np.float32)
