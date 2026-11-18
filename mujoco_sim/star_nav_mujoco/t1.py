"""Booster T1 humanoid walking with its upstream velocity policy.

The robot model and the policy come from Booster Robotics' booster_gym and
are downloaded on first use (``star-nav-mujoco --fetch-t1``); they are not
part of this repository. Both are Apache-2.0:

* MJCF and meshes: github.com/BoosterRobotics/booster_gym
  (``resources/T1/T1_locomotion.xml``, 12 actuated leg joints).
* ``T1.pt``: the trained actor (``deploy/models``), with the observation,
  gait clock, action scaling and PD gains of ``play_mujoco.py`` and
  ``envs/T1.yaml``.

``T1.pt`` is a TorchScript MLP (47 -> 256 -> 128 -> 128 -> 12, ELU). Its
weights are read straight from the archive's tensor files, without unpickling
anything, and the network runs in numpy: no PyTorch needed.

The policy follows a body-frame velocity command (forward, sideways, turn
rate) like the Lite3's, so the same ``Tracker`` steers it.
"""

from __future__ import annotations

import hashlib
import math
import os
import pathlib
import urllib.request
import zipfile

import mujoco
import numpy as np

from star_nav_mujoco import scene
from star_nav_mujoco.lite3 import MissingAssets
from star_nav_mujoco.lite3 import Tracker  # noqa: F401  the same steering

# Pinned upstream revision.
REPO = 'BoosterRobotics/booster_gym'
COMMIT = 'da396a06d6eed99e2de72d7749c48ee8748950f9'
MODEL_XML = 'resources/T1/T1_locomotion.xml'
MESHES = (
    'Trunk', 'H1', 'H2', 'AL1', 'AL2', 'AL3', 'left_hand_link', 'AR1', 'AR2',
    'AR3', 'right_hand_link', 'Waist', 'Hip_Pitch_Left', 'Hip_Roll_Left',
    'Hip_Yaw_Left', 'Shank_Left', 'Ankle_Cross_Left', 'left_foot_link',
    'Hip_Pitch_Right', 'Hip_Roll_Right', 'Hip_Yaw_Right', 'Shank_Right',
    'Ankle_Cross_Right', 'right_foot_link')
POLICY_PATH = 'deploy/models/T1.pt'
POLICY_SHA256 = '697cac61dbed2e76f25519c97ceb686f244d065dabf7d5d7ca386573143d215b'
WEIGHTS_FILE = 'policy.npz'  # T1.pt's weights, extracted by fetch()
# Layer shapes of the actor, in the order of the archive's tensor files.
LAYERS = ((256, 47), (128, 256), (128, 128), (12, 128))

PREFIX = 't1/'
TRUNK = PREFIX + 'Trunk'
CAMERA = PREFIX + 'depth'
START_HEIGHT = 0.72  # trunk height it is dropped from (envs/T1.yaml)
STAND_HEIGHT = 0.67  # trunk height when standing
TIMESTEP = 0.002
DECIMATION = 10  # policy every 20 ms (50 Hz)
GAIT_FREQUENCY = 1.5  # steps per second per leg (Hz), the training mean

# Joint order of the robot and the policy (same), and the standing pose the
# scaled actions are added to.
JOINTS = ('Left_Hip_Pitch', 'Left_Hip_Roll', 'Left_Hip_Yaw', 'Left_Knee_Pitch',
          'Left_Ankle_Pitch', 'Left_Ankle_Roll',
          'Right_Hip_Pitch', 'Right_Hip_Roll', 'Right_Hip_Yaw',
          'Right_Knee_Pitch', 'Right_Ankle_Pitch', 'Right_Ankle_Roll')
DEFAULT_POSE = np.array([-0.2, 0.0, 0.0, 0.4, -0.25, 0.0] * 2, np.float32)
KP = np.array([200.0, 200.0, 200.0, 200.0, 50.0, 50.0] * 2, np.float32)
KD = np.array([5.0, 5.0, 5.0, 5.0, 1.0, 1.0] * 2, np.float32)
MAX_CMD = 1.0  # the training command range (m/s, rad/s)
JOINT_VEL_SCALE = 0.1

# Sensors on the trunk (body frame: x forward, z up): the depth camera in the
# face, the lidar on top of the head (1.2 m above the ground).
CAMERA_POS = (0.15, 0.0, 0.40)
LIDAR_MOUNT = (0.07, 0.0, 0.53)
# Collision geoms of the legs: touching the ground, a ramp or a step with
# these is walking, not a collision (see Simulator._touching).
LEG_GEOMS = ('SHANK_collision', 'FOOT_collision')
# The house's walls are taller than the robot (1.16 m).
HOUSE_WALL_HEIGHT = 1.6


def asset_dir() -> pathlib.Path:
  env = os.environ.get('STAR_NAV_T1_DIR')
  if env:
    return pathlib.Path(env).expanduser()
  return pathlib.Path('~/.cache/star_nav/t1').expanduser()


def check_assets(root: pathlib.Path | None = None) -> pathlib.Path:
  root = root or asset_dir()
  missing = [p for p in (root / MODEL_XML, root / WEIGHTS_FILE) if not p.exists()]
  if missing:
    raise MissingAssets(
        f'T1 files not found in {root}. Download them once with '
        '"star-nav-mujoco --fetch-t1" (see mujoco_sim/README.md).')
  return root


def _download(url: str, dest: pathlib.Path) -> None:
  dest.parent.mkdir(parents=True, exist_ok=True)
  tmp = dest.with_suffix(dest.suffix + '.part')
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, 'wb') as f:
    while chunk := r.read(1 << 20):
      f.write(chunk)
  tmp.replace(dest)


def extract_weights(archive: pathlib.Path) -> dict[str, np.ndarray]:
  """Reads the actor's weights from a TorchScript archive (no unpickling).

  The archive stores each tensor as raw little-endian float32 in
  ``<name>/data/<i>``, in parameter order: weight, bias of each layer.
  """
  out = {}
  with zipfile.ZipFile(archive) as z:
    names = z.namelist()
    top = names[0].split('/')[0]
    if z.read(f'{top}/byteorder').strip() != b'little':
      raise ValueError(f'{archive}: not little-endian')
    for i, (rows, cols) in enumerate(LAYERS):
      for kind, shape, idx in (('w', (rows, cols), 2 * i), ('b', (rows,), 2 * i + 1)):
        raw = z.read(f'{top}/data/{idx}')
        if len(raw) != 4 * math.prod(shape):
          raise ValueError(f'{archive}: tensor {idx} has {len(raw)} bytes, '
                           f'expected {shape} float32')
        out[f'{kind}{i}'] = np.frombuffer(raw, '<f4').reshape(shape).copy()
  return out


def fetch(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned model, meshes and policy into ``root``."""
  root = root or asset_dir()
  raw = f'https://raw.githubusercontent.com/{REPO}/{COMMIT}'
  log(f'Downloading the T1 model ({REPO}@{COMMIT[:7]}) to {root}')
  _download(f'{raw}/{MODEL_XML}', root / MODEL_XML)
  mesh_dir = (root / MODEL_XML).parent / 'meshes'
  for name in MESHES:
    _download(f'{raw}/resources/T1/meshes/{name}.STL', mesh_dir / f'{name}.STL')
  log(f'Downloading {POLICY_PATH}')
  policy = root / 'T1.pt'
  _download(f'{raw}/{POLICY_PATH}', policy)
  digest = hashlib.sha256(policy.read_bytes()).hexdigest()
  if digest != POLICY_SHA256:
    policy.unlink()
    raise RuntimeError(f'T1.pt: unexpected sha256 {digest}')
  np.savez(root / WEIGHTS_FILE, **extract_weights(policy))
  (root / 'SOURCE.md').write_text(
      'Downloaded by star_nav_mujoco.t1.fetch; not redistributed.\n\n'
      f'- {MODEL_XML}, meshes/, T1.pt: https://github.com/{REPO}/tree/{COMMIT}'
      ' (Apache-2.0, Booster Robotics)\n'
      f'- {WEIGHTS_FILE}: the weights of T1.pt\n')
  log(f'T1: model, {len(MESHES)} meshes and the policy are in {root}')
  return root


def attach(spec: mujoco.MjSpec, start: tuple[float, float], root: pathlib.Path,
           lidar: bool = False) -> None:
  """Adds the T1, standing at ``start`` (x, y), to ``spec``.

  Adds the depth camera (``scene.CAM_*``) in the face and, with ``lidar``, a
  (visual-only) lidar housing below ``LIDAR_MOUNT``. Its collision geoms move
  to ``scene.ROBOT_GROUP`` and its visual meshes to ``scene.MARKER_GROUP``, so
  the onboard sensors do not see the robot. All names get the ``t1/`` prefix.
  """
  child = mujoco.MjSpec.from_file(str(root / MODEL_XML))
  child.njmax = child.nconmax = -1  # the scene's (default) sizes
  trunk = child.body('Trunk')
  trunk.add_camera(name='depth', pos=CAMERA_POS, xyaxes=[0, -1, 0, 0, 0, 1],
                   fovy=scene.CAM_FOVY,
                   resolution=[scene.CAM_WIDTH, scene.CAM_HEIGHT])
  if lidar:
    trunk.add_geom(name='lidar', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                   pos=LIDAR_MOUNT[:2] + (LIDAR_MOUNT[2] - 0.03,),
                   size=[0.035, 0.02, 0], rgba=[0.2, 0.25, 0.3, 1],
                   contype=0, conaffinity=0, group=scene.MARKER_GROUP)
  for geom in child.geoms:
    if geom.contype or geom.conaffinity:
      geom.group = scene.ROBOT_GROUP
      body = geom.parent.name
      if body.endswith('foot_link'):
        geom.name = body + '_' + LEG_GEOMS[1]
      elif body.startswith('Shank'):
        geom.name = body + '_' + LEG_GEOMS[0]
    else:
      geom.group = scene.MARKER_GROUP
  frame = spec.worldbody.add_frame(pos=[start[0], start[1], 0.0])
  frame.attach_body(trunk, PREFIX, '')


class Walker:
  """Runs the velocity policy (numpy) and the joint PD controller."""

  def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
               root: pathlib.Path | None = None):
    root = root or asset_dir()
    self.model, self.data = model, data
    acts = [PREFIX + j for j in JOINTS]
    if [model.actuator(i).name for i in range(model.nu)] != acts:
      raise ValueError('unexpected T1 actuator order')
    jids = model.actuator_trnid[:, 0]
    self._qpos = model.jnt_qposadr[jids]
    self._qvel = model.jnt_dofadr[jids]
    self._trunk = model.body(TRUNK).id
    self._root_qvel = model.jnt_dofadr[model.body_jntadr[self._trunk]]
    self._lo = model.actuator_ctrlrange[:, 0].copy()
    self._hi = model.actuator_ctrlrange[:, 1].copy()
    with np.load(root / WEIGHTS_FILE) as w:
      self._layers = [(w[f'w{i}'], w[f'b{i}']) for i in range(len(LAYERS))]
    self.twist = np.zeros(3, np.float32)
    self.reset()

  def reset(self) -> None:
    self.data.qpos[self._qpos] = DEFAULT_POSE
    self._last = np.zeros(len(JOINTS), np.float32)
    self._target = DEFAULT_POSE.copy()
    self._steps = 0
    self._phase = 0.0

  def _policy(self, x: np.ndarray) -> np.ndarray:
    for i, (w, b) in enumerate(self._layers):
      x = w @ x + b
      if i < len(self._layers) - 1:
        x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))  # ELU
    return x

  @property
  def walking(self) -> bool:
    """False while the command is zero: the gait clock stops, it stands."""
    return bool(np.any(self.twist))

  def observation(self) -> np.ndarray:
    d = self.data
    q = d.xquat[self._trunk]
    gravity = np.empty(3)
    mujoco.mju_rotVecQuat(gravity, np.array([0.0, 0.0, -1.0]),
                          np.array([q[0], -q[1], -q[2], -q[3]]))
    omega = d.qvel[self._root_qvel + 3:self._root_qvel + 6]  # body frame
    on = float(self.walking)
    clock = [math.cos(2 * math.pi * self._phase) * on,
             math.sin(2 * math.pi * self._phase) * on]
    return np.concatenate([
        gravity, omega, np.clip(self.twist, -MAX_CMD, MAX_CMD), clock,
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
      action = np.clip(self._policy(self.observation()), -1.0, 1.0)
      self._last = action.astype(np.float32)
      self._target = DEFAULT_POSE + self._last
    self._steps += 1
    if self.walking:
      self._phase = math.fmod(self._phase + self.model.opt.timestep * GAIT_FREQUENCY, 1.0)
    torque = KP * (self._target - d.qpos[self._qpos]) - KD * d.qvel[self._qvel]
    d.ctrl[:] = np.clip(torque, self._lo, self._hi)
