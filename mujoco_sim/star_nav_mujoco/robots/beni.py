"""Mondo Robotics' Beni, a small two-wheeled balancing robot, with its policy.

The robot model and the policy come from Mondo Robotics' open-beni and are
downloaded on first use (``star-nav-mujoco --fetch beni``); they are not part
of this repository. Both are MIT:

* MJCF, meshes and textures: github.com/Mondo-Robotics/open-beni
  (``assets/beni_base``). Two legs (hip ``L1``/``R1``, knee ``L2``/``R2``),
  a wheel on each (``LW``/``RW``) and a head that turns; 1.3 kg, the base
  0.11 m above the ground.
* ``policy.onnx`` (``assets/models``): the trained actor, with the
  observation, action scaling, motor model and timing of its training code
  (``python/beni_rl``) and browser runner (``web/game/physics.mjs``).

The policy is a GRU (40 -> 512) followed by an MLP (512 -> 512 -> 256 -> 128
-> 7, ELU). Its weights are read from the ONNX file (``policy.py``) and the
network runs in numpy: no onnxruntime needed.

The policy balances the robot, follows a forward speed and a turn rate, and
gets back onto its wheels after a fall. It cannot move sideways, so
``tracking.Unicycle`` steers it. Jumping (the policy's fourth command
group) is not used.
"""

from __future__ import annotations

import collections
import dataclasses
import math
import pathlib

import mujoco
import numpy as np

from star_nav_mujoco import assets
from star_nav_mujoco import policy as policy_lib
from star_nav_mujoco import robots
from star_nav_mujoco import scene
from star_nav_mujoco import tracking
from star_nav_mujoco.assets import MissingAssets

# Pinned upstream revision.
REPO = 'Mondo-Robotics/open-beni'
COMMIT = '4545b86a131d05519050df11a424368ef9f06b4f'
MODEL_DIR = 'assets/beni_base'
MODEL_XML = MODEL_DIR + '/mjcf/beni.xml'
MESHES = (
    'VIS_base_link_00_BASE_VIS', 'VIS_head_00_HEAD_VIS',
    'VIS_L1_Link_00_L1_VIS', 'VIS_L2_Link_00_L1_VIS', 'VIS_LW_Link_00_LW_VIS',
    'VIS_R1_Link_00_R1_VIS', 'VIS_R2_Link_00_R2_VIS', 'VIS_RW_Link_00_RW_VIS')
TEXTURES = ('beni_white.png', 'tex_beni_emissive.png', 'tex_beni_normal.png',
            'tex_beni_orm.png')
POLICY_PATH = 'assets/models/policy.onnx'
POLICY_FILE = 'policy.onnx'
POLICY_SHA256 = '7b13d7934a2abd1a8adc41aa0377571194f8595e5d311ae40845d3f85a79b014'
WEIGHTS_FILE = 'policy.npz'  # policy.onnx's weights, extracted by fetch()
OBS_DIM, HIDDEN, ACTION_DIM = 40, 512, 7
# Initializers of the ONNX graph: our name, shape.
WEIGHTS = {
    'obs_normalizer._mean': ('mean', (1, OBS_DIM)),
    'onnx::Div_97': ('std', (1, OBS_DIM)),
    'onnx::GRU_115': ('gru_w', (1, 3 * HIDDEN, OBS_DIM)),
    'onnx::GRU_116': ('gru_r', (1, 3 * HIDDEN, HIDDEN)),
    'onnx::GRU_117': ('gru_b', (1, 6 * HIDDEN)),
    'mlp.0.weight': ('w0', (512, HIDDEN)), 'mlp.0.bias': ('b0', (512,)),
    'mlp.2.weight': ('w1', (256, 512)), 'mlp.2.bias': ('b1', (256,)),
    'mlp.4.weight': ('w2', (128, 256)), 'mlp.4.bias': ('b2', (128,)),
    'mlp.6.weight': ('w3', (ACTION_DIM, 128)), 'mlp.6.bias': ('b3', (ACTION_DIM,)),
}

PREFIX = 'beni/'
TRUNK = PREFIX + 'base_link'
CAMERA = PREFIX + 'depth'
START_HEIGHT = 0.14  # base height it is dropped from (beni_robot.py)
STAND_HEIGHT = 0.11  # base height when balancing
TIMESTEP = 0.002
DECIMATION = 5  # policy every 10 ms (100 Hz)
ACTION_DELAY = 3  # physics steps from the policy to the motors (training: 2-5)

# The policy's outputs, and the joints it observes (positions: the first
# five; velocities: all).
ACTION_JOINTS = ('L1', 'R1', 'L2', 'R2', 'LW', 'RW', 'head_yaw_joint')
OBS_JOINTS = ('L1', 'R1', 'head_yaw_joint', 'L2', 'R2', 'LW', 'RW')
WHEELS = ('LW', 'RW')
HEAD_ROLL = 'head_roll_joint'  # follows the base; not driven by the policy
DEFAULT_POSE = {'L1': 0.3, 'R1': 0.3, 'L2': 0.5, 'R2': 0.5}


@dataclasses.dataclass(frozen=True)
class Motor:
  """PD motor (beni_robot.py); wheels follow a speed, the others an angle."""
  kp: float
  kd: float
  limit: float  # N m, motor torque
  scale: float = 0.0  # action to angle (rad) or wheel speed (rad/s)
  total_limit: float | None = None  # with the knee spring
  armature: float = 0.0
  frictionloss: float = 0.0


MOTORS = {
    'L1': Motor(0.5, 0.02, 1.375, 0.25, None, 0.0001625, 0.02),
    'L2': Motor(0.5, 0.02, 3.6, 0.25, 4.6, 0.0005375, 0.02),
    'LW': Motor(0.0, 0.004, 0.6, 20.0, None, 0.0, 0.005),
    'head_yaw_joint': Motor(3.0, 0.05, 0.1, 0.5),
    HEAD_ROLL: Motor(2.0, 0.0, 1.0),
}
MOTORS.update(R1=MOTORS['L1'], R2=MOTORS['L2'], RW=MOTORS['LW'])

# Commands (training: forward -2..2 m/s, turn -5..5 rad/s). The limits here
# keep it well inside; the ramp is the browser runner's.
MAX_SPEED = 1.5
MAX_TURN = 3.0
CMD_RAMP = (0.03, 0.06)  # per policy step: 3 m/s^2, 6 rad/s^2
BASE_POSE_CMD = (0.0, 0.0, 0.11)  # pitch, roll, height asked of the policy
GIMBAL_CMD = (0.0, 1.0)  # head yaw 0, enabled

# Sensors on the base (body frame: x forward, z up): the depth camera in the
# head, the lidar on top of it (0.22 m above the ground).
CAMERA_POS = (0.07, 0.0, 0.08)
LIDAR_MOUNT = (0.0, 0.0, 0.14)
# Collision geoms of the wheels: touching the ground or a ramp with these is
# driving, not a collision (see Simulator._touching).
SUPPORT_GEOMS = ('LW_collision', 'RW_collision')
HOUSE_WALL_HEIGHT = 1.0


def asset_dir() -> pathlib.Path:
  return assets.directory('beni')


def check_assets(root: pathlib.Path | None = None) -> pathlib.Path:
  root = root or asset_dir()
  missing = [p for p in (root / MODEL_XML, root / WEIGHTS_FILE) if not p.exists()]
  if missing:
    raise MissingAssets(
        f'Beni files not found in {root}. Download them once with '
        '"star-nav-mujoco --fetch beni" (see mujoco_sim/README.md).')
  return root


def extract_weights(onnx: pathlib.Path) -> dict[str, np.ndarray]:
  """Reads the actor's weights from the ONNX file (float32 initializers)."""
  out = {}
  for name, (dims, dtype, raw) in policy_lib.onnx_initializers(onnx).items():
    if name not in WEIGHTS:
      continue
    key, shape = WEIGHTS[name]
    if dtype != 1 or raw is None or dims != shape:
      raise ValueError(f'{onnx}: {name} is {dims} (type {dtype}), expected '
                       f'{shape} float32')
    out[key] = np.frombuffer(raw, '<f4').reshape(shape).copy()
  missing = [name for name, (key, _) in WEIGHTS.items() if key not in out]
  if missing:
    raise ValueError(f'{onnx}: missing {", ".join(missing)}')
  return out


def fetch(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned model, meshes, textures and policy into ``root``."""
  root = root or asset_dir()
  raw = assets.github(REPO, COMMIT)
  log(f'Downloading the Beni model ({REPO}@{COMMIT[:7]}) to {root}')
  assets.download(f'{raw}/{MODEL_XML}', root / MODEL_XML)
  for name in MESHES:
    path = f'{MODEL_DIR}/meshes/{name}.obj'
    assets.download(f'{raw}/{path}', root / path)
  for name in TEXTURES:
    path = f'{MODEL_DIR}/textures/{name}'
    assets.download(f'{raw}/{path}', root / path)
  log(f'Downloading {POLICY_PATH}')
  policy = root / POLICY_FILE
  assets.download(f'{raw}/{POLICY_PATH}', policy, POLICY_SHA256)
  np.savez(root / WEIGHTS_FILE, **extract_weights(policy))
  (root / 'SOURCE.md').write_text(
      'Downloaded by star_nav_mujoco.robots.beni.fetch; not redistributed.\n\n'
      f'- {MODEL_DIR}/, {POLICY_FILE}: https://github.com/{REPO}/tree/{COMMIT}'
      ' (MIT, Open Beni)\n'
      f'- {WEIGHTS_FILE}: the weights of {POLICY_FILE}\n')
  log(f'Beni: model, {len(MESHES)} meshes and the policy are in {root}')
  return root


def attach(spec: mujoco.MjSpec, start: tuple[float, float], root: pathlib.Path,
           lidar: bool = False) -> None:
  """Adds Beni, on its wheels at ``start`` (x, y), to ``spec``.

  The model is set up as its training code does (``get_beni_spec``): joint
  angles count from the model's pose, the model's actuators are replaced by
  one torque motor per joint (the PD loops run in ``Locomotion``), and the wheels'
  friction takes priority over the floor's.

  Adds the depth camera (``scene.CAM_*``) in the head and, with ``lidar``, a
  (visual-only) lidar housing below ``LIDAR_MOUNT``. Its collision geoms are
  in ``scene.ROBOT_GROUP`` already; its visual meshes move to
  ``scene.MARKER_GROUP``, so the onboard sensors do not see the robot. All
  names get the ``beni/`` prefix.
  """
  child = mujoco.MjSpec.from_file(str(root / MODEL_XML))
  for actuator in tuple(child.actuators):
    child.delete(actuator)
  for name in (*ACTION_JOINTS, HEAD_ROLL):
    motor = MOTORS[name]
    joint = child.joint(name)
    joint.ref = 0.0
    joint.actfrclimited = mujoco.mjtLimited.mjLIMITED_FALSE
    joint.armature = motor.armature
    joint.frictionloss = motor.frictionloss
    limit = motor.total_limit or motor.limit
    child.add_actuator(name=name, target=name,
                       trntype=mujoco.mjtTrn.mjTRN_JOINT,
                       ctrllimited=True, ctrlrange=[-limit, limit])
  base = child.body('base_link')
  base.add_camera(name='depth', pos=CAMERA_POS, xyaxes=[0, -1, 0, 0, 0, 1],
                  fovy=scene.CAM_FOVY,
                  resolution=[scene.CAM_WIDTH, scene.CAM_HEIGHT])
  if lidar:
    base.add_geom(name='lidar', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                  pos=LIDAR_MOUNT[:2] + (LIDAR_MOUNT[2] - 0.015,),
                  size=[0.02, 0.012, 0], rgba=[0.2, 0.25, 0.3, 1],
                  contype=0, conaffinity=0, group=scene.MARKER_GROUP)
  for geom in child.geoms:
    if geom.contype or geom.conaffinity:
      geom.group = scene.ROBOT_GROUP
      body = geom.parent.name
      if body in ('LW_Link', 'RW_Link'):
        geom.name = body[:2] + '_collision'
        geom.priority = 1
    else:
      geom.group = scene.MARKER_GROUP
  frame = spec.worldbody.add_frame(pos=[start[0], start[1], 0.0])
  frame.attach_body(base, PREFIX, '')


def knee_spring(angle: float) -> float:
  """Torque (N m) of the passive spring at the knee (actuator.py)."""
  a = angle - 1.0
  return 0.22 * math.sin(a) + 0.15 * math.sin(2.0 * a)


class Locomotion:
  """Runs the policy (numpy) and the motors' PD loops.

  ``twist`` is (forward m/s, unused, turn rad/s).
  """

  def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
               root: pathlib.Path | None = None):
    root = root or asset_dir()
    self.model, self.data = model, data
    names = (*ACTION_JOINTS, HEAD_ROLL)
    self._act = np.array([model.actuator(PREFIX + n).id for n in names])
    joints = [model.joint(PREFIX + n) for n in names]
    self._qpos = np.array([j.qposadr[0] for j in joints])
    self._qvel = np.array([j.dofadr[0] for j in joints])
    order = [names.index(n) for n in OBS_JOINTS]
    self._obs_qpos, self._obs_qvel = self._qpos[order[:5]], self._qvel[order]
    self._obs_default = np.array(
        [DEFAULT_POSE.get(n, 0.0) for n in OBS_JOINTS[:5]])
    motors = [MOTORS[n] for n in names]
    self._kp = np.array([m.kp for m in motors])
    self._kd = np.array([m.kd for m in motors])
    self._limit = np.array([m.limit for m in motors])
    self._total = np.array([m.total_limit or m.limit for m in motors])
    self._scale = np.array([m.scale for m in motors])
    self._default = np.array([DEFAULT_POSE.get(n, 0.0) for n in names])
    self._wheel = np.array([n in WHEELS for n in names])
    self._knees = [names.index(n) for n in ('L2', 'R2')]
    self._gyro = model.sensor(PREFIX + 'imu_gyro')
    self._head_gyro = model.sensor(PREFIX + 'head_imu_gyro')
    self._base = model.body(TRUNK).id
    with np.load(root / WEIGHTS_FILE) as w:
      self._w = {k: w[k].astype(np.float64) for k in w.files}
    self.twist = np.zeros(3, np.float32)
    self.reset()

  def reset(self) -> None:
    self.data.qpos[self._qpos] = self._default
    self._hidden = np.zeros(HIDDEN)
    self._last = np.zeros(ACTION_DIM)
    self._delayed = collections.deque(
        [np.zeros(ACTION_DIM)] * ACTION_DELAY, maxlen=ACTION_DELAY + 1)
    self._cmd = np.zeros(2)  # forward, turn: the ramped command
    self._steps = 0

  def _policy(self, obs: np.ndarray) -> np.ndarray:
    w = self._w
    x = (obs - w['mean'][0]) / w['std'][0]
    self._hidden = policy_lib.gru(w['gru_w'][0], w['gru_r'][0], w['gru_b'][0],
                                  x, self._hidden)
    return policy_lib.mlp([(w[f'w{i}'], w[f'b{i}']) for i in range(4)],
                          self._hidden)

  def observation(self) -> np.ndarray:
    d = self.data
    gravity = policy_lib.projected_gravity(d.xquat[self._base])
    gyro = d.sensordata[self._gyro.adr[0]:self._gyro.adr[0] + 3]
    head = d.sensordata[self._head_gyro.adr[0]:self._head_gyro.adr[0] + 3]
    return np.concatenate([
        gyro, gravity, [self._cmd[0], 0.0, self._cmd[1]],
        d.qpos[self._obs_qpos] - self._obs_default, d.qvel[self._obs_qvel],
        self._last, BASE_POSE_CMD, GIMBAL_CMD, head,
        np.zeros(4)])  # no jump

  @property
  def policy_due(self) -> bool:
    """True if the next ``before_step`` runs the policy (set ``twist`` now)."""
    return self._steps % DECIMATION == 0

  def before_step(self) -> None:
    """Call before every mj_step: policy every DECIMATION steps, PD always."""
    m, d = self.model, self.data
    if self._steps % DECIMATION == 0:
      want = (float(np.clip(self.twist[0], -MAX_SPEED, MAX_SPEED)),
              float(np.clip(self.twist[2], -MAX_TURN, MAX_TURN)))
      for i, ramp in enumerate(CMD_RAMP):
        self._cmd[i] += np.clip(want[i] - self._cmd[i], -ramp, ramp)
      if self._steps:
        # The gyros are sensors: bring them up to date with qpos and qvel.
        mujoco.mj_forward(m, d)
      self._last = self._policy(self.observation())
    self._steps += 1
    self._delayed.append(self._last)
    action = np.append(self._delayed.popleft(), 0.0) * self._scale  # + head roll
    q, v = d.qpos[self._qpos], d.qvel[self._qvel]
    target_q = self._default + np.where(self._wheel, 0.0, action)
    target_v = np.where(self._wheel, action, 0.0)
    torque = np.clip(self._kp * (target_q - q) + self._kd * (target_v - v),
                     -self._limit, self._limit)
    for k in self._knees:
      torque[k] += knee_spring(q[k])
    d.ctrl[self._act] = np.clip(torque, -self._total, self._total)


class Beni(robots.GroundRobot):
  name = 'beni'
  title = 'Beni'
  body = TRUNK
  camera = CAMERA
  timestep = TIMESTEP
  stand_height = STAND_HEIGHT
  start_height = START_HEIGHT
  support_geoms = SUPPORT_GEOMS
  lidar_mount = LIDAR_MOUNT
  lidar_vfov = (-30.0, 52.0)
  follow_camera = (2.0, -40.0)  # 0.2 m tall
  house_wall_height = HOUSE_WALL_HEIGHT
  tracker = tracking.Unicycle
  check_assets = staticmethod(check_assets)
  fetch = staticmethod(fetch)
  attach = staticmethod(attach)
  locomotion = Locomotion


ROBOT = Beni()
