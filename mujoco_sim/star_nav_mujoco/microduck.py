"""Microduck biped (Pollen Robotics) walking with its velstand policy.

The robot model, the walking policy and the actuator model come from upstream
and are downloaded on first use (``star-nav-mujoco --fetch-microduck``); they
are not part of this repository:

* MJCF and meshes: github.com/pollen-robotics/microduck_rl (MJCF Apache-2.0,
  meshes CC BY-NC-SA 4.0, i.e. non-commercial).
* ``velstand.onnx``: huggingface.co/pollen-robotics/microduck-policies
  (Apache-2.0). One network walks and stands; it runs at 50 Hz on
  ``[gyro, gravity, joint pos, joint vel, last action, command]``.
* Actuators: the BAM model of the XL330 servos (github.com/Rhoban/bam,
  Apache-2.0), the same model the policy was trained against.

The planner's setpoints are turned into a walking command by ``Tracker``:
the policy follows forward speed and turn rate, not sideways steps, so the
duck is steered like a unicycle.
"""

from __future__ import annotations

import dataclasses
import hashlib
import math
import os
import pathlib
import re
import urllib.request

import mujoco
import numpy as np

from star_nav_mujoco import scene

# Pinned upstream revisions.
MODEL_REPO = 'pollen-robotics/microduck_rl'
MODEL_COMMIT = '0daab160d3fb1c2cbaba16a4a19052f36709dd9e'
MODEL_DIR = 'src/mjlab_microduck/robot/microduck'
MODEL_XML = 'robot_groundcontact_backlash.xml'  # the training variant
POLICY_REPO = 'pollen-robotics/microduck-policies'
POLICY_REVISION = '236d1835b03ac7b45d962efbf2fe3a6940206112'  # tag v10
POLICY_FILE = 'velstand.onnx'
POLICY_SHA256 = 'ce894d30b3343036d499bfac1102bdaa58d59a21c1b58401ecb70fb6a2f63e0a'
BAM_REQUIREMENT = ('better-actuator-models @ git+https://github.com/Rhoban/bam.git'
                   '@62bd8ce12154340be97e06f7f41a0ca8f116d967')

PREFIX = 'duck/'
TRUNK = PREFIX + 'trunk_base'
CAMERA = PREFIX + 'depth'
TIMESTEP = 0.005  # physics; the policy runs every DECIMATION steps (50 Hz)
DECIMATION = 4
STAND_HEIGHT = 0.125  # trunk height of the standing pose (m)
VISUAL_GROUP = 2  # upstream puts the duck's visual meshes in group 2

# Joint order of the policy, and the standing pose its actions are added to.
JOINTS = ('left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
          'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
          'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch', 'right_knee',
          'right_ankle')
DEFAULT_POSE = np.array([
    0.0, -0.0873, -0.4579, -0.0049, 0.4530, 0.3491, 0.3491, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530], np.float32)

# BAM M6 model of the XL330 as used in training (infer_policy.py upstream).
BAM_KP = 200.0
BAM_VIN = 7.4
BAM_VIN_DROP_GAIN = 0.1
BAM_VIN_MIN = 6.0
BAM_SOLREF_FRICTION = (-5.0e4, -2.0e2)
BAM_SOLIMP_FRICTION = (0.99, 0.9999, 0.001, 0.5, 2.0)

# The real robot's camera looks along the +x axis of the head_camera site,
# which is 9 mm behind the face shell; the depth camera sits just outside.
CAMERA_SITE = 'head_camera'
CAMERA_FORWARD = 0.012


class MissingAssets(RuntimeError):
  pass


def asset_dir() -> pathlib.Path:
  env = os.environ.get('STAR_NAV_MICRODUCK_DIR')
  if env:
    return pathlib.Path(env).expanduser()
  return pathlib.Path('~/.cache/star_nav/microduck').expanduser()


def check_assets(root: pathlib.Path | None = None) -> pathlib.Path:
  root = root or asset_dir()
  missing = [p for p in (root / MODEL_XML, root / POLICY_FILE) if not p.exists()]
  if missing:
    raise MissingAssets(
        f'Microduck files not found in {root}. Download them once with '
        '"star-nav-mujoco --fetch-microduck" (see mujoco_sim/README.md).')
  try:
    import bam  # noqa: F401  pylint: disable=g-import-not-at-top,unused-import
    import onnxruntime  # noqa: F401  pylint: disable=g-import-not-at-top,unused-import
  except ImportError as e:
    raise MissingAssets(
        f'{e.name} is not installed; run ./scripts/setup_mujoco_env.sh '
        '--microduck (see mujoco_sim/README.md).') from e
  return root


def _download(url: str, dest: pathlib.Path) -> None:
  tmp = dest.with_suffix(dest.suffix + '.part')
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, 'wb') as f:
    while chunk := r.read(1 << 20):
      f.write(chunk)
  tmp.replace(dest)


def fetch(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned model, meshes and policy into ``root``."""
  root = root or asset_dir()
  (root / 'assets').mkdir(parents=True, exist_ok=True)
  raw = f'https://raw.githubusercontent.com/{MODEL_REPO}/{MODEL_COMMIT}/{MODEL_DIR}'
  log(f'Downloading the Microduck model ({MODEL_REPO}@{MODEL_COMMIT[:7]}) to {root}')
  _download(f'{raw}/{MODEL_XML}', root / MODEL_XML)
  meshes = sorted(set(re.findall(r'file="([^"]+\.stl)"', (root / MODEL_XML).read_text())))
  for name in meshes:
    if not (root / 'assets' / name).exists():
      _download(f'{raw}/assets/{name}', root / 'assets' / name)
  log(f'Downloading {POLICY_FILE} ({POLICY_REPO}@{POLICY_REVISION[:7]})')
  policy = root / POLICY_FILE
  _download(f'https://huggingface.co/{POLICY_REPO}/resolve/{POLICY_REVISION}/{POLICY_FILE}',
            policy)
  digest = hashlib.sha256(policy.read_bytes()).hexdigest()
  if digest != POLICY_SHA256:
    policy.unlink()
    raise RuntimeError(f'{POLICY_FILE}: unexpected sha256 {digest}')
  (root / 'SOURCE.md').write_text(
      f'Downloaded by star_nav_mujoco.microduck.fetch; not redistributed.\n\n'
      f'- {MODEL_XML}, assets/: https://github.com/{MODEL_REPO}/tree/{MODEL_COMMIT}/{MODEL_DIR}\n'
      f'  MJCF: Apache-2.0. Meshes (*.stl): CC BY-NC-SA 4.0, (c) Pollen Robotics,\n'
      f'  non-commercial use only. See LICENSE-MODELS.md in that repository.\n'
      f'- {POLICY_FILE}: https://huggingface.co/{POLICY_REPO} at {POLICY_REVISION}\n'
      f'  Apache-2.0.\n')
  log(f'Microduck: {len(meshes)} meshes and the policy are in {root}')
  return root


def attach(spec: mujoco.MjSpec, start: tuple[float, float], root: pathlib.Path) -> None:
  """Adds the duck, standing at ``start`` (x, y), to ``spec``.

  The servos become BAM torque motors (as in training) and the same depth
  camera as the quadrotor's (``scene.CAM_*``) is added at the head camera.
  All names get the ``duck/`` prefix.
  """
  child = mujoco.MjSpec.from_file(str(root / MODEL_XML))
  child.meshdir = str(root / 'assets')
  for key in list(child.keys):
    child.delete(key)

  site = child.site(CAMERA_SITE)
  rot = np.zeros(9)
  mujoco.mju_quat2Mat(rot, site.quat)
  rot = rot.reshape(3, 3)  # columns: site axes in the head frame
  # MuJoCo cameras look along -z with +y up: -z = site x, +y = site z.
  site.parent.add_camera(
      name='depth', pos=site.pos + CAMERA_FORWARD * rot[:, 0],
      xyaxes=np.concatenate([-rot[:, 1], rot[:, 2]]), fovy=scene.CAM_FOVY,
      resolution=[scene.CAM_WIDTH, scene.CAM_HEIGHT])

  from bam.model import load_model  # pylint: disable=g-import-not-at-top
  bam = load_model(motor_name='xl330', model='m6')
  force = BAM_VIN * bam.kt.value / bam.R.value
  joints = {j.name: j for j in child.joints}
  for act in child.actuators:
    if act.target.startswith('passive_'):
      continue
    act.set_to_motor()
    act.forcelimited = True
    act.forcerange = (-force, force)
    act.ctrllimited = False
    act.gear = [1.0, 0, 0, 0, 0, 0]
    joint = joints[act.target]
    joint.damping = np.zeros((3, 1))
    joint.frictionloss = 0.0
    joint.solref_friction = BAM_SOLREF_FRICTION
    joint.solimp_friction = BAM_SOLIMP_FRICTION

  frame = spec.worldbody.add_frame(pos=[start[0], start[1], 0.0])
  frame.attach_body(child.body('trunk_base'), PREFIX, '')


class Walker:
  """Runs the velstand policy and the BAM servo model."""

  def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
               root: pathlib.Path | None = None):
    import onnxruntime  # pylint: disable=g-import-not-at-top
    from bam.model import load_model  # pylint: disable=g-import-not-at-top
    from bam.mujoco import MujocoController  # pylint: disable=g-import-not-at-top

    root = root or asset_dir()
    self.model, self.data = model, data
    acts = [PREFIX + j for j in JOINTS]
    if [model.actuator(i).name for i in range(model.nu)] != acts:
      raise ValueError('unexpected Microduck actuator order')
    bam = load_model(motor_name='xl330', model='m6')
    bam.actuator.kp = BAM_KP
    bam.actuator.vin = BAM_VIN
    bam.actuator.max_current = None
    self._servos = MujocoController(bam, acts, model, data,
                                    vin_drop_gain=BAM_VIN_DROP_GAIN,
                                    vin_min=BAM_VIN_MIN)
    jids = model.actuator_trnid[:, 0]
    self._qpos = model.jnt_qposadr[jids]
    self._qvel = model.jnt_dofadr[jids]
    self._trunk = model.body(TRUNK).id
    self._gyro = model.sensor_adr[model.sensor(PREFIX + 'imu_ang_vel').id]

    opts = onnxruntime.SessionOptions()
    opts.intra_op_num_threads = 1
    self._policy = onnxruntime.InferenceSession(
        str(root / POLICY_FILE), opts, providers=['CPUExecutionProvider'])
    self._input = self._policy.get_inputs()[0].name
    self.twist = np.zeros(3, np.float32)
    self.reset()

  def reset(self) -> None:
    d = self.data
    d.qpos[self._qpos] = DEFAULT_POSE
    self._servos.reset(d.qpos)
    self._servos.q_target[:] = DEFAULT_POSE
    self._last = np.zeros(len(JOINTS), np.float32)
    self._steps = 0

  def observation(self) -> np.ndarray:
    d = self.data
    q = d.xquat[self._trunk]
    gravity = np.empty(3)
    mujoco.mju_rotVecQuat(gravity, np.array([0.0, 0.0, -1.0]),
                          np.array([q[0], -q[1], -q[2], -q[3]]))
    command = np.zeros(13, np.float32)  # twist, head pose, body pose
    command[:3] = self.twist
    return np.concatenate([
        d.sensordata[self._gyro:self._gyro + 3], gravity,
        d.qpos[self._qpos] - DEFAULT_POSE, d.qvel[self._qvel], self._last,
        command]).astype(np.float32)

  @property
  def policy_due(self) -> bool:
    """True if the next ``before_step`` runs the policy (set ``twist`` now)."""
    return self._steps % DECIMATION == 0

  def before_step(self) -> None:
    """Call before every mj_step: runs the policy every DECIMATION steps."""
    if self._steps % DECIMATION == 0:
      action = self._policy.run(None, {self._input: self.observation()[None]})[0][0]
      self._last = action.astype(np.float32)
      self._servos.q_target[:] = DEFAULT_POSE + self._last
    self._steps += 1
    self._servos.update()


@dataclasses.dataclass
class Tracker:
  """Turns a position setpoint into (forward speed, turn rate) commands.

  Measured on the policy (flat ground): it does not step below a forward
  command of ~0.25, walks at about ``0.69 * cmd - 0.09`` m/s above that, does
  not side-step, and turns in place only for a turn command of ~1.5
  (0.7 rad/s). So far-off headings are corrected in place and speeds are
  mapped through the inverse of that line.

  The duck cannot walk slower than ~0.1 m/s. When the planner's setpoint moves
  slower, the duck gets ahead of it; it then stands and waits for the setpoint
  to catch up instead of turning back.
  """
  k_pos: float = 1.0  # 1/s, position error to speed
  k_yaw: float = 1.5  # 1/s, heading error to turn command while walking
  speed_gain: float = 0.69
  speed_offset: float = 0.09
  min_walk_cmd: float = 0.3
  max_walk_cmd: float = 0.5
  max_turn_walking: float = 1.0
  turn_in_place: float = 1.5
  turn_in_place_above: float = math.radians(50)
  arrive: float = 0.08  # m: stand still this close to a resting setpoint
  wait_ahead: float = 0.3  # m: wait if this far ahead of the setpoint, on its path

  def twist(self, sp_pos, sp_vel, pos, yaw) -> np.ndarray:
    err = np.asarray(sp_pos[:2]) - pos[:2]
    sp_vel = np.asarray(sp_vel[:2], dtype=float)
    dist = float(np.hypot(*err))
    sp_speed = float(np.hypot(*sp_vel))
    if dist < self.arrive and sp_speed < 0.02:
      return np.zeros(3, np.float32)  # at a resting setpoint
    if sp_speed > 0.01 and dist < self.wait_ahead:
      path = sp_vel / sp_speed
      along = float(err @ path)
      across = abs(float(err[0] * path[1] - err[1] * path[0]))
      if along < 0.0 and across < self.arrive:
        return np.zeros(3, np.float32)  # ahead of the setpoint: wait
    vel = sp_vel + self.k_pos * err
    speed = float(np.hypot(*vel))
    heading = math.atan2(vel[1], vel[0])
    turn = math.remainder(heading - yaw, 2.0 * math.pi)
    if abs(turn) > self.turn_in_place_above:
      return np.array([0.0, 0.0, math.copysign(self.turn_in_place, turn)], np.float32)
    forward = np.clip((speed + self.speed_offset) / self.speed_gain,
                      self.min_walk_cmd, self.max_walk_cmd)
    rate = np.clip(self.k_yaw * turn, -self.max_turn_walking, self.max_turn_walking)
    return np.array([forward, 0.0, rate], np.float32)
