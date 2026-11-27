"""The robots the simulator can run: one module here per robot.

A module describes its robot with a ``Robot`` object named ``ROBOT``: what it
is called, how it is added to the scene, what drives it and where its sensors
sit. ``Simulator`` only talks to that object, so adding a robot means adding
a module here (see docs/SIM_ARCHITECTURE.md), not editing the simulator.

A planner commands a robot with a position setpoint or a body-frame velocity
(``commands.py``), and the robot's ``Controller`` says which of the two it
follows. Every robot follows a setpoint. A ground robot's controller is a
chain: a tracker (``tracking.py``) turns the setpoint into a body-frame
velocity, and the robot's locomotion policy turns that into joint torques.
It also follows a velocity from a planner that steers the robot itself; that
skips the tracker.

The planner's side (the launch files) needs a robot's size, where its lidar
sits and how fast it goes. ``Robot.describe()`` gives those, and
``description()`` writes them as the file the launch files load:
star_nav_bringup/config/robots/<name>.yaml.

Importing this package does not import MuJoCo: the command line lists the
robots before it has chosen an OpenGL backend (``__main__._select_gl``).
"""

from __future__ import annotations

import importlib
import math
import pathlib
import pkgutil
from typing import Protocol

from star_nav_mujoco import commands
from star_nav_mujoco import scene
from star_nav_mujoco import tracking

DEFAULT = 'quadrotor'


class Controller(Protocol):
  """Follows the planner's command by writing the model's actuators."""

  # The commands (classes of commands.py) it follows; the simulator refuses
  # any other.
  accepts: tuple[type, ...]

  def reset(self) -> None:
    """Puts the robot in its starting state (after mj_resetData)."""

  def step(self, cmd) -> None:
    """Called before every mj_step with the command to follow.

    ``cmd`` is of one of the classes in ``accepts``.
    """


class Locomotion(Protocol):
  """A ground robot's policy and joint control; follows ``twist``.

  ``locomotion.JointPolicy`` implements it for a robot with a learned policy
  and joint PD (the walking robots).
  """

  twist: object  # forward (m/s), sideways (m/s), turn rate (rad/s)
  policy_due: bool  # the next before_step runs the policy: set twist now

  def reset(self) -> None: ...

  def before_step(self) -> None: ...


class Robot:
  """What the simulator needs to know about a robot."""

  name = ''  # --robot value, module name and asset directory
  title = ''  # as written in messages
  # The planner's robot_type (docs/PLANNER_INTERFACE.md): aerial or legged.
  type = ''
  body = ''  # root body (a free joint: qpos[0:7] is its pose)
  camera = ''  # depth camera
  on_ground = False  # drives or walks: touching the ground is not a contact
  # Defaults of the command line.
  world = 'forest'
  rings = 0  # forest: rings to fly through
  start = (0.0, 0.0, 1.0)
  lidar_vfov = (-22.5, 22.5)  # deg
  # Lidar position in the body frame; None: LidarConfig.mount.
  lidar_mount: tuple[float, float, float] | None = None
  # Lidar tilted nose-down (deg), by lidar model (lidar_models); a model not
  # listed is mounted level.
  lidar_pitch: dict[str, float] = {}
  # The box around the robot with its lidar, standing or level: length
  # (along x), width and height (m). tests/test_robots.py measures the model.
  size = (0.0, 0.0, 0.0)
  follow_camera = (5.0, -25.0)  # viewer and recorder: distance (m), elevation
  house_wall_height = 1.0  # m

  def check_assets(self):
    """Raises ``assets.MissingAssets`` if the robot's files are not there."""

  def mount_lidar(self, cfg):
    """A ``lidar.LidarConfig`` as this robot carries it."""
    import dataclasses  # pylint: disable=g-import-not-at-top

    mount = cfg.mount if self.lidar_mount is None else self.lidar_mount
    pitch = cfg.pitch
    if pitch is None:
      pitch = self.lidar_pitch.get(cfg.model, 0.0)
    return dataclasses.replace(cfg, mount=mount, pitch=pitch)

  def describe(self) -> dict:
    """What the planner's side needs to know about the robot.

    Lengths in m; ``lidar_mount`` is in the body frame.
    """
    from star_nav_mujoco import lidar  # pylint: disable=g-import-not-at-top

    length, width, height = self.size
    mount = self.mount_lidar(lidar.LidarConfig()).mount
    return {'name': self.name, 'type': self.type, 'length': length,
            'width': width, 'height': height, 'lidar_mount': list(mount)}

  # Downloads the robot's files: fetch(log=print). None: nothing to download.
  fetch = None

  def build(self, cfg, obstacles, extra: str) -> mujoco.MjSpec:
    """Returns the scene (``scene.build_xml``) with the robot in it."""
    raise NotImplementedError

  def controller(self, model, data, cfg) -> Controller:
    raise NotImplementedError


class GroundRobot(Robot):
  """A robot on the ground, driven by a learned velocity policy.

  A subclass gives the constants, ``attach`` (adds the robot to a scene) and
  ``locomotion`` (its policy: a ``Locomotion``, for a walking robot a
  subclass of ``locomotion.JointPolicy``), and chooses its ``tracker``.
  """

  on_ground = True
  world = 'house'
  timestep = 0.002  # physics step the policy was trained with (s)
  stand_height = 0.0  # root body height when standing (m)
  start_height = 0.0  # ... when it is put into the scene
  # Collision geoms (name suffixes) it stands on: touching the ground, a ramp
  # or a step with these is not a contact (see Simulator._touching).
  support_geoms: tuple[str, ...] = ()
  follow_camera = (3.2, -50.0)
  tracker = tracking.Holonomic

  @property
  def start(self):
    return (0.0, 0.0, self.stand_height)

  def describe(self) -> dict:
    """Also the height its body stands at and the tracker's limits (m/s,
    rad/s): the fastest a command can make it go."""
    tracker = self.tracker()
    return {**super().describe(), 'stand_height': self.stand_height,
            'max_speed': tracker.max_speed, 'max_turn': tracker.max_turn}

  def attach(self, spec: mujoco.MjSpec, start: tuple[float, float],
             root: pathlib.Path, lidar: bool = False) -> None:
    raise NotImplementedError

  def locomotion(self, model, data, root: pathlib.Path) -> Locomotion:
    raise NotImplementedError

  def build(self, cfg, obstacles, extra: str) -> mujoco.MjSpec:
    import mujoco  # pylint: disable=g-import-not-at-top

    root = self.check_assets()
    spec = mujoco.MjSpec.from_string(scene.build_xml(
        obstacles, cfg.start, self.timestep, extra=extra, draw=cfg.draw))
    self.attach(spec, cfg.start[:2], root, lidar=cfg.lidar is not None)
    return spec

  def controller(self, model, data, cfg) -> Controller:
    return _GroundController(self, model, data)


class _GroundController:
  """Tracker, then the locomotion policy."""

  accepts = (commands.Setpoint, commands.Velocity)

  def __init__(self, robot: GroundRobot, model, data):
    self._robot, self._data = robot, data
    self.locomotion = robot.locomotion(model, data, robot.check_assets())
    self.tracker = robot.tracker()

  def reset(self) -> None:
    self._data.qpos[2] = self._robot.start_height
    self.locomotion.reset()
    self.tracker = self._robot.tracker()

  def step(self, cmd) -> None:
    loco, d = self.locomotion, self._data
    if loco.policy_due and isinstance(cmd, commands.Velocity):
      loco.twist = self.tracker.limit(cmd.twist)
    elif loco.policy_due:
      yaw = 2.0 * math.atan2(d.qpos[6], d.qpos[3])  # the body stays level
      loco.twist = self.tracker.twist(cmd.pos, cmd.vel, cmd.yaw, d.qpos[0:3],
                                      yaw)
    loco.before_step()


def names() -> tuple[str, ...]:
  """The robots there are, the default first."""
  found = sorted(m.name for m in pkgutil.iter_modules(__path__))
  return (DEFAULT, *(n for n in found if n != DEFAULT))


def get(name: str) -> Robot:
  if name not in names():
    raise ValueError(f'unknown robot {name!r}')
  return importlib.import_module(f'{__name__}.{name}').ROBOT


def description(name: str) -> str:
  """``describe()`` of a robot as YAML: the text of its description file."""
  robot = get(name)

  def number(v):
    return repr(round(float(v), 4))

  facts = robot.describe()
  speeds = 'max_speed in m/s, max_turn in rad/s, ' if 'max_speed' in facts else ''
  lines = [
      f'# The {robot.title} as the planner\'s side needs to know it: lengths in m,',
      f'# {speeds}lidar_mount in the body frame.',
      '# The launch files load it as /robot, and',
      '# star_nav_bringup/test/test_planner_interface.py checks what they give the',
      '# planner against it.',
      '#',
      f'# Written from mujoco_sim/star_nav_mujoco/robots/{name}.py: do not edit.',
      '# Change the robot there, then (mujoco_sim/tests/test_robots.py checks it):',
      f'#   mujoco_sim/.venv/bin/python -m star_nav_mujoco --describe {name} \\',
      f'#     > star_nav_bringup/config/robots/{name}.yaml',
  ]
  for key, value in facts.items():
    if isinstance(value, str):
      text = value
    elif isinstance(value, list):
      text = '[' + ', '.join(number(v) for v in value) + ']'
    else:
      text = number(value)
    lines.append(f'{key}: {text}')
  return '\n'.join(lines) + '\n'


def find(model: mujoco.MjModel) -> Robot:
  """The robot in a compiled scene."""
  import mujoco  # pylint: disable=g-import-not-at-top

  for name in names():
    robot = get(name)
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot.body) >= 0:
      return robot
  raise ValueError('no known robot in the model')
