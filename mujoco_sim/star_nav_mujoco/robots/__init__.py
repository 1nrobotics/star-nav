"""The robots the simulator can run: one module here per robot.

A module describes its robot with a ``Robot`` object named ``ROBOT``: what it
is called, how it is added to the scene, what drives it and where its sensors
sit. ``Simulator`` only talks to that object, so adding a robot means adding
a module here (see docs/SIM_ARCHITECTURE.md), not editing the simulator.

Every robot follows the same command, the planner's position setpoint, with
its own ``Controller``. A ground robot's is a chain: a tracker
(``tracking.py``) turns the setpoint into a body-frame velocity, and the
robot's locomotion policy turns that into joint torques.

Importing this package does not import MuJoCo: the command line lists the
robots before it has chosen an OpenGL backend (``__main__._select_gl``).
"""

from __future__ import annotations

import importlib
import math
import pathlib
import pkgutil
from typing import Protocol

from star_nav_mujoco import scene
from star_nav_mujoco import tracking

DEFAULT = 'quadrotor'


class Controller(Protocol):
  """Follows the planner's command by writing the model's actuators."""

  def reset(self) -> None:
    """Puts the robot in its starting state (after mj_resetData)."""

  def step(self, cmd) -> None:
    """Called before every mj_step with the command to follow."""


class Locomotion(Protocol):
  """A ground robot's policy and joint control; follows ``twist``."""

  twist: object  # forward (m/s), sideways (m/s), turn rate (rad/s)
  policy_due: bool  # the next before_step runs the policy: set twist now

  def reset(self) -> None: ...

  def before_step(self) -> None: ...


class Robot:
  """What the simulator needs to know about a robot."""

  name = ''  # --robot value, module name and asset directory
  title = ''  # as written in messages
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
  follow_camera = (5.0, -25.0)  # viewer and recorder: distance (m), elevation
  house_wall_height = 1.0  # m

  def check_assets(self):
    """Raises ``assets.MissingAssets`` if the robot's files are not there."""

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
  ``locomotion`` (its policy), and chooses its ``tracker``.
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

  def attach(self, spec: mujoco.MjSpec, start: tuple[float, float],
             root: pathlib.Path, lidar: bool = False) -> None:
    raise NotImplementedError

  def locomotion(self, model, data, root: pathlib.Path) -> Locomotion:
    raise NotImplementedError

  def build(self, cfg, obstacles, extra: str) -> mujoco.MjSpec:
    import mujoco  # pylint: disable=g-import-not-at-top

    root = self.check_assets()
    spec = mujoco.MjSpec.from_string(scene.build_xml(
        obstacles, cfg.params, cfg.start, self.timestep, quadrotor_body=False,
        extra=extra))
    self.attach(spec, cfg.start[:2], root, lidar=cfg.lidar is not None)
    return spec

  def controller(self, model, data, cfg) -> Controller:
    return _GroundController(self, model, data)


class _GroundController:
  """Tracker, then the locomotion policy."""

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
    if loco.policy_due:
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


def find(model: mujoco.MjModel) -> Robot:
  """The robot in a compiled scene."""
  import mujoco  # pylint: disable=g-import-not-at-top

  for name in names():
    robot = get(name)
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot.body) >= 0:
      return robot
  raise ValueError('no known robot in the model')
