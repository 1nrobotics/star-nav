"""The quadrotor: built in ``scene.py``, flown by ``quadrotor.Controller``."""

from __future__ import annotations

import mujoco

from star_nav_mujoco import quadrotor
from star_nav_mujoco import robots
from star_nav_mujoco import scene


class _Flight:
  """The SE(3) controller's rotor thrusts, and the body's drag."""

  def __init__(self, model, data, cfg, body: int):
    self._model, self._data, self._body = model, data, body
    self._params = cfg.params
    self._controller = quadrotor.Controller(cfg.params, cfg.gains)

  def reset(self) -> None:
    hover = self._params.mass * quadrotor.GRAVITY / 4.0
    self._data.ctrl[:] = hover
    self._data.act[:] = hover
    self._controller.reset()

  def step(self, cmd) -> None:
    d = self._data
    vel = d.qvel[0:3]
    d.ctrl[:] = self._controller.update(
        cmd, d.qpos[0:3], d.xmat[self._body].reshape(3, 3), vel, d.qvel[3:6],
        self._model.opt.timestep)
    d.xfrc_applied[self._body, 0:3] = -self._params.drag * vel


class Quadrotor(robots.Robot):
  name = 'quadrotor'
  title = 'Quadrotor'
  body = 'quad'
  camera = 'depth'
  rings = 50

  def build(self, cfg, obstacles, extra: str) -> mujoco.MjSpec:
    lidar_mount = cfg.lidar.mount if cfg.lidar else None
    return mujoco.MjSpec.from_string(scene.build_xml(
        obstacles, cfg.params, cfg.start, cfg.timestep,
        lidar_mount=lidar_mount, extra=extra))

  def controller(self, model, data, cfg) -> robots.Controller:
    return _Flight(model, data, cfg, model.body(self.body).id)


ROBOT = Quadrotor()
