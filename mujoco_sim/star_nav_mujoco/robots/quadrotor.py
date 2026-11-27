"""The quadrotor: its model, written here in MJCF, and its flight controller.

``quadrotor.py`` has its parameters and the SE(3) controller; this module
puts the body and its four rotors into the scene and runs the controller on
them.
"""

from __future__ import annotations

import mujoco

from star_nav_mujoco import commands
from star_nav_mujoco import quadrotor
from star_nav_mujoco import robots
from star_nav_mujoco import scene

_ASSETS = '''<material name="frame" rgba="0.15 0.15 0.18 1"/>
    <material name="prop" rgba="0.9 0.4 0.1 0.8"/>
    <material name="lidar" rgba="0.2 0.25 0.3 1"/>'''
_DEFAULTS = '''<default class="drone">
      <geom contype="2" conaffinity="1" mass="0"/>
    </default>'''


def _body_xml(p: quadrotor.Params, start: tuple[float, float, float],
              lidar_mount: tuple[float, float, float] | None) -> str:
  f = scene.fmt
  rotors = []
  for i, (x, y, _) in enumerate(p.rotor_layout()):
    rotors.append(
        f'<site name="rotor{i}" pos="{f(x, y, 0.02)}"/>\n'
        f'      <geom class="drone" type="cylinder" material="prop" '
        f'pos="{f(x, y, 0.025)}" size="{f(p.prop_radius, 0.004)}"/>\n'
        f'      <geom class="drone" type="capsule" material="frame" '
        f'fromto="0 0 0 {f(x, y, 0)}" size="0.008"/>'
    )
  lidar = ''
  if lidar_mount is not None:
    lidar = (f'<geom class="drone" type="cylinder" material="lidar" '
             f'pos="{f(*lidar_mount)}" size="0.035 0.02"/>')
  return f'''
    <body name="quad" pos="{f(*start)}">
      <freejoint name="quad"/>
      <inertial pos="0 0 0" mass="{p.mass}" diaginertia="{f(*p.inertia)}"/>
      <geom class="drone" type="box" material="frame" size="0.06 0.045 0.02"/>
      {chr(10).join('      ' + r for r in rotors).strip()}
      {lidar}
      <camera name="depth" pos="{scene.CAM_OFFSET} 0 0" xyaxes="0 -1 0 0 0 1"
              fovy="{scene.CAM_FOVY:.4f}" resolution="{scene.CAM_WIDTH} {scene.CAM_HEIGHT}"/>
      <camera name="chase" mode="trackcom" pos="-3 -1.5 1.4" xyaxes="0.45 -0.89 0 0.3 0.15 0.94"/>
    </body>'''


def _actuators_xml(p: quadrotor.Params) -> str:
  out = []
  for i, (_, _, spin) in enumerate(p.rotor_layout()):
    out.append(
        f'<general name="motor{i}" site="rotor{i}" '
        f'gear="0 0 1 0 0 {spin * p.torque_coeff:.4g}" '
        f'dyntype="filter" dynprm="{p.motor_tau}" '
        f'ctrlrange="0 {p.max_thrust}" ctrllimited="true"/>'
    )
  return '\n    '.join(out)


class _Flight:
  """The SE(3) controller's rotor thrusts, and the body's drag."""

  accepts = (commands.Setpoint,)

  def __init__(self, model, data, params: quadrotor.Params,
               gains: quadrotor.Gains, body: int):
    self._model, self._data, self._body = model, data, body
    self._params = params
    self._controller = quadrotor.Controller(params, gains)

  def reset(self) -> None:
    hover = self._params.mass * quadrotor.GRAVITY / 4.0
    self._data.ctrl[:] = hover
    self._data.act[:] = hover
    self._controller.reset()

  def step(self, cmd: commands.Setpoint) -> None:
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
  params = quadrotor.Params()
  gains = quadrotor.Gains()

  def parts(self, start: tuple[float, float, float],
            lidar_mount: tuple[float, float, float] | None = None
            ) -> scene.Parts:
    """The quadrotor at ``start``; with ``lidar_mount``, a lidar housing."""
    return scene.Parts(
        body=_body_xml(self.params, start, lidar_mount),
        actuators=_actuators_xml(self.params),
        assets=_ASSETS, defaults=_DEFAULTS)

  def build(self, cfg, obstacles, extra: str) -> mujoco.MjSpec:
    lidar_mount = cfg.lidar.mount if cfg.lidar else None
    return mujoco.MjSpec.from_string(scene.build_xml(
        obstacles, cfg.start, cfg.timestep, extra=extra, draw=cfg.draw,
        robot=self.parts(cfg.start, lidar_mount)))

  def controller(self, model, data, cfg) -> robots.Controller:
    return _Flight(model, data, self.params, self.gains,
                   model.body(self.body).id)


ROBOT = Quadrotor()
