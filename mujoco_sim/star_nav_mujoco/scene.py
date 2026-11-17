"""Builds the MuJoCo scene (MJCF) for the forest world and the quadrotor."""

from __future__ import annotations

import math

from star_nav_mujoco import quadrotor
from star_nav_mujoco import world

# Depth camera, matching the intrinsics the planner is configured with.
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FOCAL = 387.229248046875
CAM_FOVY = math.degrees(2.0 * math.atan(CAM_HEIGHT / 2.0 / CAM_FOCAL))
CAM_OFFSET = 0.14  # metres ahead of the body centre

RING_SEGMENTS = 24
RING_TUBE = 0.04

# Visual-only geoms (the setpoint marker): hidden from the depth camera.
MARKER_GROUP = 2


def _f(*v: float) -> str:
  return ' '.join(f'{x:.4g}' for x in v)


def _obstacles_xml(obstacles: list[world.Obstacle]) -> str:
  out = []
  for ob in obstacles:
    if ob.kind == world.PILLAR:
      out.append(
          f'<geom class="obstacle" type="cylinder" material="pillar" '
          f'pos="{_f(ob.x, ob.y, ob.b / 2)}" size="{_f(ob.a, ob.b / 2)}"/>'
      )
    else:
      pts = world.ring_points(ob, RING_SEGMENTS)
      for p, q in zip(pts[:-1], pts[1:]):
        out.append(
            f'<geom class="obstacle" type="capsule" material="ring" '
            f'fromto="{_f(*p, *q)}" size="{RING_TUBE}"/>'
        )
  return '\n    '.join(out)


def _quadrotor_xml(p: quadrotor.Params, start: tuple[float, float, float]) -> str:
  rotors = []
  for i, (x, y, _) in enumerate(p.rotor_layout()):
    rotors.append(
        f'<site name="rotor{i}" pos="{_f(x, y, 0.02)}"/>\n'
        f'      <geom class="drone" type="cylinder" material="prop" '
        f'pos="{_f(x, y, 0.025)}" size="{_f(p.prop_radius, 0.004)}"/>\n'
        f'      <geom class="drone" type="capsule" material="frame" '
        f'fromto="0 0 0 {_f(x, y, 0)}" size="0.008"/>'
    )
  return f'''
    <body name="quad" pos="{_f(*start)}">
      <freejoint name="quad"/>
      <inertial pos="0 0 0" mass="{p.mass}" diaginertia="{_f(*p.inertia)}"/>
      <geom class="drone" type="box" material="frame" size="0.06 0.045 0.02"/>
      {chr(10).join('      ' + r for r in rotors).strip()}
      <camera name="depth" pos="{CAM_OFFSET} 0 0" xyaxes="0 -1 0 0 0 1"
              fovy="{CAM_FOVY:.4f}" resolution="{CAM_WIDTH} {CAM_HEIGHT}"/>
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


def build_xml(
    obstacles: list[world.Obstacle],
    params: quadrotor.Params,
    start: tuple[float, float, float],
    timestep: float = 0.002,
) -> str:
  return f'''<mujoco model="star_nav">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <statistic extent="6" center="{_f(start[0], start[1], 0.5)}"/>
  <visual>
    <global offwidth="{max(CAM_WIDTH, 1280)}" offheight="{max(CAM_HEIGHT, 720)}"
            azimuth="-135" elevation="-30"/>
    <map znear="0.008" zfar="15"/>
    <quality shadowsize="4096"/>
    <headlight ambient="0.35 0.35 0.35" diffuse="0.5 0.5 0.5"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.55 0.7 0.9"
             rgb2="0.95 0.97 1" width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.78 0.8 0.78"
             rgb2="0.7 0.72 0.7" width="512" height="512"/>
    <material name="ground" texture="grid" texrepeat="25 25"/>
    <material name="pillar" rgba="0.35 0.55 0.3 1"/>
    <material name="ring" rgba="0.85 0.55 0.2 1"/>
    <material name="frame" rgba="0.15 0.15 0.18 1"/>
    <material name="prop" rgba="0.9 0.4 0.1 0.8"/>
    <material name="target" rgba="0.1 0.6 1 0.6"/>
  </asset>
  <default>
    <default class="obstacle">
      <geom contype="1" conaffinity="1"/>
    </default>
    <default class="drone">
      <geom contype="2" conaffinity="1" mass="0"/>
    </default>
  </default>
  <worldbody>
    <light pos="4 -4 12" dir="-0.3 0.3 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="ground" type="plane" size="{_f(30, 30, 0.1)}" material="ground"
          contype="1" conaffinity="2"/>
    {_obstacles_xml(obstacles)}
    {_quadrotor_xml(params, start)}
    <body name="setpoint" mocap="true" pos="{_f(*start)}">
      <geom type="sphere" size="0.06" material="target" contype="0" conaffinity="0"
            group="{MARKER_GROUP}"/>
    </body>
  </worldbody>
  <actuator>
    {_actuators_xml(params)}
  </actuator>
</mujoco>
'''
