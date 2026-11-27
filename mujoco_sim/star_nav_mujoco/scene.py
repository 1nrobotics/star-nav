"""Builds the MuJoCo scene (MJCF): the ground, the obstacles and the lights.

The robot is not in here. A robot written in MJCF gives its ``Parts`` to
``build_xml`` (the quadrotor); one with a model file of its own is attached
to the scene with MjSpec (the walking robots). See ``robots/``.
"""

from __future__ import annotations

import dataclasses
import math

from star_nav_mujoco import overlay
from star_nav_mujoco import world

# Depth camera, matching the intrinsics the planner is configured with.
CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_FOCAL = 387.229248046875
CAM_FOVY = math.degrees(2.0 * math.atan(CAM_HEIGHT / 2.0 / CAM_FOCAL))
CAM_OFFSET = 0.14  # metres ahead of the body centre

# Geom group of visual-only markers; sensors ignore it.
MARKER_GROUP = 2
# Geom group of an attached robot's own geoms (its legs); sensors ignore it.
ROBOT_GROUP = 3
RING_SEGMENTS = 24
RING_TUBE = 0.04


@dataclasses.dataclass(frozen=True)
class Parts:
  """A robot written in MJCF: what goes into each section of the scene."""

  body: str = ''  # <worldbody>
  actuators: str = ''  # <actuator>
  assets: str = ''  # <asset>
  defaults: str = ''  # <default>


def fmt(*v: float) -> str:
  """Numbers as an MJCF attribute value."""
  return ' '.join(f'{x:.4g}' for x in v)


def _obstacles_xml(obstacles: list[world.Obstacle]) -> str:
  out = []
  for ob in obstacles:
    if ob.kind == world.PILLAR:
      out.append(
          f'<geom class="obstacle" type="cylinder" material="pillar" '
          f'pos="{fmt(ob.x, ob.y, ob.b / 2)}" size="{fmt(ob.a, ob.b / 2)}"/>'
      )
    else:
      pts = world.ring_points(ob, RING_SEGMENTS)
      for p, q in zip(pts[:-1], pts[1:]):
        out.append(
            f'<geom class="obstacle" type="capsule" material="ring" '
            f'fromto="{fmt(*p, *q)}" size="{RING_TUBE}"/>'
        )
  return '\n    '.join(out)


def build_xml(
    obstacles: list[world.Obstacle],
    start: tuple[float, float, float],
    timestep: float = 0.002,
    extra: str = '',
    draw: bool = False,
    robot: Parts = Parts(),
) -> str:
  """Returns the scene; without ``robot`` the robot is attached later.

  ``start`` is where the robot stands: the view is centred on it and the
  setpoint marker starts there. ``extra`` is more worldbody XML (e.g.
  ``house.xml()``). ``draw`` adds the markers the planner's paths are drawn
  with (overlay.py).
  """
  markers = overlay.xml(MARKER_GROUP) if draw else ''
  return f'''<mujoco model="star_nav">
  <option timestep="{timestep}" integrator="implicitfast"/>
  <statistic extent="6" center="{fmt(start[0], start[1], 0.5)}"/>
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
    {robot.assets}
    <!-- The house (house.py) and the ground; textures.py textures them -->
    <material name="floor" rgba="0.78 0.6 0.42 1"/>
    <material name="grass" rgba="0.36 0.52 0.26 1"/>
    <material name="wall" rgba="0.88 0.86 0.82 1"/>
    <material name="wood" rgba="0.62 0.45 0.3 1"/>
    <material name="fabric" rgba="0.35 0.45 0.6 1"/>
    <material name="stone" rgba="0.6 0.6 0.62 1"/>
    <material name="red" rgba="0.75 0.3 0.25 1"/>
    <material name="green" rgba="0.3 0.55 0.3 1"/>
  </asset>
  <default>
    <default class="obstacle">
      <geom contype="1" conaffinity="1"/>
    </default>
    {robot.defaults}
  </default>
  <worldbody>
    <light pos="4 -4 12" dir="-0.3 0.3 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="ground" type="plane" size="{fmt(30, 30, 0.1)}" material="ground"
          contype="1" conaffinity="2"/>
    {_obstacles_xml(obstacles)}
    {extra}
    {robot.body}
    <!-- The planner's position setpoint: not drawn -->
    <body name="setpoint" mocap="true" pos="{fmt(*start)}"/>
    {markers}
  </worldbody>
  <actuator>
    {robot.actuators}
  </actuator>
</mujoco>
'''
