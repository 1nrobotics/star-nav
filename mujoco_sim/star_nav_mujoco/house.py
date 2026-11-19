"""A house for walking robots: furnished rooms, a ramp, a platform and stairs.

11 x 11 m, laid out around the default 9 m lawnmower mission (rows at
x = 4.5, 1.5, -1.5, -4.5, waypoints 1 m from the outer walls):

* West room (x < -0.6): a dining table on legs (top at 0.75 m, the robot
  walks under it on row x = -1.5), chairs, a sofa, a bookshelf and a 15 cm
  box on the floor (too high to step over).
* East (x > -0.6): row x = 1.5 runs through a corridor with a 12 degree ramp
  up to a 0.21 m platform and three 7 cm steps (0.3 m treads) down. The
  Lite3's blind velocity policy climbs 7 cm steps reliably, 8 cm sometimes and
  falls on 10 cm ones (measured), so the steps are 7 cm. Going around the
  corridor is ~3 m longer.
* The wall between them is open at y > 3.4.

Geoms are boxes and cylinders (class ``obstacle``). The walkable ones (ramp,
platform, steps) are named ``walk/...``: a lower leg or foot on them is
walking, not a collision.
"""

from __future__ import annotations

import math

from star_nav_mujoco import scene

WALL_H = 1.0
WALL_T = 0.05  # half thickness
LEG = 0.025  # table/chair leg half width

# Materials (defined in scene.py, textured by textures.py).
WALL = 'wall'
WOOD = 'wood'
FABRIC = 'fabric'
STONE = 'stone'
RED = 'red'
GREEN = 'green'

WALKABLE = 'walk/'

# The ramp: up along +y from (y0, 0) to (y1, rise), in the corridor.
CORRIDOR_X = (0.6, 2.4)  # wall centres
RAMP_Y = (-2.2, -1.2)
STEP = 0.07  # riser
RISE = 3 * STEP  # platform height
PLATFORM_Y = (-1.2, 0.6)
TREAD = 0.3


def _box(name, centre, half, material, euler=None):
  return ('box', name, tuple(centre), tuple(half), material, euler)


def _wall_x(x, y0, y1, h=WALL_H):
  return _box('', (x, (y0 + y1) / 2, h / 2), (WALL_T, (y1 - y0) / 2, h / 2), WALL)


def _wall_y(y, x0, x1, h=WALL_H):
  return _box('', ((x0 + x1) / 2, y, h / 2), ((x1 - x0) / 2, WALL_T, h / 2), WALL)


def _table(x, y, hx, hy, h):
  out = [_box('', (x, y, h - 0.015), (hx, hy, 0.015), WOOD)]
  for sx in (-1, 1):
    for sy in (-1, 1):
      out.append(_box('', (x + sx * (hx - LEG), y + sy * (hy - LEG), (h - 0.03) / 2),
                      (LEG, LEG, (h - 0.03) / 2), WOOD))
  return out


def geoms(wall_height=WALL_H):
  """(type, name, centre, half sizes, material, euler) of every geom."""
  g = []
  # Outer walls and the wall between west and east (open at y > 3.4).
  h = wall_height
  g += [_wall_y(-5.5, -5.5, 5.5, h), _wall_y(5.5, -5.5, 5.5, h),
        _wall_x(-5.5, -5.5, 5.5, h), _wall_x(5.5, -5.5, 5.5, h),
        _wall_x(-0.6, -5.5, 3.4, h)]
  # East: the terrain corridor on row x = 1.5.
  cx = sum(CORRIDOR_X) / 2
  half_w = (CORRIDOR_X[1] - CORRIDOR_X[0]) / 2 - WALL_T
  steps_end = PLATFORM_Y[1] + 2 * TREAD
  g += [_wall_x(CORRIDOR_X[0], RAMP_Y[0] - 1.0, steps_end + 0.6, h),
        _wall_x(CORRIDOR_X[1], RAMP_Y[0] - 1.0, steps_end + 0.6, h)]
  run = RAMP_Y[1] - RAMP_Y[0]
  pitch = math.atan2(RISE, run)
  length = math.hypot(run, RISE)
  t = 0.05  # half thickness
  mid_y, mid_z = (RAMP_Y[0] + RAMP_Y[1]) / 2, RISE / 2
  g.append(_box(WALKABLE + 'ramp',
                (cx, mid_y + t * math.sin(pitch), mid_z - t * math.cos(pitch)),
                (half_w, length / 2, t), STONE, (pitch, 0.0, 0.0)))
  g.append(_box(WALKABLE + 'platform', (cx, sum(PLATFORM_Y) / 2, RISE / 2),
                (half_w, (PLATFORM_Y[1] - PLATFORM_Y[0]) / 2, RISE / 2), STONE))
  for i, top in enumerate((RISE - STEP, RISE - 2 * STEP)):
    y0 = PLATFORM_Y[1] + i * TREAD
    g.append(_box(f'{WALKABLE}step{i}', (cx, y0 + TREAD / 2, top / 2),
                  (half_w, TREAD / 2, top / 2), STONE))
  g += [_box('', (3.5, -2.0, 0.4), (0.2, 0.5, 0.4), WOOD),  # cabinet
        ('cylinder', '', (3.6, 2.5, 0.3), (0.2, 0.3), GREEN, None)]  # plant
  # West room.
  g += _table(-1.5, -2.0, 0.7, 0.5, 0.75)  # the robot walks under it
  g += _table(-2.6, -2.0, 0.2, 0.2, 0.45)  # chair
  g += _table(-2.6, -0.9, 0.2, 0.2, 0.45)  # chair
  g += [_box('', (-3.5, 1.5, 0.225), (0.4, 0.9, 0.225), FABRIC),  # sofa
        _box('', (-5.2, -1.0, 0.9), (0.15, 0.6, 0.9), WOOD),  # bookshelf
        _box('', (-3.0, -3.5, 0.075), (0.15, 0.15, 0.075), RED)]  # box
  return g


def xml(wall_height=WALL_H) -> str:
  """The house's geoms; ``wall_height`` for all walls."""
  # A wooden floor inside the walls, for show: sensors ignore its group.
  out = ['<geom type="box" pos="0 0 0.001" size="5.5 5.5 0.001" material="floor" '
         f'contype="0" conaffinity="0" group="{scene.MARKER_GROUP}"/>']
  for kind, name, centre, half, material, euler in geoms(wall_height):
    attrs = f'class="obstacle" type="{kind}"'
    if name:
      attrs += f' name="{name}"'
    attrs += ' pos="{}" size="{}" material="{}"'.format(
        ' '.join(f'{v:.4g}' for v in centre), ' '.join(f'{v:.4g}' for v in half),
        material)
    if euler is not None:
      # The scene uses MuJoCo's default angle unit, degrees.
      attrs += ' euler="{}"'.format(' '.join(f'{math.degrees(v):.6g}' for v in euler))
    out.append(f'<geom {attrs}/>')
  return '\n    '.join(out)
