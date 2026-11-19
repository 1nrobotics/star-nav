"""Tests for the T1 humanoid and the scene textures.

The weight-extraction and texture tests always run. The walking and sensor
tests need the downloaded model and policy (``star-nav-mujoco --fetch-t1``);
they are skipped without them.
"""

import math
import os
import zipfile

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np
import pytest

from star_nav_mujoco import house
from star_nav_mujoco import lidar
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import sim
from star_nav_mujoco import t1
from star_nav_mujoco import textures
from star_nav_mujoco import world


# -- no assets needed -----------------------------------------------------------


def test_extract_weights_reads_the_tensor_files_in_order(tmp_path):
  rng = np.random.default_rng(0)
  ref = {}
  path = tmp_path / 'T1.pt'
  with zipfile.ZipFile(path, 'w') as z:
    z.writestr('T1/byteorder', 'little')
    z.writestr('T1/data.pkl', b'not read')
    for i, (rows, cols) in enumerate(t1.LAYERS):
      ref[f'w{i}'] = rng.standard_normal((rows, cols)).astype('<f4')
      ref[f'b{i}'] = rng.standard_normal(rows).astype('<f4')
      z.writestr(f'T1/data/{2 * i}', ref[f'w{i}'].tobytes())
      z.writestr(f'T1/data/{2 * i + 1}', ref[f'b{i}'].tobytes())
  got = t1.extract_weights(path)
  assert got.keys() == ref.keys()
  for k, v in ref.items():
    np.testing.assert_array_equal(got[k], v)


def test_extract_weights_rejects_a_different_network(tmp_path):
  path = tmp_path / 'T1.pt'
  with zipfile.ZipFile(path, 'w') as z:
    z.writestr('T1/byteorder', 'little')
    z.writestr('T1/data/0', np.zeros((128, 47), '<f4').tobytes())
  with pytest.raises(ValueError):
    t1.extract_weights(path)


def test_textures_cover_the_scene_materials():
  spec = mujoco.MjSpec.from_string(scene.build_xml(
      [], quadrotor.Params(), (0.0, 0.0, 1.0), extra=house.xml()))
  textures.apply(spec)
  m = spec.compile()
  role = mujoco.mjtTextureRole.mjTEXROLE_RGB
  for name in textures.TEXTURES:
    assert m.mat_texid[m.material(name).id, role] >= 0, name
  assert m.geom('ground').matid == m.material('grass').id
  # Tileable and not flat.
  for make, _ in textures.TEXTURES.values():
    img = make(np.random.default_rng(0))
    assert img.shape == (textures.SIZE, textures.SIZE, 3)
    assert img.dtype == np.uint8 and img.std() > 2.0


def test_house_floor_is_for_show_only():
  spec = mujoco.MjSpec.from_string(scene.build_xml(
      [], quadrotor.Params(), (0.0, 0.0, 1.0), extra=house.xml()))
  m = spec.compile()
  floor = [g for g in range(m.ngeom)
           if m.geom_matid[g] == m.material('floor').id]
  assert len(floor) == 1
  g = floor[0]
  assert m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0
  assert m.geom_group[g] == scene.MARKER_GROUP  # sensors skip it


# -- simulation (needs the downloaded robot) ------------------------------------


def _assets_available() -> bool:
  try:
    t1.check_assets()
  except t1.MissingAssets:
    return False
  return True


needs_t1 = pytest.mark.skipif(
    not _assets_available(),
    reason='T1 not downloaded (star-nav-mujoco --fetch-t1)')


def _robot(obstacles, **kw):
  cfg = sim.SimConfig(world=world.WorldConfig(size_x=10, size_y=10),
                      start=(0.0, 0.0, t1.STAND_HEIGHT), robot=sim.T1, **kw)
  return sim.Simulator(cfg, obstacles=obstacles, render_depth=False)


def _go(s, x, y, yaw, seconds):
  cmd = quadrotor.Command(np.array([x, y, t1.STAND_HEIGHT]), np.zeros(3),
                          np.zeros(3), yaw, 0.0)
  for _ in range(int(seconds / 0.1)):
    s.set_command(cmd)  # keep it fresh (cmd_timeout)
    s.step(int(0.1 / t1.TIMESTEP))


@needs_t1
def test_stands_on_a_zero_command():
  s = _robot([])
  s.step(int(3.0 / t1.TIMESTEP))
  assert s.pos[2] == pytest.approx(t1.STAND_HEIGHT, abs=0.03)
  assert np.hypot(*s.pos[:2]) < 0.1
  assert s.contact_steps == 0
  s.close()


@needs_t1
def test_walks_to_a_setpoint_and_turns_to_its_yaw():
  s = _robot([])
  _go(s, 2.0, 1.0, math.pi / 2, 14.0)
  yaw = 2.0 * math.atan2(s.data.qpos[6], s.data.qpos[3])
  assert np.hypot(s.pos[0] - 2.0, s.pos[1] - 1.0) < 0.15
  assert abs(math.remainder(yaw - math.pi / 2, 2 * math.pi)) < math.radians(10)
  assert not s._fallen and s.contact_steps == 0
  s.close()


@needs_t1
def test_obstacle_contact_is_counted_but_not_the_floor():
  pillar = world.Obstacle(world.PILLAR, 1.0, 0.0, 0.0, 0.15, 1.0, 0.0)
  s = _robot([pillar])
  _go(s, 2.0, 0.0, 0.0, 8.0)
  assert s.contact_steps > 0
  s.close()


@needs_t1
def test_sensors_do_not_see_the_robot():
  s = sim.Simulator(sim.SimConfig(
      world=world.WorldConfig(size_x=10, size_y=10),
      start=(0.0, 0.0, t1.STAND_HEIGHT), robot=sim.T1,
      lidar=lidar.LidarConfig(vfov=(-50.0, 30.0))), obstacles=[])
  s.step(int(1.0 / t1.TIMESTEP))
  scan = s.scan_lidar()
  # Only the floor: the nearest return is the floor at -50 deg, not an arm.
  nearest = np.linalg.norm(scan.points, axis=1).min()
  assert nearest > 0.9 * scan.pos[2] / math.sin(math.radians(50.0))
  # The depth camera (level, in the face) sees the floor from where the
  # bottom of its view meets it, not the robot's own chest or feet.
  depth = s.render_depth()
  near = depth.pos[2] / math.tan(math.radians(scene.CAM_FOVY / 2))
  assert depth.image_mm[depth.image_mm > 0].min() / 1000.0 > 0.95 * near
  s.close()


def _house(start, yaw):
  cfg = sim.SimConfig(start=(start[0], start[1], t1.STAND_HEIGHT),
                      robot=sim.T1, house=True)
  s = sim.Simulator(cfg, render_depth=False)
  s.data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
  s.step(int(1.0 / t1.TIMESTEP))
  return s


def _walk_y(s, y0, y1, speed, yaw):
  """Moves the setpoint along x = 1.5 (the terrain corridor) at ``speed``."""
  d = 1.0 if y1 > y0 else -1.0
  y, peak = y0, 0.0
  for _ in range(int(abs(y1 - y0) / speed / 0.02) + 300):
    y = min(y + d * speed * 0.02, y1) if d > 0 else max(y + d * speed * 0.02, y1)
    vel = np.array([0.0, d * speed if y != y1 else 0.0, 0.0])
    s.set_command(quadrotor.Command(np.array([1.5, y, t1.STAND_HEIGHT]), vel,
                                    np.zeros(3), yaw, 0.0))
    s.step(int(0.02 / t1.TIMESTEP))
    peak = max(peak, s.pos[2])
  return peak


@needs_t1
def test_walks_up_the_house_ramp_and_down_the_stairs():
  # The mission's way through the corridor. Climbing the 7 cm steps up is
  # not reliable: the policy was trained on terrain of at most ~2 cm steps.
  s = _house((1.5, -4.0), math.pi / 2)
  peak = _walk_y(s, -4.0, 3.0, 0.4, math.pi / 2)
  assert peak > t1.STAND_HEIGHT + house.RISE - 0.04  # on the platform
  assert abs(s.pos[1] - 3.0) < 0.25
  assert not s._fallen and s.contact_steps == 0
  s.close()
