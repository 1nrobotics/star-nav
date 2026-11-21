"""Tests for Beni, the two-wheeled balancing robot.

The weight-extraction and tracker tests always run. The driving and sensor
tests need the downloaded model and policy (``star-nav-mujoco --fetch-beni``);
they are skipped without them.
"""

import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import beni
from star_nav_mujoco import lidar
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import sim
from star_nav_mujoco import world


# -- no assets needed -----------------------------------------------------------


def _varint(n: int) -> bytes:
  out = b''
  while True:
    out += bytes([(n & 0x7F) | (0x80 if n > 0x7F else 0)])
    n >>= 7
    if not n:
      return out


def _field(number: int, payload: bytes) -> bytes:
  return _varint(number << 3 | 2) + _varint(len(payload)) + payload


def _tensor(name: str, array: np.ndarray, packed: bool) -> bytes:
  if packed:
    dims = _field(1, b''.join(_varint(d) for d in array.shape))
  else:
    dims = b''.join(_varint(1 << 3) + _varint(d) for d in array.shape)
  return (dims + _varint(2 << 3) + _varint(1) + _field(8, name.encode())
          + _field(9, array.astype('<f4').tobytes()))


def _onnx(tensors: dict, packed: bool = True) -> bytes:
  graph = _field(1, b'a node') + b''.join(
      _field(5, _tensor(k, v, packed)) for k, v in tensors.items())
  return _varint(1 << 3) + _varint(8) + _field(7, graph)  # ir_version, graph


def _random_weights():
  rng = np.random.default_rng(0)
  return {name: rng.standard_normal(shape).astype('<f4')
          for name, (_, shape) in beni.WEIGHTS.items()}


@pytest.mark.parametrize('packed', [True, False])
def test_extract_weights_reads_the_initializers(tmp_path, packed):
  ref = _random_weights()
  extra = {'unused': np.zeros((2, 3), '<f4')}
  path = tmp_path / 'policy.onnx'
  path.write_bytes(_onnx({**extra, **ref}, packed))
  got = beni.extract_weights(path)
  assert got.keys() == {key for key, _ in beni.WEIGHTS.values()}
  for name, (key, _) in beni.WEIGHTS.items():
    np.testing.assert_array_equal(got[key], ref[name])


def test_extract_weights_rejects_a_different_network(tmp_path):
  ref = _random_weights()
  path = tmp_path / 'policy.onnx'
  path.write_bytes(_onnx({**ref, 'mlp.6.weight': np.zeros((8, 128), '<f4')}))
  with pytest.raises(ValueError):
    beni.extract_weights(path)
  del ref['mlp.6.weight']
  path.write_bytes(_onnx(ref))
  with pytest.raises(ValueError):
    beni.extract_weights(path)


def test_tracker_turns_before_it_drives_and_never_sideways():
  t = beni.Tracker()
  pos = np.zeros(3)
  ahead = t.twist((1.0, 0.0), (0.0, 0.0), 0.0, pos, 0.0)
  assert ahead[0] > 0.5 and ahead[1] == 0.0 and abs(ahead[2]) < 1e-6
  left = t.twist((0.0, 1.0), (0.0, 0.0), 0.0, pos, 0.0)
  assert left[0] == 0.0 and left[1] == 0.0 and left[2] > 1.0
  behind = t.twist((-1.0, 0.0), (0.0, 0.0), 0.0, pos, 0.1)
  assert behind[0] == 0.0 and behind[2] > 1.0


def test_tracker_holds_a_resting_setpoint_and_turns_to_its_yaw():
  t = beni.Tracker()
  pos = np.zeros(3)
  near = t.twist((0.03, 0.0), (0.0, 0.0), 1.0, pos, 0.0)
  assert near[0] == 0.0 and near[2] > 1.0
  # Still holding a little further out (hysteresis), not beyond ``release``.
  assert t.twist((0.1, 0.0), (0.0, 0.0), 0.0, pos, 0.0)[0] == 0.0
  assert t.twist((0.2, 0.0), (0.0, 0.0), 0.0, pos, 0.0)[0] > 0.0
  # A moving setpoint is followed even when it is close.
  assert t.twist((0.03, 0.0), (0.5, 0.0), 0.0, pos, 0.0)[0] > 0.4


# -- simulation (needs the downloaded robot) ------------------------------------


def _assets_available() -> bool:
  try:
    beni.check_assets()
  except beni.MissingAssets:
    return False
  return True


needs_beni = pytest.mark.skipif(
    not _assets_available(),
    reason='Beni not downloaded (star-nav-mujoco --fetch-beni)')


def _robot(obstacles, **kw):
  cfg = sim.SimConfig(world=world.WorldConfig(size_x=10, size_y=10),
                      start=(0.0, 0.0, beni.STAND_HEIGHT), robot=sim.BENI, **kw)
  return sim.Simulator(cfg, obstacles=obstacles, render_depth=False)


def _go(s, x, y, yaw, seconds):
  cmd = quadrotor.Command(np.array([x, y, beni.STAND_HEIGHT]), np.zeros(3),
                          np.zeros(3), yaw, 0.0)
  for _ in range(int(seconds / 0.1)):
    s.set_command(cmd)  # keep it fresh (cmd_timeout)
    s.step(int(0.1 / beni.TIMESTEP))


def _up(s) -> float:
  """z component of the base's up axis: 1 upright, 0 lying down."""
  return s.data.xmat[s.model.body(beni.TRUNK).id][8]


@needs_beni
def test_extracted_weights_match_the_download():
  root = beni.check_assets()
  got = beni.extract_weights(root / beni.POLICY_FILE)
  with np.load(root / beni.WEIGHTS_FILE) as saved:
    for key in got:
      np.testing.assert_array_equal(got[key], saved[key])


@needs_beni
def test_balances_on_a_zero_command():
  s = _robot([])
  s.step(int(5.0 / beni.TIMESTEP))
  assert s.pos[2] == pytest.approx(beni.STAND_HEIGHT, abs=0.02)
  assert _up(s) > 0.95
  assert np.hypot(*s.pos[:2]) < 0.3  # it creeps a little without a setpoint
  assert not s._fallen and s.contact_steps == 0
  s.close()


@needs_beni
def test_drives_to_a_setpoint_and_turns_to_its_yaw():
  s = _robot([])
  _go(s, 2.0, 1.0, math.pi / 2, 8.0)
  yaw = 2.0 * math.atan2(s.data.qpos[6], s.data.qpos[3])
  assert np.hypot(s.pos[0] - 2.0, s.pos[1] - 1.0) < 0.15
  assert abs(math.remainder(yaw - math.pi / 2, 2 * math.pi)) < math.radians(10)
  assert not s._fallen and s.contact_steps == 0
  s.close()


@needs_beni
@pytest.mark.parametrize('pitch', [1.4, -1.4])
def test_gets_back_up_after_a_fall(pitch):
  s = _robot([])
  s.step(int(1.0 / beni.TIMESTEP))
  s.data.qvel[:] = 0.0
  s.data.qpos[2] = 0.08
  s.data.qpos[3:7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]
  _go(s, 0.0, 0.0, 0.0, 3.0)
  assert _up(s) > 0.95
  assert s.pos[2] == pytest.approx(beni.STAND_HEIGHT, abs=0.02)
  s.close()


@needs_beni
def test_obstacle_contact_is_counted_but_not_the_floor():
  pillar = world.Obstacle(world.PILLAR, 1.0, 0.0, 0.0, 0.15, 1.0, 0.0)
  s = _robot([pillar])
  _go(s, 2.0, 0.0, 0.0, 5.0)
  assert s.contact_steps > 0
  s.close()


@needs_beni
def test_sensors_do_not_see_the_robot():
  s = sim.Simulator(sim.SimConfig(
      world=world.WorldConfig(size_x=10, size_y=10),
      start=(0.0, 0.0, beni.STAND_HEIGHT), robot=sim.BENI,
      lidar=lidar.LidarConfig(vfov=(-30.0, 52.0))), obstacles=[])
  s.step(int(1.0 / beni.TIMESTEP))
  scan = s.scan_lidar()
  # Only the floor: the nearest return is the floor at -30 deg, not the head.
  assert scan.pos[2] == pytest.approx(
      beni.STAND_HEIGHT + beni.LIDAR_MOUNT[2], abs=0.02)
  nearest = np.linalg.norm(scan.points, axis=1).min()
  assert nearest > 0.9 * scan.pos[2] / math.sin(math.radians(30.0))
  depth = s.render_depth()
  near = depth.pos[2] / math.tan(math.radians(scene.CAM_FOVY / 2))
  assert depth.image_mm[depth.image_mm > 0].min() / 1000.0 > 0.9 * near
  s.close()


@needs_beni
def test_drives_up_the_house_ramp():
  # Row x = 1.5 runs up a 12 degree ramp to the 0.21 m platform. (The 7 cm
  # steps beyond it are higher than the wheels' axles: not for Beni.)
  cfg = sim.SimConfig(start=(1.5, -4.0, beni.STAND_HEIGHT), robot=sim.BENI,
                      house=True)
  s = sim.Simulator(cfg, render_depth=False)
  s.data.qpos[3:7] = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]
  s.step(int(1.0 / beni.TIMESTEP))
  y = -4.0
  while y < 0.2:
    y += 0.6 * 0.02
    s.set_command(quadrotor.Command(
        np.array([1.5, y, beni.STAND_HEIGHT]), np.array([0.0, 0.6, 0.0]),
        np.zeros(3), math.pi / 2, 0.0))
    s.step(int(0.02 / beni.TIMESTEP))
  _go(s, 1.5, y, math.pi / 2, 2.0)
  assert s.pos[2] > beni.STAND_HEIGHT + 0.21 - 0.03  # on the platform
  assert abs(s.pos[1] - y) < 0.15
  assert not s._fallen and s.contact_steps == 0
  s.close()
