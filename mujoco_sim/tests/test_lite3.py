"""Tests for the Lite3 quadruped.

The tracker tests always run. The walking and sensor tests need the
downloaded model and policy (``star-nav-mujoco --fetch lite3``) and
onnxruntime; they are skipped without them.
"""

import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import lidar
from star_nav_mujoco import quadrotor
from star_nav_mujoco import sim
from star_nav_mujoco import tracking
from star_nav_mujoco import world
from star_nav_mujoco.robots import lite3


# -- tracker (no assets needed) -------------------------------------------------


def test_tracker_walks_sideways_in_the_body_frame():
  t = tracking.Holonomic()
  # Facing +y; the setpoint is 0.2 m along +x: that is to the right.
  fwd, side, turn = t.twist([0.2, 0.0, 0.3], np.zeros(3), math.pi / 2,
                            np.zeros(3), math.pi / 2)
  assert abs(fwd) < 1e-6 and side == pytest.approx(-t.k_pos * 0.2)
  assert turn == 0.0


def test_tracker_follows_setpoint_velocity_within_the_limit():
  t = tracking.Holonomic()
  fwd, side, _ = t.twist(np.zeros(3), [2.0, 0.0, 0.0], 0.0, np.zeros(3), 0.0)
  assert fwd == pytest.approx(t.max_speed) and side == 0.0


def test_tracker_turns_towards_the_planner_yaw():
  t = tracking.Holonomic()
  _, _, turn = t.twist(np.zeros(3), np.zeros(3), 1.0, np.zeros(3), 0.0)
  assert turn == pytest.approx(t.max_turn)
  _, _, turn = t.twist(np.zeros(3), np.zeros(3), -0.2, np.zeros(3), 0.0)
  assert turn == pytest.approx(-t.k_yaw * 0.2)


def test_tracker_stands_at_a_resting_setpoint():
  t = tracking.Holonomic()
  twist = t.twist([0.01, 0.0, 0.3], np.zeros(3), 0.01, np.zeros(3), 0.0)
  np.testing.assert_array_equal(twist, 0.0)


def test_velocity_command_is_kept_within_the_limits():
  t = tracking.Holonomic(max_speed=0.8, max_turn=0.8)
  np.testing.assert_allclose(t.limit([0.3, -0.1, 0.2]), [0.3, -0.1, 0.2])
  fwd, side, turn = t.limit([3.0, 4.0, -2.0])
  assert math.hypot(fwd, side) == pytest.approx(0.8)
  assert side / fwd == pytest.approx(4.0 / 3.0)
  assert turn == pytest.approx(-0.8)


# -- simulation (needs the downloaded robot) ------------------------------------


def _assets_available() -> bool:
  try:
    lite3.check_assets()
  except lite3.MissingAssets:
    return False
  return True


needs_lite3 = pytest.mark.skipif(
    not _assets_available(),
    reason='Lite3 not downloaded (star-nav-mujoco --fetch lite3)')


def _robot(obstacles, **kw):
  cfg = sim.SimConfig(world=world.WorldConfig(size_x=10, size_y=10),
                      start=(0.0, 0.0, lite3.STAND_HEIGHT), robot='lite3', **kw)
  return sim.Simulator(cfg, obstacles=obstacles, render_depth=False)


def _go(s, x, y, yaw, seconds):
  cmd = quadrotor.Command(np.array([x, y, lite3.STAND_HEIGHT]), np.zeros(3),
                          np.zeros(3), yaw, 0.0)
  for _ in range(int(seconds / 0.1)):
    s.set_command(cmd)  # keep it fresh (cmd_timeout)
    s.step(int(0.1 / lite3.TIMESTEP))


@needs_lite3
def test_stands_on_a_zero_command():
  s = _robot([])
  s.step(int(2.0 / lite3.TIMESTEP))
  assert s.pos[2] == pytest.approx(lite3.STAND_HEIGHT, abs=0.03)
  assert np.hypot(*s.pos[:2]) < 0.05
  assert s.contact_steps == 0
  s.close()


@needs_lite3
def test_walks_to_a_setpoint_and_turns_to_its_yaw():
  s = _robot([])
  _go(s, 2.0, 1.0, math.pi / 2, 10.0)
  yaw = 2.0 * math.atan2(s.data.qpos[6], s.data.qpos[3])
  assert np.hypot(s.pos[0] - 2.0, s.pos[1] - 1.0) < 0.1
  assert abs(math.remainder(yaw - math.pi / 2, 2 * math.pi)) < math.radians(8)
  assert not s.fallen and s.contact_steps == 0
  s.close()


@needs_lite3
def test_follows_a_velocity_command_and_stands_when_it_stops():
  s = _robot([])
  assert not s.set_velocity([float('nan'), 0.0, 0.0])
  for _ in range(40):
    assert s.set_velocity([0.4, 0.0, 0.3])  # keep it fresh (cmd_timeout)
    s.step(int(0.1 / lite3.TIMESTEP))
  yaw = 2.0 * math.atan2(s.data.qpos[6], s.data.qpos[3])
  assert 0.9 < yaw < 1.6  # 0.3 rad/s for 4 s (measured: 1.43)
  assert 1.0 < np.hypot(*s.pos[:2]) < 1.7  # 0.4 m/s along an arc
  # No more commands: it stands where it is, not back at the setpoint.
  s.step(int(1.5 / lite3.TIMESTEP))
  rest = s.pos
  s.step(int(2.0 / lite3.TIMESTEP))
  assert np.hypot(*(s.pos - rest)[:2]) < 0.05
  assert not s.fallen and s.contact_steps == 0
  s.close()


@needs_lite3
def test_obstacle_contact_is_counted_but_not_the_floor():
  pillar = world.Obstacle(world.PILLAR, 1.0, 0.0, 0.0, 0.15, 1.0, 0.0)
  s = _robot([pillar])
  _go(s, 2.0, 0.0, 0.0, 6.0)
  assert s.contact_steps > 0
  s.close()


@needs_lite3
def test_lidar_does_not_see_the_robot():
  s = _robot([], lidar=lidar.LidarConfig())
  s.step(int(1.0 / lite3.TIMESTEP))
  scan = s.scan_lidar()
  # Only the floor around it: the nearest return is the floor at -22.5 deg,
  # not a leg a few centimetres away.
  height = scan.pos[2]
  nearest = np.linalg.norm(scan.points, axis=1).min()
  assert nearest > 0.9 * height / math.sin(math.radians(22.5))
  assert np.allclose(scan.pos[:2], s.pos[:2] + 0.0, atol=0.05)
  s.close()


def _house(start, yaw):
  cfg = sim.SimConfig(start=(start[0], start[1], lite3.STAND_HEIGHT),
                      robot='lite3', house=True)
  s = sim.Simulator(cfg, render_depth=False)
  s.data.qpos[3:7] = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
  s.step(int(1.0 / lite3.TIMESTEP))
  return s


def _walk_y(s, y0, y1, speed, yaw):
  """Moves the setpoint along x = 1.5 (the terrain corridor) at ``speed``."""
  d = 1.0 if y1 > y0 else -1.0
  y, peak = y0, 0.0
  for _ in range(int(abs(y1 - y0) / speed / 0.02) + 300):
    y = min(y + d * speed * 0.02, y1) if d > 0 else max(y + d * speed * 0.02, y1)
    vel = np.array([0.0, d * speed if y != y1 else 0.0, 0.0])
    s.set_command(quadrotor.Command(np.array([1.5, y, lite3.STAND_HEIGHT]), vel,
                                    np.zeros(3), yaw, 0.0))
    s.step(int(0.02 / lite3.TIMESTEP))
    peak = max(peak, s.pos[2])
  return peak


@needs_lite3
@pytest.mark.parametrize('up', ['ramp', 'stairs'])
def test_walks_over_the_house_ramp_platform_and_stairs(up):
  from star_nav_mujoco import house  # pylint: disable=g-import-not-at-top
  if up == 'ramp':
    s = _house((1.5, -4.0), math.pi / 2)
    peak = _walk_y(s, -4.0, 3.0, 0.45, math.pi / 2)
    end = 3.0
  else:
    s = _house((1.5, 3.0), -math.pi / 2)
    peak = _walk_y(s, 3.0, -4.0, 0.45, -math.pi / 2)
    end = -4.0
  assert peak > lite3.STAND_HEIGHT + house.RISE - 0.03  # on the platform
  assert abs(s.pos[1] - end) < 0.2
  assert not s.fallen and s.contact_steps == 0
  s.close()
