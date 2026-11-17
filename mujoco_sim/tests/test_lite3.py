"""Tests for the Lite3 quadruped.

The tracker tests always run. The walking and sensor tests need the
downloaded model and policy (``star-nav-mujoco --fetch-lite3``) and
onnxruntime; they are skipped without them.
"""

import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import lidar
from star_nav_mujoco import lite3
from star_nav_mujoco import quadrotor
from star_nav_mujoco import sim
from star_nav_mujoco import world


# -- tracker (no assets needed) -------------------------------------------------


def test_tracker_walks_sideways_in_the_body_frame():
  t = lite3.Tracker()
  # Facing +y; the setpoint is 0.2 m along +x: that is to the right.
  fwd, side, turn = t.twist([0.2, 0.0, 0.3], np.zeros(3), math.pi / 2,
                            np.zeros(3), math.pi / 2)
  assert abs(fwd) < 1e-6 and side == pytest.approx(-t.k_pos * 0.2)
  assert turn == 0.0


def test_tracker_follows_setpoint_velocity_within_the_limit():
  t = lite3.Tracker()
  fwd, side, _ = t.twist(np.zeros(3), [2.0, 0.0, 0.0], 0.0, np.zeros(3), 0.0)
  assert fwd == pytest.approx(t.max_speed) and side == 0.0


def test_tracker_turns_towards_the_planner_yaw():
  t = lite3.Tracker()
  _, _, turn = t.twist(np.zeros(3), np.zeros(3), 1.0, np.zeros(3), 0.0)
  assert turn == pytest.approx(t.max_turn)
  _, _, turn = t.twist(np.zeros(3), np.zeros(3), -0.2, np.zeros(3), 0.0)
  assert turn == pytest.approx(-t.k_yaw * 0.2)


def test_tracker_stands_at_a_resting_setpoint():
  t = lite3.Tracker()
  twist = t.twist([0.01, 0.0, 0.3], np.zeros(3), 0.01, np.zeros(3), 0.0)
  np.testing.assert_array_equal(twist, 0.0)


# -- simulation (needs the downloaded robot) ------------------------------------


def _assets_available() -> bool:
  try:
    lite3.check_assets()
  except lite3.MissingAssets:
    return False
  return True


needs_lite3 = pytest.mark.skipif(
    not _assets_available(),
    reason='Lite3 not downloaded (star-nav-mujoco --fetch-lite3)')


def _robot(obstacles, **kw):
  cfg = sim.SimConfig(world=world.WorldConfig(size_x=10, size_y=10),
                      start=(0.0, 0.0, lite3.STAND_HEIGHT), robot=sim.LITE3, **kw)
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
  assert not s._fallen and s.contact_steps == 0
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
