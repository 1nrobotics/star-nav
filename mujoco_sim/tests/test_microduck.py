"""Tests for the Microduck biped.

The tracker tests always run. The walking and camera tests need the
downloaded model and policy (``star-nav-mujoco --fetch-microduck``) and the
optional dependencies; they are skipped without them.
"""

import dataclasses
import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import microduck
from star_nav_mujoco import quadrotor
from star_nav_mujoco import sim
from star_nav_mujoco import world


# -- tracker (no assets needed) -------------------------------------------------


def test_tracker_walks_forward_towards_setpoint():
  t = microduck.Tracker()
  vx, vy, wz = t.twist([1.0, 0.1, 0.1], [0.0, 0.0, 0.0], np.zeros(3), 0.0)
  assert vy == 0.0
  assert t.min_walk_cmd <= vx <= t.max_walk_cmd
  assert wz > 0.0  # setpoint slightly to the left


def test_tracker_maps_planner_speed_through_the_policy_response():
  t = microduck.Tracker(k_pos=0.0)
  vx, _, _ = t.twist([0.0, 0.0, 0.1], [0.2, 0.0, 0.0], np.zeros(3), 0.0)
  # The policy walks at 0.69 * cmd - 0.09 m/s.
  assert t.speed_gain * vx - t.speed_offset == pytest.approx(0.2, abs=1e-6)


def test_tracker_turns_in_place_for_large_heading_errors():
  t = microduck.Tracker()
  vx, _, wz = t.twist([-1.0, 0.0, 0.1], [0.0, 0.0, 0.0], np.zeros(3), 0.0)
  assert vx == 0.0 and abs(wz) == t.turn_in_place


def test_tracker_stands_at_a_resting_setpoint():
  t = microduck.Tracker()
  twist = t.twist([0.05, 0.0, 0.1], [0.0, 0.0, 0.0], np.zeros(3), 1.0)
  np.testing.assert_array_equal(twist, 0.0)


def test_tracker_waits_when_ahead_of_a_slow_setpoint():
  """Walking faster than the planner must not make the duck turn back."""
  t = microduck.Tracker()
  # Setpoint 0.15 m behind, moving forward (+x) at 0.05 m/s.
  twist = t.twist([-0.15, 0.01, 0.1], [0.05, 0.0, 0.0], np.zeros(3), 0.0)
  np.testing.assert_array_equal(twist, 0.0)
  # Once it has passed the duck, walk again.
  vx, _, _ = t.twist([0.12, 0.0, 0.1], [0.05, 0.0, 0.0], np.zeros(3), 0.0)
  assert vx >= t.min_walk_cmd


# -- simulation (needs the downloaded duck) -------------------------------------


def _assets_available() -> bool:
  try:
    microduck.check_assets()
  except microduck.MissingAssets:
    return False
  return True


needs_duck = pytest.mark.skipif(
    not _assets_available(),
    reason='Microduck not downloaded (star-nav-mujoco --fetch-microduck)')


def _duck(obstacles, render_depth=False):
  cfg = sim.SimConfig(
      world=dataclasses.replace(world.WorldConfig(), **world.MICRODUCK_FOREST),
      start=(0.0, 0.0, microduck.STAND_HEIGHT), robot=sim.MICRODUCK)
  return sim.Simulator(cfg, render_depth=render_depth, obstacles=obstacles)


@needs_duck
def test_duck_stands_still():
  s = _duck([])
  s.step(int(3 / s.model.opt.timestep))
  assert np.hypot(*s.pos[:2]) < 0.03
  assert s.pos[2] == pytest.approx(0.116, abs=0.01)
  assert s.rot[2, 2] > 0.99  # upright


@needs_duck
def test_duck_walks_to_a_setpoint_and_stops():
  s = _duck([])
  goal = np.array([1.0, 1.0, 0.12])  # 45 deg to the left: turns, then walks
  s.set_command(quadrotor.Command.hover(goal))
  s.step(int(20 / s.model.opt.timestep))
  assert np.hypot(*(s.pos[:2] - goal[:2])) < 0.15
  assert s.rot[2, 2] > 0.95
  assert s.contact_steps == 0


@needs_duck
def test_duck_contact_with_obstacle_is_detected_but_not_the_floor():
  pillar = [world.Obstacle(world.PILLAR, 0.6, 0.0, 0.0, 0.1, 1.0, 0.0)]
  s = _duck(pillar)
  s.step(int(2 / s.model.opt.timestep))
  assert s.contact_steps == 0  # standing on the floor
  s.set_command(quadrotor.Command.hover([2.0, 0.0, 0.12]))
  s.step(int(10 / s.model.opt.timestep))
  assert s.contact_steps > 0


@needs_duck
def test_duck_depth_back_projects_onto_floor_and_pillar():
  """The head camera's image, pose and intrinsics agree, also while walking."""
  pillar = world.Obstacle(world.PILLAR, 1.5, 0.0, 0.0, 0.1, 1.0, 0.0)
  s = _duck([pillar], render_depth=True)
  s.set_command(quadrotor.Command.hover([0.8, 0.0, 0.12]))
  try:
    for _ in range(5):
      s.step(int(0.5 / s.model.opt.timestep))
      d = s.render_depth()
      assert d.pos[2] == pytest.approx(0.23, abs=0.03)  # camera in the head
      fx, fy, cx, cy = d.intrinsics
      z = d.image_mm / 1000.0
      v, u = np.nonzero(z > 0)
      pc = np.stack([(u - cx) * z[v, u] / fx, (v - cy) * z[v, u] / fy, z[v, u]], 1)
      w, x, y, q = d.quat
      rot = np.array([
          [1 - 2 * (y * y + q * q), 2 * (x * y - w * q), 2 * (x * q + w * y)],
          [2 * (x * y + w * q), 1 - 2 * (x * x + q * q), 2 * (y * q - w * x)],
          [2 * (x * q - w * y), 2 * (y * q + w * x), 1 - 2 * (x * x + y * y)]])
      p = pc @ rot.T + d.pos
      on_pillar = np.abs(np.hypot(p[:, 0] - 1.5, p[:, 1]) - 0.1) < 0.02
      on_floor = np.abs(p[:, 2]) < 0.02
      # Every return is on the floor or the pillar: no part of the duck,
      # no setpoint marker, no projection error.
      assert (on_pillar | on_floor).mean() > 0.995
      assert on_pillar.sum() > 1000
      assert z[z > 0].min() > 0.15  # the beak and body are closer than this
  finally:
    s.close()
