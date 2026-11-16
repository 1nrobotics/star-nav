"""Tests for the standalone simulator (no ROS required)."""

import math
import os

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import protocol
from star_nav_mujoco import quadrotor
from star_nav_mujoco import sim
from star_nav_mujoco import world


def _quat_to_rot(q):
  w, x, y, z = q
  return np.array([
      [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  ])


def test_world_is_deterministic_and_keeps_start_clear():
  cfg = world.WorldConfig(seed=7)
  a, b = world.generate(cfg), world.generate(cfg)
  assert a == b
  assert len(a) == cfg.pillar_num + cfg.ring_num
  assert world.generate(world.WorldConfig(seed=8)) != a
  for ob in a:
    assert math.hypot(ob.x, ob.y) >= cfg.start_clearance


def test_protocol_sizes_match_cpp_bridge():
  # Same values as the static_asserts in mujoco_bridge/src/protocol.h.
  assert protocol.STATE.size == 124
  assert protocol.DEPTH.size == 88
  assert protocol.WORLD.size == 8
  assert protocol.OBSTACLE.size == 28
  assert protocol.CMD.size == 104


def test_protocol_round_trip():
  s = protocol.State(1.5, np.array([1., 2., 3.]), np.array([1., 0., 0., 0.]),
                     np.array([.1, .2, .3]), np.array([.4, .5, .6]), 3)
  r = protocol.unpack_state(protocol.pack_state(s))
  assert r.t == s.t and r.contacts == 3
  np.testing.assert_array_equal(r.vel, s.vel)

  img = np.arange(12, dtype=np.uint16).reshape(3, 4)
  d = protocol.Depth(2.0, np.zeros(3), np.array([1., 0., 0., 0.]),
                     (1., 2., 3., 4.), img)
  r = protocol.unpack_depth(protocol.pack_depth(d))
  np.testing.assert_array_equal(r.image_mm, img)
  assert r.intrinsics == (1., 2., 3., 4.)

  obs = world.generate(world.WorldConfig(pillar_num=3, ring_num=2))
  back = protocol.unpack_world(protocol.pack_world(obs))
  assert len(back) == 5
  assert back[0].kind == world.PILLAR and back[-1].kind == world.RING
  assert back[0].x == pytest.approx(obs[0].x, abs=1e-5)

  cmd = protocol.unpack_cmd(protocol.pack_cmd(1., [1, 2, 3], [0, 0, 1],
                                              [0, 1, 0], 0.5, 0.1))
  assert cmd[4] == 0.5
  np.testing.assert_array_equal(cmd[1], [1, 2, 3])

  bad = bytearray(protocol.pack_cmd(1., [0] * 3, [0] * 3, [0] * 3, 0., 0.))
  bad[0] = 99
  with pytest.raises(protocol.ProtocolError):
    protocol.unpack_cmd(bytes(bad))


def test_hover_is_stable():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.step(int(5 / s.model.opt.timestep))
  np.testing.assert_allclose(s.pos, [0, 0, 1], atol=0.01)
  assert s.contact_steps == 0


def test_step_response():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.set_command(quadrotor.Command.hover([2.0, 0.0, 1.0], yaw=1.0))
  peak = 0.0
  for _ in range(int(4 / s.model.opt.timestep)):
    s.step()
    peak = max(peak, s.pos[0])
  assert peak - 2.0 < 0.1
  np.testing.assert_allclose(s.pos, [2, 0, 1], atol=0.05)
  assert quadrotor.yaw_of(s.rot) == pytest.approx(1.0, abs=0.02)


def test_tracks_planner_style_trajectory():
  s = sim.Simulator(render_depth=False, obstacles=[])
  r, w = 2.0, 0.75  # 1.5 m/s circle
  errors = []
  for _ in range(int(10 / s.model.opt.timestep)):
    t = s.time
    c, si = math.cos(w * t), math.sin(w * t)
    s.set_command(quadrotor.Command(
        np.array([r * c - r, r * si, 1.0]), np.array([-r * w * si, r * w * c, 0]),
        np.array([-r * w * w * c, -r * w * w * si, 0]), yaw=w * t, yaw_rate=w))
    s.step()
    if t > 3.0:
      errors.append(np.linalg.norm(s.pos - s._cmd.pos))  # pylint: disable=protected-access
  assert np.mean(errors) < 0.08
  assert np.max(errors) < 0.15


def test_holds_position_when_commands_stop():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.set_command(quadrotor.Command(np.array([0.5, 0, 1]), vel=np.array([3, 0, 0])))
  s.step(int(3 / s.model.opt.timestep))
  np.testing.assert_allclose(s.pos, [0.5, 0, 1], atol=0.05)


def test_collision_is_detected():
  wall = [world.Obstacle(world.PILLAR, 1.5, 0.0, 0.0, 0.4, 3.0, 0.0)]
  s = sim.Simulator(render_depth=False, obstacles=wall)
  s.set_command(quadrotor.Command.hover([3.0, 0.0, 1.0]))
  s.step(int(3 / s.model.opt.timestep))
  assert s.contact_steps > 0


@pytest.fixture(scope='module')
def depth_scene():
  pillar = world.Obstacle(world.PILLAR, 3.0, 0.0, 0.0, 0.5, 3.0, 0.0)
  s = sim.Simulator(obstacles=[pillar])
  s.set_command(quadrotor.Command.hover([0, 0, 1], yaw=0.3))
  s.step(int(3 / s.model.opt.timestep))
  yield s, s.render_depth()
  s.close()


def _back_project(d, u, v):
  fx, fy, cx, cy = d.intrinsics
  z = d.image_mm[v, u] / 1000.0
  p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
  return _quat_to_rot(d.quat) @ p_cam + d.pos


def test_depth_image_format(depth_scene):
  _, d = depth_scene
  assert d.image_mm.shape == (480, 640) and d.image_mm.dtype == np.uint16
  assert (d.image_mm[0] == 0).mean() > 0.5  # mostly sky: no return
  assert (d.image_mm > 0).mean() > 0.3


def test_depth_back_projects_onto_ground(depth_scene):
  """Ground pixels must land on z = 0 using the published pose and intrinsics."""
  _, d = depth_scene
  for u in (40, 320, 600):
    for v in (380, 470):
      if d.image_mm[v, u]:
        assert _back_project(d, u, v)[2] == pytest.approx(0.0, abs=0.03)


def test_depth_back_projects_onto_pillar(depth_scene):
  s, d = depth_scene
  # Pixel that looks at the pillar centre at camera height.
  rot = _quat_to_rot(d.quat)
  p_cam = rot.T @ (np.array([3.0, 0.0, d.pos[2]]) - d.pos)
  fx, fy, cx, cy = d.intrinsics
  u = int(round(fx * p_cam[0] / p_cam[2] + cx))
  v = int(round(fy * p_cam[1] / p_cam[2] + cy))
  p = _back_project(d, u, v)
  assert math.hypot(p[0] - 3.0, p[1]) == pytest.approx(0.5, abs=0.03)
  # Optical axis is the body x axis.
  np.testing.assert_allclose(rot[:, 2], s.rot[:, 0], atol=1e-6)
