"""Tests for the standalone simulator (no ROS required)."""

import dataclasses
import math
import os
import shutil

import mujoco

os.environ.setdefault('MUJOCO_GL', 'egl')

import numpy as np
import pytest

from star_nav_mujoco import assets
from star_nav_mujoco import commands
from star_nav_mujoco import lidar
from star_nav_mujoco import lidar_models
from star_nav_mujoco import overlay
from star_nav_mujoco import protocol
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
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
  assert protocol.LIDAR.size == 72
  assert protocol.VEL.size == 40
  assert protocol.DRAW.size == 8


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

  t, twist = protocol.unpack_vel(protocol.pack_vel(2., 0.5, -0.1, 0.3))
  assert t == 2.
  np.testing.assert_array_equal(twist, [0.5, -0.1, 0.3])


def test_hover_is_stable():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.step(int(5 / s.model.opt.timestep))
  np.testing.assert_allclose(s.pos, [0, 0, 1], atol=0.01)
  assert s.contact_steps == 0


def test_step_response():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.set_command(commands.Setpoint.hover([2.0, 0.0, 1.0], yaw=1.0))
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
    s.set_command(commands.Setpoint(
        np.array([r * c - r, r * si, 1.0]), np.array([-r * w * si, r * w * c, 0]),
        np.array([-r * w * w * c, -r * w * w * si, 0]), yaw=w * t, yaw_rate=w))
    s.step()
    if t > 3.0:
      errors.append(np.linalg.norm(s.pos - s._cmd.pos))  # pylint: disable=protected-access
  assert np.mean(errors) < 0.08
  assert np.max(errors) < 0.15


def test_holds_position_when_commands_stop():
  s = sim.Simulator(render_depth=False, obstacles=[])
  s.set_command(commands.Setpoint(np.array([0.5, 0, 1]), vel=np.array([3, 0, 0])))
  s.step(int(3 / s.model.opt.timestep))
  np.testing.assert_allclose(s.pos, [0.5, 0, 1], atol=0.05)


def test_collision_is_detected():
  wall = [world.Obstacle(world.PILLAR, 1.5, 0.0, 0.0, 0.4, 3.0, 0.0)]
  s = sim.Simulator(render_depth=False, obstacles=wall)
  s.set_command(commands.Setpoint.hover([3.0, 0.0, 1.0]))
  s.step(int(3 / s.model.opt.timestep))
  assert s.contact_steps > 0


@pytest.fixture(scope='module')
def depth_scene():
  pillar = world.Obstacle(world.PILLAR, 3.0, 0.0, 0.0, 0.5, 3.0, 0.0)
  s = sim.Simulator(obstacles=[pillar])
  s.set_command(commands.Setpoint.hover([0, 0, 1], yaw=0.3))
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


@pytest.mark.skipif(shutil.which('ffmpeg') is None, reason='needs ffmpeg')
def test_recorder_writes_video(tmp_path):
  from star_nav_mujoco import record  # pylint: disable=g-import-not-at-top
  s = sim.Simulator(render_depth=False)
  path = tmp_path / 'flight.mp4'
  rec = record.Recorder(s.model, str(path), width=320, height=240, fps=10.0)
  for _ in range(10):
    s.step(50)  # 0.1 s
    rec.maybe_capture(s.data)
  rec.close()
  s.close()
  assert rec.frames == 10
  assert path.stat().st_size > 1000


def test_depth_sees_nothing_at_the_setpoint():
  # The setpoint is not drawn; it must not appear as an obstacle.
  s = sim.Simulator(obstacles=[])
  s.set_command(commands.Setpoint.hover((0.6, 0.0, 1.0)))
  mujoco.mj_forward(s.model, s.data)
  centre = s.render_depth().image_mm[200:280, 280:360].astype(int)
  assert not ((centre > 0) & (centre < 2000)).any()  # nothing within 2 m


# -- lidar ----------------------------------------------------------------------


def _surface_distance(points, obstacles):
  """Distance from world points to the nearest ground, pillar or ring surface."""
  best = np.abs(points[:, 2])  # ground plane
  for ob in obstacles:
    if ob.kind == world.PILLAR:
      dr = np.hypot(points[:, 0] - ob.x, points[:, 1] - ob.y) - ob.a
      dz = points[:, 2] - ob.b
      inside = (dr <= 0) & (dz <= 0)
      d = np.where(inside, np.minimum(-dr, -dz),
                   np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)))
    else:
      # Rings are built from capsules between consecutive ring points.
      pts = np.array(world.ring_points(ob, scene.RING_SEGMENTS))
      a, b = pts[:-1], pts[1:]
      ab = b - a
      t = np.einsum('nkj,kj->nk', points[:, None] - a[None], ab) / (ab * ab).sum(1)
      t = np.clip(t, 0.0, 1.0)
      closest = a[None] + t[..., None] * ab[None]
      d = np.linalg.norm(points[:, None] - closest, axis=2).min(1) - scene.RING_TUBE
    best = np.minimum(best, np.abs(d))
  return best


@pytest.fixture(scope='module')
def lidar_scene():
  obstacles = [
      world.Obstacle(world.PILLAR, 3.0, 0.0, 0.0, 0.5, 2.5, 0.0),
      world.Obstacle(world.PILLAR, -2.0, 2.5, 0.0, 0.3, 0.8, 0.0),
      world.Obstacle(world.RING, 0.5, -2.5, 1.0, 0.6, 0.9, 0.4),
  ]
  cfg = sim.SimConfig(lidar=lidar.LidarConfig(range_max=8.0))
  s = sim.Simulator(cfg, render_depth=False, obstacles=obstacles)
  # Tilt and turn the body so the scan exercises the full sensor rotation.
  q = np.empty(4)
  mujoco.mju_euler2Quat(q, np.array([0.15, -0.1, 0.7]), 'xyz')
  s.data.qpos[3:7] = q
  mujoco.mj_forward(s.model, s.data)
  yield s, s.scan_lidar(), obstacles
  s.close()


def test_lidar_points_lie_on_surfaces(lidar_scene):
  s, scan, obstacles = lidar_scene
  rot = _quat_to_rot(scan.quat)
  world_pts = scan.pos + scan.points.astype(float) @ rot.T
  d = _surface_distance(world_pts, obstacles)
  assert len(world_pts) > 5000
  assert d.max() < 2e-3  # float32 points, exact ray casting


def test_lidar_pose_is_body_mount(lidar_scene):
  s, scan, _ = lidar_scene
  np.testing.assert_allclose(scan.pos, s.pos + s.rot @ np.array(s.cfg.lidar.mount))
  np.testing.assert_allclose(_quat_to_rot(scan.quat), s.rot, atol=1e-9)


def test_lidar_sees_all_obstacles_and_respects_range(lidar_scene):
  s, scan, obstacles = lidar_scene
  cfg = s.cfg.lidar
  r = np.linalg.norm(scan.points, axis=1)
  assert r.min() >= cfg.range_min and r.max() <= cfg.range_max
  rot = _quat_to_rot(scan.quat)
  world_pts = scan.pos + scan.points.astype(float) @ rot.T
  for ob in obstacles:
    near = np.hypot(world_pts[:, 0] - ob.x, world_pts[:, 1] - ob.y) < 1.3
    assert (near & (world_pts[:, 2] > 0.05)).sum() > 20, ob
  # Elevations stay within the vertical field of view (sensor frame).
  el = np.degrees(np.arcsin(scan.points[:, 2] / r))
  assert el.min() >= cfg.vfov[0] - 1e-3 and el.max() <= cfg.vfov[1] + 1e-3


def test_lidar_ignores_own_body_and_markers():
  cfg = sim.SimConfig(lidar=lidar.LidarConfig())
  s = sim.Simulator(cfg, render_depth=False, obstacles=[])
  scan = s.scan_lidar()
  r = np.linalg.norm(scan.points, axis=1)
  assert r.min() > 1.0  # nothing near the sensor; only ground returns
  world_z = scan.pos[2] + scan.points[:, 2]
  np.testing.assert_allclose(world_z, 0.0, atol=2e-3)
  s.close()


def test_lidar_noise():
  cfg = sim.SimConfig(lidar=lidar.LidarConfig(noise=0.03))
  s = sim.Simulator(cfg, render_depth=False, obstacles=[])
  scan = s.scan_lidar()
  world_z = scan.pos[2] + scan.points[:, 2]
  # Range noise along downward rays shows up as height error on the ground.
  r = np.linalg.norm(scan.points, axis=1)
  err = world_z / (-scan.points[:, 2] / r)
  assert 0.02 < err.std() < 0.04
  s.close()


def test_lidar_pitch_tilts_the_sensor():
  cfg = sim.SimConfig(lidar=lidar.LidarConfig(beams=1, vfov=(0.0, 0.0), pitch=30.0))
  s = sim.Simulator(cfg, render_depth=False, obstacles=[])
  scan = s.scan_lidar()
  world_pts = scan.pos + scan.points.astype(float) @ _quat_to_rot(scan.quat).T
  np.testing.assert_allclose(_quat_to_rot(scan.quat), scan.rot, atol=1e-9)
  # The horizontal beam now meets the ground ahead, at height / tan(30 deg).
  assert np.abs(world_pts[:, 2]).max() < 2e-3
  ahead = world_pts[np.argmax(world_pts[:, 0])]
  assert ahead[0] == pytest.approx(scan.pos[2] / np.tan(np.deg2rad(30.0)), abs=0.01)
  assert world_pts[:, 0].min() > 0  # nothing behind: those beams point up
  s.close()


def test_lidar_range_is_the_models_unless_given():
  for name in lidar_models.names():
    model = lidar_models.get(name)
    cfg = lidar.LidarConfig(model=name).resolved()
    assert (cfg.range_min, cfg.range_max) == (model.range_min, model.range_max)
    assert cfg.pitch == 0.0
  assert lidar.LidarConfig(model='mid360', range_max=12.0).resolved().range_max == 12.0
  with pytest.raises(ValueError, match='unknown lidar model'):
    lidar.LidarConfig(model='nope').resolved()


def test_uniform_model():
  cfg = lidar.LidarConfig(beams=4, columns=8, vfov=(-10.0, 20.0))
  model = lidar_models.get('uniform')
  table = model.table(cfg)
  assert model.repetitive and model.lines(cfg) == 4
  assert table.shape == (32, 3) and model.scan_size(cfg, table) == 32
  np.testing.assert_allclose(np.linalg.norm(table, axis=1), 1.0)
  elevation = np.rad2deg(np.arcsin(table[:, 2]))
  assert elevation.min() == pytest.approx(-10.0) and elevation.max() == pytest.approx(20.0)


def test_mid360_model():
  model = lidar_models.get('mid360')
  try:
    table = model.table(lidar.LidarConfig(model='mid360'))
  except assets.MissingAssets:
    pytest.skip('Mid-360 beam table not downloaded (--fetch mid360)')
  assert not model.repetitive
  np.testing.assert_allclose(np.linalg.norm(table, axis=1), 1.0)
  elevation = np.rad2deg(np.arcsin(table[:, 2]))
  assert -7.3 < elevation.min() < -7.0 and 52.0 < elevation.max() < 52.3

  cfg = sim.SimConfig(lidar=lidar.LidarConfig(model='mid360', pitch=45.0))
  s = sim.Simulator(cfg, render_depth=False, obstacles=[])
  assert s.cfg.lidar.range_max == model.range_max
  first, second = s.scan_lidar(), s.scan_lidar()
  assert len(s.lidar.directions) == 20000  # 200,000 points/s at 10 Hz
  for scan in (first, second):
    world_pts = scan.pos + scan.points.astype(float) @ scan.rot.T
    assert len(world_pts) > 1000
    assert np.abs(world_pts[:, 2]).max() < 2e-3  # all on the ground
  # Non-repetitive: the next scan uses the next part of the table.
  assert not np.array_equal(first.points[:100], second.points[:100])
  s.close()


def test_mid360_table_missing(tmp_path, monkeypatch):
  monkeypatch.setenv('STAR_NAV_LIDAR_DIR', str(tmp_path))
  with pytest.raises(assets.MissingAssets, match='--fetch mid360'):
    lidar.Lidar(None, lidar.LidarConfig(model='mid360'), 0)


def test_lidar_protocol_round_trip(lidar_scene):
  _, scan, _ = lidar_scene
  t, pos, quat, beams, pts = protocol.unpack_lidar(protocol.pack_lidar(scan, 32))
  assert t == scan.t and beams == 32
  np.testing.assert_array_equal(pts, scan.points)
  np.testing.assert_array_equal(quat, scan.quat)
  assert protocol.LIDAR.size == 72
  assert protocol.VEL.size == 40


def test_quadrotor_does_not_follow_a_velocity_command():
  s = sim.Simulator(obstacles=[], render_depth=False)
  assert s.controller.accepts == (commands.Setpoint,)
  assert not s.set_velocity([0.5, 0.0, 0.0])
  s.step(500)
  np.testing.assert_allclose(s.pos, s.cfg.start, atol=0.02)
  s.close()


def test_draw_protocol_round_trip():
  pts = np.array([[0., 0., 0.], [1., 2., 3.]])
  layer, back = protocol.unpack_draw(protocol.pack_draw(overlay.ROUTE, pts))
  assert layer == overlay.ROUTE
  np.testing.assert_array_equal(back, pts)
  layer, back = protocol.unpack_draw(protocol.pack_draw(overlay.PLAN, []))
  assert layer == overlay.PLAN and back.shape == (0, 3)
  with pytest.raises(protocol.ProtocolError):
    protocol.unpack_draw(protocol.pack_draw(overlay.PLAN, pts)[:-4])


def test_overlay_resample():
  line = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.]])
  pts = overlay.resample(line, 0.1, 100)
  assert len(pts) == 21
  np.testing.assert_allclose(pts[10], [1., 0., 0.], atol=1e-9)
  np.testing.assert_allclose(pts[-1], [1., 1., 0.], atol=1e-9)
  # Too long for the pool: the same span with wider spacing.
  pts = overlay.resample(line, 0.1, 10)
  assert len(pts) == 11
  np.testing.assert_allclose(pts[-1], [1., 1., 0.], atol=1e-9)
  # Nothing to draw.
  assert len(overlay.resample(line[:1], 0.1, 10)) == 0
  assert len(overlay.resample(np.zeros((3, 3)), 0.1, 10)) == 0


def test_paths_are_drawn_with_markers():
  cfg = sim.SimConfig(draw=True, lidar=lidar.LidarConfig())
  s = sim.Simulator(cfg, render_depth=False, obstacles=[])
  m, d = s.model, s.data

  def placed(name):
    spec = overlay.LAYERS[name]
    ids = [m.body_mocapid[m.body(f'draw/{spec.name}/{i}').id]
           for i in range(spec.count)]
    return d.mocap_pos[ids][d.mocap_pos[ids][:, 2] > 0]

  assert not len(placed(overlay.PLAN))
  assert s.draw(overlay.PLAN, [[0., 0., 0.5], [0.6, 0., 0.5]])
  seg = placed(overlay.PLAN)
  assert len(seg) == 4  # 0.6 m in 0.15 m pieces
  np.testing.assert_allclose(seg[0], [0.075, 0., 0.5], atol=1e-9)
  # A capsule's axis (z) lies along the path.
  quat = d.mocap_quat[m.body_mocapid[m.body('draw/plan/0').id]]
  np.testing.assert_allclose(_quat_to_rot(quat)[:, 2], [1., 0., 0.], atol=1e-9)

  dots = np.random.default_rng(0).uniform(0.1, 1.0, (1000, 3))
  assert s.draw(overlay.CANDIDATES, dots)
  assert len(placed(overlay.CANDIDATES)) == overlay.LAYERS[overlay.CANDIDATES].count

  wps = [[1., 1., 0.3], [2., 1., 0.3], [2., 2., 0.3]]
  assert s.draw(overlay.WAYPOINTS, wps)
  np.testing.assert_allclose(placed(overlay.WAYPOINTS), wps)

  # A new path replaces the old one; an empty one clears the layer.
  assert s.draw(overlay.PLAN, [[0., 0., 0.5], [0.15, 0., 0.5]])
  assert len(placed(overlay.PLAN)) == 1
  assert s.draw(overlay.PLAN, [])
  assert not len(placed(overlay.PLAN))
  assert not s.draw(7, [[0., 0., 0.]])
  assert not s.draw(overlay.PLAN, [[0., 0., math.nan], [1., 0., 0.]])

  # The markers touch nothing, and the lidar does not see them.
  s.draw(overlay.PLAN, [[-1., 0., 1.], [1., 0., 1.]])  # through the quadrotor
  s.draw(overlay.ROUTE, [[-1., 0., 0.5], [1., 0., 0.5]])  # under it
  s.step(50)
  assert s.contact_steps == 0
  scan = s.scan_lidar()
  np.testing.assert_allclose(scan.pos[2] + scan.points[:, 2], 0.0, atol=2e-3)
  # It scans a copy of the scene without them: the same scan as without.
  plain = sim.Simulator(dataclasses.replace(cfg, draw=False),
                        render_depth=False, obstacles=[world.Obstacle(
                            world.PILLAR, 2.0, 0.5, 0.0, 0.3, 2.0, 0.0)])
  both = sim.Simulator(cfg, render_depth=False, obstacles=plain.obstacles)
  for sm in (plain, both):
    sm.step(50)
  a, b = plain.scan_lidar(), both.scan_lidar()
  assert len(a.points) > 1000 and np.linalg.norm(a.points, axis=1).min() < 2.5
  np.testing.assert_allclose(a.pos, b.pos, atol=1e-9)
  np.testing.assert_allclose(a.points, b.points, atol=1e-6)
  plain.close()
  both.close()
  groups = np.zeros(6, np.uint8)
  groups[scene.MARKER_GROUP] = 1
  geom = np.zeros(1, np.int32)
  assert mujoco.mj_ray(m, d, [0., 0., 3.], [0., 0., -1.], groups, 0, -1, geom) > 0
  assert m.geom_group[geom[0]] == scene.MARKER_GROUP
  s.close()


def test_no_markers_unless_asked():
  s = sim.Simulator(obstacles=[], render_depth=False)
  assert s.model.nmocap == 1  # the setpoint
  assert s.draw(overlay.PLAN, [[0., 0., 0.5], [0.6, 0., 0.5]])
  assert not s.draw(7, [])
  s.close()


def test_non_finite_command_fields_are_ignored():
  # A NaN yaw rate once reached the motors and the quadrotor fell.
  s = sim.Simulator(render_depth=False, obstacles=[])
  bad = commands.Setpoint(np.array([0.0, 0.0, 1.0]), np.array([np.nan, 0.0, 0.0]),
                          np.zeros(3), float('nan'), float('nan'))
  assert not s.set_command(bad)
  s.step(1000)  # 2 s
  assert np.all(np.isfinite(s.data.ctrl))
  assert s.pos[2] == pytest.approx(1.0, abs=0.05)
  # A non-finite position is rejected; the previous target stays.
  assert not s.set_command(commands.Setpoint.hover((np.inf, 0.0, 1.0)))
  s.step(500)
  np.testing.assert_allclose(s.pos, [0.0, 0.0, 1.0], atol=0.05)
