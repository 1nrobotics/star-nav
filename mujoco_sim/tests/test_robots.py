"""The robot registry, and what every robot has to provide.

Also checks the robot's description, which the planner's side reads: that
its sizes are the model's, that the file in star_nav_bringup/config/robots/
is current, and the numbers each robot's launch file repeats (roslaunch
cannot read an argument from a file).
star_nav_bringup/test/test_planner_interface.py checks what the nodes are
given in the end.
"""

import pathlib
import sys
import xml.etree.ElementTree as ET

import pytest

from star_nav_mujoco import assets
from star_nav_mujoco import robots
from star_nav_mujoco import sim

BRINGUP = pathlib.Path(__file__).parents[2] / 'star_nav_bringup'
LAUNCH = BRINGUP / 'launch'
GROUND = [n for n in robots.names() if robots.get(n).on_ground]


def test_the_quadrotor_is_the_default_and_listed_first():
  assert robots.names()[0] == robots.DEFAULT == 'quadrotor'
  assert sim.SimConfig().robot == 'quadrotor'
  assert {'lite3', 't1'} <= set(robots.names())


def test_an_unknown_robot_is_rejected():
  with pytest.raises(ValueError, match='unknown robot'):
    robots.get('hexapod')
  with pytest.raises(ValueError, match='unknown robot'):
    sim.Simulator(sim.SimConfig(robot='hexapod'), render_depth=False)


@pytest.mark.parametrize('name', robots.names())
def test_descriptor_is_complete(name):
  robot = robots.get(name)
  assert robot.name == name and robot.title
  assert robot.body and robot.camera
  assert robot.world in ('forest', 'house')
  assert len(robot.start) == 3 and len(robot.follow_camera) == 2
  low, high = robot.lidar_vfov
  assert -90 < low < high < 90
  assert robot.type in ('aerial', 'legged')
  assert len(robot.size) == 3 and min(robot.size) > 0
  if robot.on_ground:
    assert 0 < robot.stand_height <= robot.start_height
    assert robot.start[2] == robot.stand_height
    assert robot.support_geoms and len(robot.lidar_mount) == 3
    assert robot.body.startswith(name + '/')  # names carry the robot's prefix
    assert callable(robot.fetch)


@pytest.mark.parametrize('name', GROUND)
def test_missing_files_name_the_download_command(name, tmp_path, monkeypatch):
  monkeypatch.setenv(f'STAR_NAV_{name.upper()}_DIR', str(tmp_path))
  with pytest.raises(assets.MissingAssets, match=f'--fetch {name}'):
    sim.Simulator(sim.SimConfig(robot=name), render_depth=False)


def test_the_quadrotor_is_found_in_its_scene():
  s = sim.Simulator(sim.SimConfig(), render_depth=False, obstacles=[])
  try:
    assert robots.find(s.model) is robots.get('quadrotor')
    assert not s.fallen
  finally:
    s.close()


def test_listing_the_robots_does_not_import_mujoco():
  # MuJoCo fixes its OpenGL backend on import; the command line lists the
  # robots before it has chosen one.
  import subprocess
  code = ('import sys; from star_nav_mujoco import robots; robots.names(); '
          'sys.exit("mujoco" in sys.modules)')
  assert subprocess.run([sys.executable, '-c', code], check=False).returncode == 0


def test_the_robot_mounts_its_lidar():
  from star_nav_mujoco import lidar
  from star_nav_mujoco import lidar_models

  for name in robots.names():
    robot = robots.get(name)
    assert set(robot.lidar_pitch) <= set(lidar_models.names()), name
    cfg = robot.mount_lidar(lidar.LidarConfig())
    assert cfg.mount == (robot.lidar_mount or lidar.LidarConfig().mount)
    assert cfg.pitch == 0.0
  t1 = robots.get('t1')
  # Tilted for the Mid-360 unless the caller says otherwise.
  assert t1.mount_lidar(lidar.LidarConfig(model='mid360')).pitch == 30.0
  assert t1.mount_lidar(lidar.LidarConfig(model='mid360', pitch=0.0)).pitch == 0.0


@pytest.mark.skipif(not LAUNCH.is_dir(), reason='star_nav_bringup not here')
@pytest.mark.parametrize('name', GROUND)
def test_launch_file_agrees_with_the_robot(name):
  robot = robots.get(name)
  root = ET.parse(LAUNCH / f'exploration_{name}.launch').getroot()
  args = {a.get('name'): a.get('value') or a.get('default')
          for a in root.iter('arg')}
  assert args['robot'] == name
  # The planner flies its "drone" at the height the robot's body stands at.
  assert float(args['altitude']) == pytest.approx(robot.stand_height)
  mount = [float(v) for v in args['lidar_offset'].strip('[]').split(',')]
  assert mount == pytest.approx(robot.lidar_mount)
  assert args['robot_type'] == robot.type
  size = [float(args[k]) for k in ('robot_length', 'robot_width')]
  assert size == pytest.approx(robot.size[:2])


def test_describe_gives_what_the_planner_side_needs():
  quad = robots.get('quadrotor').describe()
  assert quad == {'name': 'quadrotor', 'type': 'aerial', 'length': 0.37,
                  'width': 0.37, 'height': 0.1, 'lidar_mount': [0.0, 0.0, 0.06]}
  for name in GROUND:
    robot = robots.get(name)
    d = robot.describe()
    assert set(d) == set(quad) | {'stand_height', 'max_speed', 'max_turn'}
    assert (d['length'], d['width'], d['height']) == robot.size
    assert d['lidar_mount'] == list(robot.lidar_mount)
    assert d['stand_height'] == robot.stand_height
    # A command is kept within the tracker's limits.
    twist = robot.tracker().limit([9.0, 0.0, 9.0])
    assert (d['max_speed'], d['max_turn']) == pytest.approx(twist[[0, 2]])


@pytest.mark.skipif(not BRINGUP.is_dir(), reason='star_nav_bringup not here')
@pytest.mark.parametrize('name', robots.names())
def test_description_file_is_current(name, capsys):
  from star_nav_mujoco import __main__ as cli

  assert cli.main(['--describe', name]) == 0
  text = capsys.readouterr().out
  assert text == robots.description(name)
  path = BRINGUP / 'config' / 'robots' / f'{name}.yaml'
  assert path.read_text() == text, (
      f'{path.name} is not what robots/{name}.py describes: write it again '
      f'(the command is in the file)')


def test_every_description_file_is_a_robots():
  if not BRINGUP.is_dir():
    pytest.skip('star_nav_bringup not here')
  files = {p.stem for p in (BRINGUP / 'config' / 'robots').glob('*.yaml')}
  assert files == set(robots.names())


@pytest.mark.parametrize('name', robots.names())
def test_size_is_the_models(name):
  """The box around the standing (or level) robot with its lidar."""
  import mujoco
  import numpy as np
  from star_nav_mujoco import lidar
  from star_nav_mujoco import world

  robot = robots.get(name)
  try:
    robot.check_assets()
  except assets.MissingAssets:
    pytest.skip(f'{name} not downloaded (star-nav-mujoco --fetch {name})')
  cfg = sim.SimConfig(world=world.WorldConfig(size_x=10, size_y=10),
                      start=tuple(robot.start), robot=name,
                      lidar=lidar.LidarConfig())
  s = sim.Simulator(cfg, obstacles=[], render_depth=False)
  try:
    s.step(int(2.0 / s.model.opt.timestep))
    m, d = s.model, s.data
    root = m.body(robot.body).id
    pos, rot = d.xpos[root], d.xmat[root].reshape(3, 3)
    points = []
    for g in range(m.ngeom):
      if m.body_rootid[m.geom_bodyid[g]] != root:
        continue
      if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
        i = m.geom_dataid[g]
        local = m.mesh_vert[m.mesh_vertadr[i]:m.mesh_vertadr[i] + m.mesh_vertnum[i]]
      else:  # the corners of its bounding box
        c, h = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
        local = np.array([c + h * [x, y, z] for x in (-1, 1) for y in (-1, 1)
                          for z in (-1, 1)])
      w = d.geom_xpos[g] + local @ d.geom_xmat[g].reshape(3, 3).T
      # In the body frame; a ground robot's height is above the ground.
      points.append(np.column_stack([(w - pos) @ rot, w[:, 2]]))
    points = np.concatenate(points)
    low, high = points.min(0), points.max(0)
    height = high[3] if robot.on_ground else high[2] - low[2]
    assert (high[0] - low[0], high[1] - low[1], height) == pytest.approx(
        robot.size, abs=0.01)
  finally:
    s.close()
