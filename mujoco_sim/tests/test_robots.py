"""The robot registry, and what every robot has to provide.

Also checks the numbers each robot's launch file repeats (roslaunch cannot
read them from here) against the robot's descriptor.
"""

import pathlib
import sys
import xml.etree.ElementTree as ET

import pytest

from star_nav_mujoco import assets
from star_nav_mujoco import robots
from star_nav_mujoco import sim

LAUNCH = pathlib.Path(__file__).parents[2] / 'star_nav_bringup' / 'launch'
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
