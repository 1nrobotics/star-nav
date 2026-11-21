"""Command-line entry point: ``python -m star_nav_mujoco``."""

from __future__ import annotations

import argparse
import logging
import os
import signal
import subprocess
import sys


def _select_gl() -> None:
  """Picks a headless OpenGL backend unless MUJOCO_GL is already set."""
  if os.environ.get('MUJOCO_GL'):
    return
  if sys.platform == 'darwin':
    return  # CGL works headless on macOS
  # PyOpenGL fixes its platform on first import, so probe EGL in a child.
  probe = ('import mujoco; m = mujoco.MjModel.from_xml_string("<mujoco/>"); '
           'mujoco.Renderer(m, 8, 8).close()')
  env = dict(os.environ, MUJOCO_GL='egl', PYOPENGL_PLATFORM='egl')
  ok = subprocess.run([sys.executable, '-c', probe], env=env, check=False,
                      stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL).returncode == 0
  os.environ['MUJOCO_GL'] = 'egl' if ok else 'osmesa'


# Per-robot defaults: walking robots need no rings, start standing, and show
# off in the house (ramp, stairs).
_DEFAULTS = {
    'quadrotor': dict(rings=50, start=(0.0, 0.0, 1.0), world='forest'),
    'lite3': dict(rings=0, start=(0.0, 0.0, 0.34), world='house'),
    't1': dict(rings=0, start=(0.0, 0.0, 0.67), world='house'),
    'beni': dict(rings=0, start=(0.0, 0.0, 0.11), world='house'),
}


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(
      prog='star_nav_mujoco',
      description='MuJoCo simulator for Star Nav (quadrotor, Lite3 '
      'quadruped, T1 humanoid or Beni). Talks to the ROS bridge (mujoco_bridge) '
      'over ZeroMQ and serves a web viewer.')
  ap.add_argument('--robot', choices=('quadrotor', 'lite3', 't1', 'beni'),
                  default='quadrotor')
  ap.add_argument('--fetch-lite3', action='store_true',
                  help='download the Lite3 model and walking policy, then '
                  'exit')
  ap.add_argument('--fetch-t1', action='store_true',
                  help='download the T1 model and walking policy, then exit')
  ap.add_argument('--fetch-beni', action='store_true',
                  help='download the Beni model and policy, then exit')
  ap.add_argument('--world', choices=('forest', 'house'), default=None,
                  help='forest (random pillars and rings) or house (rooms, a '
                  'ramp and stairs; house.py). Default: forest (quadrotor), '
                  'house (lite3, t1, beni)')
  ap.add_argument('--seed', type=int, default=1, help='forest seed')
  ap.add_argument('--map-size', type=float, nargs=2, default=(50.0, 50.0),
                  metavar=('X', 'Y'))
  ap.add_argument('--pillars', type=int, default=100)
  ap.add_argument('--rings', type=int, default=None,
                  help='default: 50 (quadrotor), 0 (lite3, t1, beni)')
  ap.add_argument('--start', type=float, nargs=3, default=None,
                  metavar=('X', 'Y', 'Z'), help='default: 0 0 1 (quadrotor); '
                  'walking robots ignore Z')
  ap.add_argument('--bind', default='127.0.0.1',
                  help='address for the ZeroMQ sockets (use 0.0.0.0 to '
                  'accept a bridge on another host)')
  ap.add_argument('--state-port', type=int, default=5555)
  ap.add_argument('--cmd-port', type=int, default=5556)
  ap.add_argument('--sensor', choices=('depth', 'lidar', 'both'),
                  default='depth', help='onboard sensor(s) to simulate')
  ap.add_argument('--depth-rate', type=float, default=30.0)
  lid = ap.add_argument_group('lidar (with --sensor lidar or both)')
  lid.add_argument('--lidar-beams', type=int, default=32)
  lid.add_argument('--lidar-columns', type=int, default=1024,
                   help='azimuth samples per revolution')
  lid.add_argument('--lidar-vfov', type=float, nargs=2, default=(-22.5, 22.5),
                   metavar=('LOW', 'HIGH'), help='beam elevations (deg)')
  lid.add_argument('--lidar-rate', type=float, default=10.0)
  lid.add_argument('--lidar-range', type=float, default=10.0,
                   help='maximum range (m)')
  lid.add_argument('--lidar-noise', type=float, default=0.0,
                   help='range noise standard deviation (m)')
  ap.add_argument('--realtime-factor', type=float, default=1.0)
  ap.add_argument('--no-viewer', action='store_true',
                  help='do not start the web viewer')
  ap.add_argument('--viewer-port', type=int, default=8080)
  ap.add_argument('--duration', type=float, default=0.0,
                  help='stop after this many simulated seconds')
  ap.add_argument('--record', default='', metavar='FILE',
                  help='record a video that follows the robot (needs ffmpeg)')
  args = ap.parse_args(argv)

  logging.basicConfig(level=logging.INFO,
                      format='[%(asctime)s] [mujoco_sim] %(message)s',
                      datefmt='%H:%M:%S')
  logging.getLogger('OpenGL').setLevel(logging.WARNING)
  if args.fetch_lite3:
    from star_nav_mujoco import lite3  # pylint: disable=g-import-not-at-top
    lite3.fetch(log=logging.info)
    return 0
  if args.fetch_t1:
    from star_nav_mujoco import t1  # pylint: disable=g-import-not-at-top
    t1.fetch(log=logging.info)
    return 0
  if args.fetch_beni:
    from star_nav_mujoco import beni  # pylint: disable=g-import-not-at-top
    beni.fetch(log=logging.info)
    return 0
  for key, value in _DEFAULTS[args.robot].items():
    if getattr(args, key) is None:
      setattr(args, key, value)
  _select_gl()

  from star_nav_mujoco import lidar  # pylint: disable=g-import-not-at-top
  from star_nav_mujoco import sim  # pylint: disable=g-import-not-at-top
  from star_nav_mujoco import world  # pylint: disable=g-import-not-at-top

  cfg = sim.SimConfig(
      world=world.WorldConfig(size_x=args.map_size[0], size_y=args.map_size[1],
                              seed=args.seed, pillar_num=args.pillars,
                              ring_num=args.rings),
      start=tuple(args.start), robot=args.robot, house=args.world == 'house')
  if args.sensor != 'depth':
    cfg.lidar = lidar.LidarConfig(
        beams=args.lidar_beams, vfov=tuple(args.lidar_vfov),
        columns=args.lidar_columns, rate=args.lidar_rate,
        range_max=args.lidar_range, noise=args.lidar_noise)
  rc = sim.RunConfig(bind=args.bind, state_port=args.state_port,
                     cmd_port=args.cmd_port, depth_rate=args.depth_rate,
                     realtime_factor=args.realtime_factor,
                     viewer=not args.no_viewer, viewer_port=args.viewer_port,
                     duration=args.duration, record=args.record)
  from star_nav_mujoco import lite3  # pylint: disable=g-import-not-at-top
  try:
    simulator = sim.Simulator(cfg, render_depth=args.sensor != 'lidar')
  except lite3.MissingAssets as e:
    logging.error('%s', e)
    return 1
  logging.info('%s in a world of %d obstacles (seed %d), sensor %s, GL '
               'backend %s, MuJoCo %s', args.robot, len(simulator.obstacles),
               args.seed, args.sensor, os.environ['MUJOCO_GL'],
               __import__('mujoco').__version__)
  # roslaunch stops nodes with SIGINT, then SIGTERM; both should let the
  # recording finish cleanly.
  signal.signal(signal.SIGTERM, signal.default_int_handler)
  try:
    sim.run(simulator, rc)
  except KeyboardInterrupt:
    pass
  finally:
    simulator.close()
  return 0


if __name__ == '__main__':
  sys.exit(main())
