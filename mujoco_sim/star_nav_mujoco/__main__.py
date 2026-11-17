"""Command-line entry point: ``python -m star_nav_mujoco``."""

from __future__ import annotations

import argparse
import dataclasses
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


# World defaults per robot: the 25 cm Microduck gets a smaller forest of thin,
# tall pillars (no rings, which it would walk under or into).
_DEFAULTS = {
    'quadrotor': dict(map_size=(50.0, 50.0), pillars=100, rings=50,
                      start=(0.0, 0.0, 1.0)),
    'microduck': dict(map_size=(12.0, 12.0), pillars=60, rings=0,
                      start=(0.0, 0.0, 0.125)),
}


def main(argv: list[str] | None = None) -> int:
  ap = argparse.ArgumentParser(
      prog='star_nav_mujoco',
      description='MuJoCo simulator for Star Nav (quadrotor or Microduck '
      'biped). Talks to the ROS bridge (mujoco_bridge) over ZeroMQ and serves '
      'a web viewer.')
  ap.add_argument('--robot', choices=('quadrotor', 'microduck'),
                  default='quadrotor')
  ap.add_argument('--fetch-microduck', action='store_true',
                  help='download the Microduck model and walking policy, '
                  'then exit')
  ap.add_argument('--seed', type=int, default=1, help='forest seed')
  ap.add_argument('--map-size', type=float, nargs=2, default=None,
                  metavar=('X', 'Y'), help='default: 50 50 (quadrotor), '
                  '12 12 (microduck)')
  ap.add_argument('--pillars', type=int, default=None,
                  help='default: 100 (quadrotor), 60 (microduck)')
  ap.add_argument('--rings', type=int, default=None,
                  help='default: 50 (quadrotor), 0 (microduck)')
  ap.add_argument('--start', type=float, nargs=3, default=None,
                  metavar=('X', 'Y', 'Z'), help='default: 0 0 1 (the duck '
                  'ignores Z)')
  ap.add_argument('--bind', default='127.0.0.1',
                  help='address for the ZeroMQ sockets (use 0.0.0.0 to '
                  'accept a bridge on another host)')
  ap.add_argument('--state-port', type=int, default=5555)
  ap.add_argument('--cmd-port', type=int, default=5556)
  ap.add_argument('--depth-rate', type=float, default=30.0)
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
  if args.fetch_microduck:
    from star_nav_mujoco import microduck  # pylint: disable=g-import-not-at-top
    microduck.fetch(log=logging.info)
    return 0
  for key, value in _DEFAULTS[args.robot].items():
    if getattr(args, key) is None:
      setattr(args, key, value)
  _select_gl()

  from star_nav_mujoco import microduck  # pylint: disable=g-import-not-at-top
  from star_nav_mujoco import sim  # pylint: disable=g-import-not-at-top
  from star_nav_mujoco import world  # pylint: disable=g-import-not-at-top

  wcfg = world.WorldConfig(size_x=args.map_size[0], size_y=args.map_size[1],
                           seed=args.seed, pillar_num=args.pillars,
                           ring_num=args.rings)
  if args.robot == 'microduck':
    wcfg = dataclasses.replace(wcfg, **world.MICRODUCK_FOREST)
  cfg = sim.SimConfig(world=wcfg, start=tuple(args.start), robot=args.robot)
  rc = sim.RunConfig(bind=args.bind, state_port=args.state_port,
                     cmd_port=args.cmd_port, depth_rate=args.depth_rate,
                     realtime_factor=args.realtime_factor,
                     viewer=not args.no_viewer, viewer_port=args.viewer_port,
                     duration=args.duration, record=args.record)
  try:
    simulator = sim.Simulator(cfg)
  except microduck.MissingAssets as e:
    logging.error('%s', e)
    return 1
  logging.info('%s in a world of %d obstacles (seed %d), GL backend %s, '
               'MuJoCo %s', args.robot, len(simulator.obstacles), args.seed,
               os.environ['MUJOCO_GL'], __import__('mujoco').__version__)
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
