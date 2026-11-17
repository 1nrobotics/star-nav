"""Writes the MJCF of the browser demo: the default forest and quadrotor.

Same world as exploration_mujoco.launch (seed 1, 50 m map), keeping only the
obstacles around the exploration area so the page stays light. Run with the
simulator's Python:

    mujoco_sim/.venv/bin/python site/tools/export_web_scene.py site/assets/mujoco/forest.xml
"""

import dataclasses
import sys

from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import world

HALF = 8.0  # m: keep obstacles within this box around the origin
START = (0.0, 0.0, 1.0)


def main() -> None:
  cfg = world.WorldConfig(seed=1, start=START[:2])
  obstacles = [ob for ob in world.generate(cfg)
               if abs(ob.x) <= HALF and abs(ob.y) <= HALF]
  xml = scene.build_xml(obstacles, quadrotor.Params(), START)
  with open(sys.argv[1], 'w') as f:
    f.write(xml)
  print(f'{len(obstacles)} obstacles -> {sys.argv[1]}')


if __name__ == '__main__':
  main()
