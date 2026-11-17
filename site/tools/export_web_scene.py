"""Writes the MJCF of the browser demo for one robot.

quadrotor: the same world as exploration_mujoco.launch (seed 1, 50 m map),
keeping only the obstacles around the exploration area so the page stays
light.

microduck: the same world as exploration_microduck.launch (seed 1, 12 m map)
with the duck attached as the simulator does it. Its meshes are referenced by
file name only: the page downloads them from the pinned upstream commit
(CC BY-NC-SA 4.0, not redistributed here). The BAM armature, which the servo
model sets at run time in Python, is written into the file.

Needs the simulator's Python (and, for the duck, the downloaded model):

    mujoco_sim/.venv/bin/python site/tools/export_web_scene.py quadrotor site/assets/mujoco/forest.xml
    mujoco_sim/.venv/bin/python site/tools/export_web_scene.py microduck site/assets/mujoco/microduck.xml
"""

import dataclasses
import os
import sys

import mujoco

from star_nav_mujoco import microduck
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import world

HALF = 8.0  # m: keep the drone's obstacles within this box around the origin


def quadrotor_xml() -> tuple[str, int]:
  start = (0.0, 0.0, 1.0)
  cfg = world.WorldConfig(seed=1, start=start[:2])
  obstacles = [ob for ob in world.generate(cfg)
               if abs(ob.x) <= HALF and abs(ob.y) <= HALF]
  return scene.build_xml(obstacles, quadrotor.Params(), start), len(obstacles)


def microduck_xml() -> tuple[str, int]:
  from bam.model import load_model  # pylint: disable=g-import-not-at-top

  start = (0.0, 0.0, microduck.STAND_HEIGHT)
  cfg = dataclasses.replace(
      world.WorldConfig(size_x=12.0, size_y=12.0, seed=1, pillar_num=60,
                        ring_num=0, start=start[:2]),
      **world.MICRODUCK_FOREST)
  obstacles = world.generate(cfg)
  xml = scene.build_xml(obstacles, quadrotor.Params(), start,
                        microduck.TIMESTEP, quadrotor_body=False,
                        marker_radius=0.025)
  spec = mujoco.MjSpec.from_string(xml)
  microduck.attach(spec, start[:2], microduck.check_assets())
  spec.meshdir = ''
  for mesh in spec.meshes:
    mesh.file = os.path.basename(mesh.file)
  armature = load_model(motor_name='xl330', model='m6').armature.value
  for name in microduck.JOINTS:
    spec.joint(microduck.PREFIX + name).armature = armature
  spec.compile()  # check
  return spec.to_xml(), len(obstacles)


def main() -> None:
  robot, out = sys.argv[1:3]
  xml, n = {'quadrotor': quadrotor_xml, 'microduck': microduck_xml}[robot]()
  with open(out, 'w') as f:
    f.write(xml)
  print(f'{robot}: {n} obstacles -> {out}')


if __name__ == '__main__':
  main()
