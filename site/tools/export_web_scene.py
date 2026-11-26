"""Writes the scenes of the browser demos.

``forest.xml``: the default forest and the quadrotor, the world of
exploration_mujoco.launch (seed 1, 50 m map), keeping only the obstacles
around the exploration area so the page stays light.

For each walking robot (``lite3``, ``t1``):

* ``<robot>.xml``: the house, with the robot attached from its upstream MJCF
  by file name. The page downloads that file, its meshes and the policy from
  the upstream repository (through jsDelivr, at the commits the simulator
  pins); they are not copied here.
* ``<robot>.json``: where those files are, and the numbers of the robot's
  controller (``robots/<robot>.py``, ``tracking.py``) that ``walk.js`` uses.

Run with the simulator's Python:

    mujoco_sim/.venv/bin/python site/tools/export_web_scene.py site/assets/mujoco
"""

import json
import pathlib
import sys

from star_nav_mujoco import house
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import tracking
from star_nav_mujoco import world
from star_nav_mujoco.robots import lite3
from star_nav_mujoco.robots import t1

HALF = 8.0  # m: keep obstacles within this box around the origin
START = (0.0, 0.0, 1.0)
CDN = 'https://cdn.jsdelivr.net/gh/{repo}@{commit}/{path}'


def forest() -> str:
  cfg = world.WorldConfig(seed=1, start=START[:2])
  obstacles = [ob for ob in world.generate(cfg)
               if abs(ob.x) <= HALF and abs(ob.y) <= HALF]
  return scene.build_xml(obstacles, quadrotor.Params(), START)


def walker_scene(module) -> str:
  """The house, and the robot's upstream MJCF attached as it is."""
  name = module.ROBOT.name
  xml = scene.build_xml(
      [], quadrotor.Params(), (0.0, 0.0, 0.0), module.TIMESTEP,
      quadrotor_body=False, extra=house.xml(module.HOUSE_WALL_HEIGHT))
  body = module.TRUNK[len(module.PREFIX):]
  model = f'    <model name="{name}" file="{name}/{module.MODEL_XML}"/>\n'
  attach = (f'    <attach model="{name}" body="{body}" '
            f'prefix="{module.PREFIX}"/>\n')
  xml = xml.replace('  <asset>\n', '  <asset>\n' + model, 1)
  head, tail = xml.rsplit('  </worldbody>', 1)
  return head + attach + '  </worldbody>' + tail


def tracker() -> dict:
  t = tracking.Holonomic()
  return {'k_pos': t.k_pos, 'k_yaw': t.k_yaw, 'max_speed': t.max_speed,
          'max_turn': t.max_turn, 'hold': t.hold, 'hold_yaw': t.hold_yaw}


def _floats(values) -> list:
  return [round(float(v), 6) for v in values]


def lite3_manifest() -> dict:
  m = lite3
  model = lambda path: CDN.format(repo=m.MODEL_REPO, commit=m.MODEL_COMMIT,
                                  path=f'{m.MODEL_DIR}/{path}')
  files = {f'lite3/{m.MODEL_XML}': model(m.MODEL_XML)}
  files.update({f'lite3/meshes/{name}': model(f'meshes/{name}')
                for name in m.MESHES})
  return {
      'robot': 'lite3',
      'source': f'{m.MODEL_REPO}, {m.POLICY_REPO} (BSD-3-Clause)',
      'files': files,
      'policy': {
          'url': CDN.format(repo=m.POLICY_REPO, commit=m.POLICY_COMMIT,
                            path=m.POLICY_PATH),
          'sha256': m.POLICY_SHA256, 'format': 'onnx', 'layers': 4},
      'trunk': m.TRUNK,
      'actuators': [m.PREFIX + j + '_ctrl' for j in m.JOINTS],
      'stand_height': m.STAND_HEIGHT, 'start_height': m.START_HEIGHT,
      'decimation': m.DECIMATION,
      'default_pose': _floats(m.DEFAULT_POSE),
      'action_scale': _floats(m.ACTION_SCALE),
      'kp': [m.KP] * len(m.JOINTS), 'kd': [m.KD] * len(m.JOINTS),
      'torque_limit': m.TORQUE_LIMIT, 'max_cmd': m.MAX_CMD,
      'omega_scale': m.OMEGA_SCALE, 'joint_vel_scale': m.JOINT_VEL_SCALE,
      # The upstream meshes are one grey: the simulator's colour per part.
      'appearance': {part[len('lite3_'):]: rgba
                     for part, (rgba, _, _) in m.APPEARANCE.items()},
      'tracker': tracker(),
  }


def t1_manifest() -> dict:
  m = t1
  url = lambda path: CDN.format(repo=m.REPO, commit=m.COMMIT, path=path)
  mesh_dir = str(pathlib.PurePosixPath(m.MODEL_XML).parent / 'meshes')
  files = {f't1/{m.MODEL_XML}': url(m.MODEL_XML)}
  files.update({f't1/{mesh_dir}/{name}.STL': url(f'{mesh_dir}/{name}.STL')
                for name in m.MESHES})
  return {
      'robot': 't1',
      'source': f'{m.REPO} (Apache-2.0)',
      'files': files,
      'policy': {'url': url(m.POLICY_PATH), 'sha256': m.POLICY_SHA256,
                 'format': 'torchscript', 'layers': [list(s) for s in m.LAYERS]},
      'trunk': m.TRUNK,
      'actuators': [m.PREFIX + j for j in m.JOINTS],
      'stand_height': m.STAND_HEIGHT, 'start_height': m.START_HEIGHT,
      'decimation': m.DECIMATION,
      'default_pose': _floats(m.DEFAULT_POSE),
      'kp': _floats(m.KP), 'kd': _floats(m.KD),
      'max_cmd': m.MAX_CMD, 'joint_vel_scale': m.JOINT_VEL_SCALE,
      'gait_frequency': m.GAIT_FREQUENCY,
      'tracker': tracker(),
  }


def main() -> None:
  out = pathlib.Path(sys.argv[1])
  (out / 'forest.xml').write_text(forest())
  for module, manifest in ((lite3, lite3_manifest()), (t1, t1_manifest())):
    name = manifest['robot']
    (out / f'{name}.xml').write_text(walker_scene(module))
    (out / f'{name}.json').write_text(json.dumps(manifest, indent=1) + '\n')
  print(f'forest.xml, lite3.xml, lite3.json, t1.xml, t1.json -> {out}')


if __name__ == '__main__':
  main()
