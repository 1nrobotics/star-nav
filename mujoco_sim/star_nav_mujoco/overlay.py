"""The planner's paths, drawn into the scene (browser viewer and ``--record``).

The bridge sends them as ``draw`` messages (protocol.py). The browser viewer
only shows what is in the model, so each layer is a fixed pool of visual-only
mocap bodies, moved to where the path is and parked below the ground when
not in use. Each body costs about 0.5 us per physics step and slows ray
casts (the lidar scans a copy of the scene without them: sim.py), so the
pools are small and only built when something shows them
(``SimConfig.draw``). They are in the scene's marker group: no contacts, and
the depth camera and the lidar do not see them.

scene.py imports this module, and listing the robots must not import MuJoCo
(robots/__init__.py), so MuJoCo is imported where it is used.
"""

from __future__ import annotations

import dataclasses

import numpy as np

# Layers (the ``layer`` field of a draw message).
PLAN = 0  # the path the planner follows now
ROUTE = 1  # a longer route the plan is taken from
CANDIDATES = 2  # the paths the planner chose from
WAYPOINTS = 3  # the mission's waypoints

_PARK = (0.0, 0.0, -1.0)


@dataclasses.dataclass(frozen=True)
class Layer:
  name: str
  count: int
  radius: float
  segment: float  # length of one piece of a line (m); 0: separate dots
  rgba: tuple[float, float, float, float]


LAYERS = {
    PLAN: Layer('plan', 60, 0.014, 0.15, (0.95, 0.15, 0.15, 1.0)),
    ROUTE: Layer('route', 60, 0.008, 0.25, (1.0, 0.8, 0.1, 1.0)),
    CANDIDATES: Layer('candidates', 400, 0.012, 0.0, (0.2, 0.9, 0.5, 0.7)),
    WAYPOINTS: Layer('waypoints', 40, 0.09, 0.0, (1.0, 0.25, 0.85, 0.6)),
}


def xml(group: int) -> str:
  """Worldbody XML of every layer's pool, in geom group ``group``."""
  out = []
  for layer in LAYERS.values():
    r, g, b, a = layer.rgba
    shape = (f'type="capsule" size="{layer.radius} {layer.segment / 2}"'
             if layer.segment else f'type="sphere" size="{layer.radius}"')
    for i in range(layer.count):
      out.append(
          f'<body name="draw/{layer.name}/{i}" mocap="true" '
          f'pos="{_PARK[0]} {_PARK[1]} {_PARK[2]}">'
          f'<geom {shape} rgba="{r} {g} {b} {a}" group="{group}" '
          f'contype="0" conaffinity="0"/></body>')
  return '\n    '.join(out)


def resample(points: np.ndarray, spacing: float, limit: int) -> np.ndarray:
  """Points every ``spacing`` along a polyline, at most ``limit`` + 1 of them.

  A polyline too long for that is sampled further apart instead.
  """
  points = np.asarray(points, float).reshape(-1, 3)
  if len(points) < 2:
    return points[:0]
  step = np.linalg.norm(np.diff(points, axis=0), axis=1)
  along = np.concatenate([[0.0], np.cumsum(step)])
  if along[-1] < spacing / 2:
    return points[:0]
  spacing = max(spacing, along[-1] / limit)
  at = np.arange(0.0, along[-1] + spacing / 2, spacing)[:limit + 1]
  return np.stack([np.interp(at, along, points[:, k]) for k in range(3)], axis=1)


class Overlay:
  """Moves the pools of a compiled model to the paths it is given."""

  def __init__(self, model):
    self._ids = {
        key: np.array([model.body_mocapid[model.body(f'draw/{layer.name}/{i}').id]
                       for i in range(layer.count)])
        for key, layer in LAYERS.items()}

  def set(self, data, layer: int, points: np.ndarray) -> bool:
    """Draws world-frame ``points`` (n, 3) on a layer; none clears it."""
    import mujoco  # pylint: disable=g-import-not-at-top
    spec = LAYERS.get(layer)
    points = np.asarray(points, float).reshape(-1, 3)
    if spec is None or not np.all(np.isfinite(points)):
      return False
    ids = self._ids[layer]
    data.mocap_pos[ids] = _PARK
    if spec.segment:
      pts = resample(points, spec.segment, spec.count)
      quat = np.empty(4)
      for i, (p, q) in zip(ids, zip(pts[:-1], pts[1:])):
        data.mocap_pos[i] = (p + q) / 2
        mujoco.mju_quatZ2Vec(quat, q - p)  # normalises its argument
        data.mocap_quat[i] = quat
    elif len(points):
      keep = np.linspace(0, len(points) - 1, min(len(points), spec.count))
      data.mocap_pos[ids[:len(keep)]] = points[keep.astype(int)]
    return True
