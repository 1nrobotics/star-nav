"""Procedural forest world, equivalent to map_generator/random_forest_sensing.

Obstacles are kept as primitive descriptions so the same world can be built
into a MuJoCo scene and sent to the ROS bridge (which rasterises it into the
global point cloud shown in RViz).
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

PILLAR = 0
RING = 1


@dataclasses.dataclass(frozen=True)
class Obstacle:
  kind: int  # PILLAR or RING
  x: float
  y: float
  z: float  # pillar: 0, ring: centre height
  a: float  # pillar: radius, ring: horizontal semi-axis
  b: float  # pillar: height, ring: vertical semi-axis
  yaw: float  # ring orientation about z (pillar: 0)


@dataclasses.dataclass(frozen=True)
class WorldConfig:
  size_x: float = 50.0
  size_y: float = 50.0
  seed: int = 1
  pillar_num: int = 100
  pillar_width: tuple[float, float] = (0.5, 0.7)
  pillar_height: tuple[float, float] = (0.0, 3.0)
  ring_num: int = 50
  ring_radius: tuple[float, float] = (0.5, 0.7)
  ring_z: tuple[float, float] = (0.7, 0.8)
  ring_theta: float = 0.5
  min_distance: float = 0.8  # between pillar centres
  start: tuple[float, float] = (0.0, 0.0)
  start_clearance: float = 1.5  # keep the spawn point free


# Forest for the Microduck: pillars 5-30 cm wide (the drawn width is scaled by
# 0.5-1.5) and taller than its camera, at least 0.9 m apart.
MICRODUCK_FOREST = dict(pillar_width=(0.1, 0.2), pillar_height=(0.6, 1.2),
                        min_distance=0.9, start_clearance=0.6)


def generate(cfg: WorldConfig) -> list[Obstacle]:
  """Returns a deterministic list of obstacles for ``cfg``."""
  rng = np.random.default_rng(cfg.seed)
  hx, hy = cfg.size_x / 2.0, cfg.size_y / 2.0
  start = np.array(cfg.start)
  obstacles: list[Obstacle] = []
  centres: list[np.ndarray] = []

  while len(centres) < cfg.pillar_num:
    p = rng.uniform((-hx, -hy), (hx, hy))
    width = rng.uniform(*cfg.pillar_width) * rng.uniform(0.5, 1.5)
    height = max(rng.uniform(*cfg.pillar_height), 0.3)
    if any(np.linalg.norm(p - c) < cfg.min_distance for c in centres):
      continue
    if np.linalg.norm(p - start) < cfg.start_clearance + width / 2:
      continue
    centres.append(p)
    obstacles.append(Obstacle(PILLAR, p[0], p[1], 0.0, width / 2, height, 0.0))

  rings = 0
  while rings < cfg.ring_num:
    p = rng.uniform((-hx, -hy), (hx, hy))
    z = rng.uniform(*cfg.ring_z)
    yaw = rng.uniform(-cfg.ring_theta, cfg.ring_theta)
    lo, hi = sorted(cfg.ring_radius)
    a = rng.uniform(lo, hi)
    b = rng.uniform(lo, 1.2)
    if np.linalg.norm(p - start) < cfg.start_clearance + a:
      continue
    rings += 1
    obstacles.append(Obstacle(RING, p[0], p[1], z, a, b, yaw))
  return obstacles


def ring_points(ob: Obstacle, segments: int) -> np.ndarray:
  """Returns ``segments + 1`` points on a ring (closed polyline)."""
  t = np.linspace(0.0, 2.0 * math.pi, segments + 1)
  local = np.stack([np.zeros_like(t), ob.a * np.cos(t), ob.b * np.sin(t)], 1)
  c, s = math.cos(ob.yaw), math.sin(ob.yaw)
  rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
  return local @ rot.T + np.array([ob.x, ob.y, ob.z])
