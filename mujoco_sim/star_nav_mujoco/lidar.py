"""360-degree multi-beam lidar, simulated with MuJoCo ray casting."""

from __future__ import annotations

import concurrent.futures
import dataclasses
import os

import mujoco
import numpy as np

from star_nav_mujoco import scene


@dataclasses.dataclass
class LidarConfig:
  """A spinning lidar with evenly spaced beams (defaults: 32 x 1024, 10 Hz)."""

  beams: int = 32
  vfov: tuple[float, float] = (-22.5, 22.5)  # elevation of lowest/highest beam (deg)
  columns: int = 1024  # azimuth steps per revolution
  rate: float = 10.0  # scans per second
  range_min: float = 0.3  # returns closer than this are dropped (m)
  range_max: float = 10.0  # no return beyond this (m)
  noise: float = 0.0  # range noise standard deviation (m)
  mount: tuple[float, float, float] = (0.0, 0.0, 0.06)  # sensor origin in body frame


@dataclasses.dataclass
class Scan:
  t: float
  pos: np.ndarray  # sensor origin in world
  quat: np.ndarray  # sensor orientation in world (w, x, y, z)
  points: np.ndarray  # (n, 3) float32 in the sensor frame (x forward, z up)


class Lidar:
  """Casts one full revolution instantly (no motion distortion).

  The sensor frame is aligned with the body frame (x forward, y left, z up),
  offset by ``mount``. Rays ignore the robot's own geoms.
  """

  def __init__(self, model: mujoco.MjModel, cfg: LidarConfig, body: int,
               threads: int | None = None, seed: int = 0):
    self.cfg = cfg
    self._model = model
    self._body = body
    elevation = np.deg2rad(np.linspace(cfg.vfov[0], cfg.vfov[1], cfg.beams))
    azimuth = np.linspace(0.0, 2.0 * np.pi, cfg.columns, endpoint=False)
    el, az = np.meshgrid(elevation, azimuth, indexing='ij')
    self.directions = np.stack(
        [np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)],
        axis=-1).reshape(-1, 3)
    self._mount = np.asarray(cfg.mount, dtype=float)
    # Only physical geoms reflect: skip visual markers (e.g. the setpoint)
    # and the robot's own legs.
    self._groups = np.ones(6, np.uint8)
    self._groups[scene.MARKER_GROUP] = 0
    self._groups[scene.ROBOT_GROUP] = 0
    self._rng = np.random.default_rng(seed)

    n = len(self.directions)
    threads = threads or min(4, os.cpu_count() or 1)
    self._chunks = np.array_split(np.arange(n), threads)
    self._geom = np.empty(n, np.int32)
    self._dist = np.empty(n)
    # mj_multiRay releases the GIL, so chunks run in parallel.
    self._pool = (concurrent.futures.ThreadPoolExecutor(threads)
                  if threads > 1 else None)

  def pose(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    rot = data.xmat[self._body].reshape(3, 3)
    pos = data.xpos[self._body] + rot @ self._mount
    return pos, data.xquat[self._body].copy()

  def scan(self, data: mujoco.MjData) -> Scan:
    rot = data.xmat[self._body].reshape(3, 3)
    pos, quat = self.pose(data)
    world_dirs = self.directions @ rot.T
    cutoff = self.cfg.range_max

    def cast(idx: np.ndarray) -> None:
      geom = np.empty(len(idx), np.int32)
      dist = np.empty(len(idx))
      mujoco.mj_multiRay(self._model, data, pos,
                         np.ascontiguousarray(world_dirs[idx]).ravel(), self._groups, 1,
                         self._body, geom, dist, None, len(idx), cutoff)
      self._geom[idx] = geom
      self._dist[idx] = dist

    if self._pool is None:
      cast(self._chunks[0])
    else:
      list(self._pool.map(cast, self._chunks))

    dist = self._dist.copy()
    if self.cfg.noise > 0:
      dist += self._rng.normal(0.0, self.cfg.noise, dist.shape)
    valid = ((self._geom >= 0) & (dist >= self.cfg.range_min) &
             (dist <= self.cfg.range_max))
    points = (self.directions[valid] * dist[valid, None]).astype(np.float32)
    return Scan(data.time, pos, quat, points)

  def close(self) -> None:
    if self._pool is not None:
      self._pool.shutdown()
      self._pool = None
