"""360-degree lidar, simulated with MuJoCo ray casting.

Which sensor it imitates is a ``lidar_models.LidarModel`` (an ideal spinning
lidar, Livox's Mid-360): the model gives the beam directions and the range,
this module casts them.
"""

from __future__ import annotations

import concurrent.futures
import dataclasses
import os

import mujoco
import numpy as np

from star_nav_mujoco import lidar_models
from star_nav_mujoco import scene


@dataclasses.dataclass
class LidarConfig:
  """A 360-degree lidar (defaults: spinning, 32 x 1024 beams, 10 Hz)."""

  model: str = lidar_models.DEFAULT  # a name in lidar_models
  # The uniform model only; other models have their own beams.
  beams: int = 32
  vfov: tuple[float, float] = (-22.5, 22.5)  # elevation of lowest/highest beam (deg)
  columns: int = 1024  # azimuth steps per revolution
  rate: float = 10.0  # scans per second
  # None: the model's.
  range_min: float | None = None  # returns closer than this are dropped (m)
  range_max: float | None = None  # no return beyond this (m)
  noise: float = 0.0  # range noise standard deviation (m)
  mount: tuple[float, float, float] = (0.0, 0.0, 0.06)  # sensor origin in body frame
  # Sensor tilted nose-down by this angle (deg). None: the robot's for this
  # model (robots.Robot.lidar_pitch), level without a robot.
  pitch: float | None = None

  def resolved(self) -> LidarConfig:
    """This configuration with the model's values where it has None."""
    model = lidar_models.get(self.model)
    return dataclasses.replace(
        self,
        range_min=model.range_min if self.range_min is None else self.range_min,
        range_max=model.range_max if self.range_max is None else self.range_max,
        pitch=self.pitch or 0.0)


@dataclasses.dataclass
class Scan:
  t: float
  pos: np.ndarray  # sensor origin in world
  quat: np.ndarray  # sensor orientation in world (w, x, y, z)
  rot: np.ndarray  # the same as a rotation matrix
  points: np.ndarray  # (n, 3) float32 in the sensor frame (x forward, z up)


class Lidar:
  """Casts one scan instantly (no motion distortion).

  A scan is the model's whole beam table (a repetitive scan, one revolution)
  or the next 1 / ``rate`` seconds of it (a non-repetitive scan, so
  successive scans differ). The sensor
  frame is the body frame (x forward, y left, z up) offset by ``mount`` and
  tilted nose-down by ``pitch``. Rays ignore the robot's own geoms.
  """

  def __init__(self, model: mujoco.MjModel, cfg: LidarConfig, body: int,
               threads: int | None = None, seed: int = 0):
    self.cfg = cfg = cfg.resolved()
    self.sensor = lidar_models.get(cfg.model)
    self.lines = self.sensor.lines(cfg)
    self.model = model
    self._body = body
    self._table = self.sensor.table(cfg)
    n = self.sensor.scan_size(cfg, self._table)
    self._next = 0  # first beam of the next scan in the table
    self.directions = self._table[:n]  # of the last scan, sensor frame
    self._mount = np.asarray(cfg.mount, dtype=float)
    pitch = np.deg2rad(cfg.pitch)
    self._tilt = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                           [0.0, 1.0, 0.0],
                           [-np.sin(pitch), 0.0, np.cos(pitch)]])
    self._tilt_quat = np.array([np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0])
    # Only physical geoms reflect: skip visual markers (e.g. a robot's visual
    # meshes) and the robot's own legs.
    self._groups = np.ones(6, np.uint8)
    self._groups[scene.MARKER_GROUP] = 0
    self._groups[scene.ROBOT_GROUP] = 0
    self._rng = np.random.default_rng(seed)

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
    quat = np.empty(4)
    mujoco.mju_mulQuat(quat, data.xquat[self._body], self._tilt_quat)
    return pos, quat

  def scan(self, data: mujoco.MjData) -> Scan:
    rot = data.xmat[self._body].reshape(3, 3) @ self._tilt
    pos, quat = self.pose(data)
    n = len(self._geom)
    self.directions = self._table.take(np.arange(self._next, self._next + n),
                                       axis=0, mode='wrap')
    self._next = (self._next + n) % len(self._table)
    world_dirs = self.directions @ rot.T
    cutoff = self.cfg.range_max

    def cast(idx: np.ndarray) -> None:
      geom = np.empty(len(idx), np.int32)
      dist = np.empty(len(idx))
      mujoco.mj_multiRay(self.model, data, pos,
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
    return Scan(data.time, pos, quat, rot, points)

  def close(self) -> None:
    if self._pool is not None:
      self._pool.shutdown()
      self._pool = None
