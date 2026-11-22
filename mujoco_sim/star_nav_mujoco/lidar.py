"""360-degree lidar, simulated with MuJoCo ray casting.

Two scan patterns: ``uniform``, a spinning lidar with evenly spaced beams, and
``mid360``, the non-repetitive pattern of Livox's Mid-360. The Mid-360 beam
table is Livox's (github.com/Livox-SDK/livox_laser_simulation, MIT); it is
downloaded once (``star-nav-mujoco --fetch-mid360``) and is not part of this
repository.
"""

from __future__ import annotations

import concurrent.futures
import dataclasses
import hashlib
import os
import pathlib
import urllib.request

import mujoco
import numpy as np

from star_nav_mujoco import scene


UNIFORM, MID360 = 'uniform', 'mid360'
PATTERNS = (UNIFORM, MID360)

# Livox's beam table for the Mid-360: 800,000 directions (azimuth, zenith),
# 4 s at 200,000 points per second; elevations -7.2 to 52.2 degrees.
MID360_REPO = 'Livox-SDK/livox_laser_simulation'
MID360_COMMIT = '1cce1073633a062b92e30243a4c2920e45551bb5'
MID360_PATH = 'scan_mode/mid360.csv'
MID360_SHA256 = 'aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a'
MID360_POINT_RATE = 200000  # points per second


class MissingPattern(RuntimeError):
  """The Mid-360 beam table has not been downloaded."""


def pattern_dir() -> pathlib.Path:
  env = os.environ.get('STAR_NAV_LIDAR_DIR')
  return pathlib.Path(env) if env else pathlib.Path.home() / '.cache/star_nav/lidar'


def fetch_mid360(root: pathlib.Path | None = None, log=print) -> pathlib.Path:
  """Downloads the pinned Mid-360 beam table (17 MB) into ``root``."""
  root = root or pattern_dir()
  dest = root / 'mid360.csv'
  log(f'Downloading {MID360_PATH} ({MID360_REPO}@{MID360_COMMIT[:7]}) to {root}')
  root.mkdir(parents=True, exist_ok=True)
  tmp = dest.with_suffix('.csv.part')
  url = f'https://raw.githubusercontent.com/{MID360_REPO}/{MID360_COMMIT}/{MID360_PATH}'
  digest = hashlib.sha256()
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, 'wb') as f:
    while chunk := r.read(1 << 20):
      digest.update(chunk)
      f.write(chunk)
  if digest.hexdigest() != MID360_SHA256:
    tmp.unlink()
    raise RuntimeError(f'mid360.csv: unexpected sha256 {digest.hexdigest()}')
  tmp.replace(dest)
  (root / 'SOURCE.md').write_text(
      'Downloaded by star_nav_mujoco.lidar.fetch_mid360; not redistributed.\n\n'
      f'- mid360.csv: https://github.com/{MID360_REPO}/blob/{MID360_COMMIT}/'
      f'{MID360_PATH} (MIT, Livox)\n')
  return dest


def _directions(azimuth: np.ndarray, elevation: np.ndarray) -> np.ndarray:
  return np.stack([np.cos(elevation) * np.cos(azimuth),
                   np.cos(elevation) * np.sin(azimuth), np.sin(elevation)],
                  axis=-1)


def mid360_directions(root: pathlib.Path | None = None) -> np.ndarray:
  """The Mid-360 beam table as unit vectors, in firing order."""
  path = (root or pattern_dir()) / 'mid360.csv'
  if not path.exists():
    raise MissingPattern(
        f'Mid-360 beam table not found in {path.parent}. Download it once '
        'with "star-nav-mujoco --fetch-mid360".')
  table = np.loadtxt(path, delimiter=',', skiprows=1, usecols=(1, 2))
  return _directions(np.deg2rad(table[:, 0]), np.deg2rad(90.0 - table[:, 1]))


@dataclasses.dataclass
class LidarConfig:
  """A 360-degree lidar (defaults: spinning, 32 x 1024 beams, 10 Hz)."""

  pattern: str = UNIFORM  # UNIFORM or MID360
  # UNIFORM only; the Mid-360 has its own beam table.
  beams: int = 32
  vfov: tuple[float, float] = (-22.5, 22.5)  # elevation of lowest/highest beam (deg)
  columns: int = 1024  # azimuth steps per revolution
  rate: float = 10.0  # scans per second
  range_min: float = 0.3  # returns closer than this are dropped (m)
  range_max: float = 10.0  # no return beyond this (m)
  noise: float = 0.0  # range noise standard deviation (m)
  mount: tuple[float, float, float] = (0.0, 0.0, 0.06)  # sensor origin in body frame
  pitch: float = 0.0  # sensor tilted nose-down by this angle (deg)


@dataclasses.dataclass
class Scan:
  t: float
  pos: np.ndarray  # sensor origin in world
  quat: np.ndarray  # sensor orientation in world (w, x, y, z)
  rot: np.ndarray  # the same as a rotation matrix
  points: np.ndarray  # (n, 3) float32 in the sensor frame (x forward, z up)


class Lidar:
  """Casts one scan instantly (no motion distortion).

  A scan is one revolution (``uniform``) or the next 1 / ``rate`` seconds of
  the beam table (``mid360``), so successive Mid-360 scans differ. The sensor
  frame is the body frame (x forward, y left, z up) offset by ``mount`` and
  tilted nose-down by ``pitch``. Rays ignore the robot's own geoms.
  """

  def __init__(self, model: mujoco.MjModel, cfg: LidarConfig, body: int,
               threads: int | None = None, seed: int = 0):
    self.cfg = cfg
    self._model = model
    self._body = body
    if cfg.pattern == MID360:
      self._table = mid360_directions()
      n = round(MID360_POINT_RATE / cfg.rate)
    elif cfg.pattern == UNIFORM:
      elevation = np.deg2rad(np.linspace(cfg.vfov[0], cfg.vfov[1], cfg.beams))
      azimuth = np.linspace(0.0, 2.0 * np.pi, cfg.columns, endpoint=False)
      el, az = np.meshgrid(elevation, azimuth, indexing='ij')
      self._table = _directions(az, el).reshape(-1, 3)
      n = len(self._table)
    else:
      raise ValueError(f'unknown lidar pattern {cfg.pattern!r}')
    self._next = 0  # first beam of the next scan in the table
    self.directions = self._table[:n]  # of the last scan, sensor frame
    self._mount = np.asarray(cfg.mount, dtype=float)
    pitch = np.deg2rad(cfg.pitch)
    self._tilt = np.array([[np.cos(pitch), 0.0, np.sin(pitch)],
                           [0.0, 1.0, 0.0],
                           [-np.sin(pitch), 0.0, np.cos(pitch)]])
    self._tilt_quat = np.array([np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0])
    # Only physical geoms reflect: skip visual markers (e.g. the setpoint)
    # and the robot's own legs.
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
    return Scan(data.time, pos, quat, rot, points)

  def close(self) -> None:
    if self._pool is not None:
      self._pool.shutdown()
      self._pool = None
