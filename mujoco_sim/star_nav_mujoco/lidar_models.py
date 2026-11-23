"""The lidars the simulator can imitate: one ``LidarModel`` each.

A model is what tells one sensor from another: its beam directions, how many
of them make a scan, and its range. ``lidar.Lidar`` casts whatever table of
directions the model gives, so adding a sensor means adding a model here.

Where the sensor sits on a robot is the robot's business
(``robots.Robot.lidar_mount`` and ``lidar_pitch``).

This module does not import MuJoCo: the command line lists the models before
it has chosen an OpenGL backend (``__main__._select_gl``).
"""

from __future__ import annotations

import numpy as np

from star_nav_mujoco import assets

DEFAULT = 'uniform'


def directions(azimuth: np.ndarray, elevation: np.ndarray) -> np.ndarray:
  """Unit vectors in the sensor frame (x forward, y left, z up), from rad."""
  return np.stack([np.cos(elevation) * np.cos(azimuth),
                   np.cos(elevation) * np.sin(azimuth), np.sin(elevation)],
                  axis=-1)


class LidarModel:
  """What the simulator needs to know about a lidar."""

  name = ''  # --lidar-model value
  title = ''  # as written in messages
  # A repetitive scan casts the same beams every time; a non-repetitive one
  # walks through its table, so successive scans differ.
  repetitive = True
  range_min = 0.3  # returns closer than this are dropped (m)
  range_max = 10.0  # no return beyond this (m)

  def check_assets(self) -> None:
    """Raises ``assets.MissingAssets`` if the model's files are not there."""

  # Downloads the model's files: fetch(log=print). None: nothing to download.
  fetch = None

  def table(self, cfg) -> np.ndarray:
    """Beam directions (n, 3) in firing order, for a ``lidar.LidarConfig``."""
    raise NotImplementedError

  def scan_size(self, cfg, table: np.ndarray) -> int:
    """Beams in one scan."""
    return len(table)

  def lines(self, cfg) -> int:
    """Laser lines, as sent with a scan."""
    raise NotImplementedError


class Uniform(LidarModel):
  """An ideal spinning lidar: ``beams`` x ``columns``, evenly spaced."""

  name = 'uniform'
  title = 'spinning lidar with evenly spaced beams'

  def table(self, cfg) -> np.ndarray:
    elevation = np.deg2rad(np.linspace(cfg.vfov[0], cfg.vfov[1], cfg.beams))
    azimuth = np.linspace(0.0, 2.0 * np.pi, cfg.columns, endpoint=False)
    el, az = np.meshgrid(elevation, azimuth, indexing='ij')
    return directions(az, el).reshape(-1, 3)

  def lines(self, cfg) -> int:
    return cfg.beams


class Mid360(LidarModel):
  """Livox Mid-360: 200,000 points per second, never the same scan twice.

  The beams are Livox's own table for their Gazebo plugin (MIT): 800,000
  directions, 4 s of scanning, elevations -7.2 to 52.2 deg. It is downloaded,
  not committed. Range from the datasheet (40 m is for 10 % reflectivity).
  """

  name = 'mid360'
  title = 'Livox Mid-360'
  repetitive = False
  range_min = 0.1
  range_max = 40.0
  point_rate = 200000  # points per second

  REPO = 'Livox-SDK/livox_laser_simulation'
  COMMIT = '1cce1073633a062b92e30243a4c2920e45551bb5'
  PATH = 'scan_mode/mid360.csv'  # time (s), azimuth (deg), zenith (deg)
  SHA256 = 'aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a'

  def _file(self):
    return assets.directory('lidar') / 'mid360.csv'

  def check_assets(self) -> None:
    path = self._file()
    if not path.exists():
      raise assets.MissingAssets(
          f'Mid-360 beam table not found in {path.parent}. Download it once '
          'with "star-nav-mujoco --fetch mid360".')

  def fetch(self, log=print) -> None:
    dest = self._file()
    log(f'Downloading {self.PATH} ({self.REPO}@{self.COMMIT[:7]}, 17 MB) to '
        f'{dest.parent}')
    assets.download(f'{assets.github(self.REPO, self.COMMIT)}/{self.PATH}',
                    dest, self.SHA256)
    (dest.parent / 'SOURCE.md').write_text(
        'Downloaded by star_nav_mujoco.lidar_models; not redistributed.\n\n'
        f'- mid360.csv: https://github.com/{self.REPO}/blob/{self.COMMIT}/'
        f'{self.PATH} (MIT, Livox)\n')

  def table(self, cfg) -> np.ndarray:
    self.check_assets()
    rows = np.loadtxt(self._file(), delimiter=',', skiprows=1, usecols=(1, 2))
    return directions(np.deg2rad(rows[:, 0]), np.deg2rad(90.0 - rows[:, 1]))

  def scan_size(self, cfg, table: np.ndarray) -> int:
    return round(self.point_rate / cfg.rate)

  def lines(self, cfg) -> int:
    return 4


_MODELS = {m.name: m for m in (Uniform(), Mid360())}


def names() -> tuple[str, ...]:
  """The models there are, the default first."""
  return (DEFAULT, *(n for n in _MODELS if n != DEFAULT))


def get(name: str) -> LidarModel:
  if name not in _MODELS:
    raise ValueError(f'unknown lidar model {name!r}')
  return _MODELS[name]
