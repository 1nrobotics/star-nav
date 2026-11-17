"""Records the flight to a video file through ffmpeg (``--record``)."""

from __future__ import annotations

import shutil
import subprocess

import mujoco
import numpy as np

from star_nav_mujoco import scene


# Lidar returns drawn into the video, at most this many per frame.
_MAX_POINTS = 4000


class Recorder:
  """Renders a camera that follows the robot and pipes frames to ffmpeg.

  Frames are taken at ``fps`` in simulated time, so the video plays at the
  simulated speed regardless of ``--realtime-factor``. Lidar returns passed to
  ``maybe_capture`` are drawn as small dots.
  """

  def __init__(self, model: mujoco.MjModel, path: str, width: int = 1280,
               height: int = 720, fps: float = 30.0):
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg is None:
      raise RuntimeError('--record needs ffmpeg on PATH')
    self.period = 1.0 / fps
    self.next_time = 0.0
    self.frames = 0
    self._renderer = mujoco.Renderer(model, height, width,
                                     max_geom=model.ngeom + _MAX_POINTS + 1000)
    self._camera = mujoco.MjvCamera()
    self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    self._camera.trackbodyid = scene.robot_body(model)
    self._camera.distance = 5.0
    self._camera.azimuth = -135.0
    self._camera.elevation = -25.0
    self._ffmpeg = subprocess.Popen(
        [ffmpeg, '-loglevel', 'error', '-y', '-f', 'rawvideo',
         '-pix_fmt', 'rgb24', '-s', f'{width}x{height}', '-r', f'{fps:g}',
         '-i', '-', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
         '-pix_fmt', 'yuv420p', '-movflags', '+faststart', path],
        stdin=subprocess.PIPE)

  def maybe_capture(self, data: mujoco.MjData,
                    points: np.ndarray | None = None) -> None:
    if data.time < self.next_time:
      return
    self._renderer.update_scene(data, camera=self._camera)
    if points is not None and len(points):
      self._draw_points(points)
    frame = self._renderer.render()
    self._ffmpeg.stdin.write(np.ascontiguousarray(frame).tobytes())
    self.frames += 1
    self.next_time = max(self.next_time + self.period, data.time)

  def _draw_points(self, points: np.ndarray) -> None:
    scene = self._renderer.scene
    step = max(1, len(points) // _MAX_POINTS)
    pts = points[::step]
    # Colour by height: blue near the ground to red at 2.5 m.
    h = np.clip(pts[:, 2] / 2.5, 0.0, 1.0)
    size = np.array([0.025, 0.0, 0.0])
    eye = np.eye(3).flatten()
    for p, k in zip(pts, h):
      if scene.ngeom >= scene.maxgeom:
        break
      mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                          size, p.astype(float), eye,
                          np.array([k, 0.35, 1.0 - k, 1.0], np.float32))
      scene.ngeom += 1

  def close(self) -> None:
    if self._ffmpeg.stdin and not self._ffmpeg.stdin.closed:
      self._ffmpeg.stdin.close()
    self._ffmpeg.wait()
    self._renderer.close()
