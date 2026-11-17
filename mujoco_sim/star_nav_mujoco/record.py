"""Records the flight to a video file through ffmpeg (``--record``)."""

from __future__ import annotations

import shutil
import subprocess

import mujoco
import numpy as np


class Recorder:
  """Renders a camera that follows the quadrotor and pipes frames to ffmpeg.

  Frames are taken at ``fps`` in simulated time, so the video plays at the
  simulated speed regardless of ``--realtime-factor``.
  """

  def __init__(self, model: mujoco.MjModel, path: str, width: int = 1280,
               height: int = 720, fps: float = 30.0):
    ffmpeg = shutil.which('ffmpeg')
    if ffmpeg is None:
      raise RuntimeError('--record needs ffmpeg on PATH')
    self.period = 1.0 / fps
    self.next_time = 0.0
    self.frames = 0
    self._renderer = mujoco.Renderer(model, height, width)
    self._camera = mujoco.MjvCamera()
    self._camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    self._camera.trackbodyid = model.body('quad').id
    self._camera.distance = 5.0
    self._camera.azimuth = -135.0
    self._camera.elevation = -25.0
    self._ffmpeg = subprocess.Popen(
        [ffmpeg, '-loglevel', 'error', '-y', '-f', 'rawvideo',
         '-pix_fmt', 'rgb24', '-s', f'{width}x{height}', '-r', f'{fps:g}',
         '-i', '-', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
         '-pix_fmt', 'yuv420p', '-movflags', '+faststart', path],
        stdin=subprocess.PIPE)

  def maybe_capture(self, data: mujoco.MjData) -> None:
    if data.time < self.next_time:
      return
    self._renderer.update_scene(data, camera=self._camera)
    frame = self._renderer.render()
    self._ffmpeg.stdin.write(np.ascontiguousarray(frame).tobytes())
    self.frames += 1
    self.next_time = max(self.next_time + self.period, data.time)

  def close(self) -> None:
    if self._ffmpeg.stdin and not self._ffmpeg.stdin.closed:
      self._ffmpeg.stdin.close()
    self._ffmpeg.wait()
    self._renderer.close()
