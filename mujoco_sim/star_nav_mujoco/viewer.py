"""Browser viewer (MuJoCo Studio web viewer) for the simulator.

Imported only when the viewer is enabled, so headless runs do not load Studio.
This module must not use ``from __future__ import annotations``: Studio
resolves handler annotations at decoration time.
"""

import contextlib
import logging

import mujoco
from mujoco.studio import launch_web
from mujoco.studio import messages
from mujoco.studio import plugin
from mujoco.studio import viewer_handle
from mujoco.studio import viewer_protocol

from star_nav_mujoco import robots

log = logging.getLogger('star_nav_mujoco')


class FollowQuadrotor(plugin.ViewerPlugin):
  """Starts the browser camera orbiting around the robot."""

  @messages.handler
  def on_post_model(self, event: messages.PostModelEvent) -> None:
    cam = self.viewer.camera
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    robot = robots.find(event.model)
    cam.trackbodyid = event.model.body(robot.body).id
    cam.distance, cam.elevation = robot.follow_camera
    cam.azimuth = -135.0


def launch(port: int, stack: contextlib.ExitStack) -> viewer_handle.ViewerHandle:
  """Starts the web viewer on ``port``; it closes when ``stack`` exits.

  Note: Studio binds the viewer to all network interfaces.
  """
  config = viewer_protocol.ViewerConfig(
      title='Star Nav MuJoCo', gfx='web', http_port=port)
  handle = stack.enter_context(
      launch_web.launch(config, plugins=[FollowQuadrotor()]))
  log.info('Web viewer: http://localhost:%d', port)
  return handle
