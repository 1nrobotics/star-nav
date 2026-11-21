"""Trackers: from the planner's setpoint to a body-frame velocity command.

A ground robot's locomotion policy follows a velocity (forward, sideways,
turn rate). A tracker gives it that velocity from the position setpoint and
yaw the planner sends, and the robot's pose. Each robot names its tracker
(``Robot.tracker``): ``Holonomic`` for one that walks in any direction.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np


@dataclasses.dataclass
class Holonomic:
  """Follows the setpoint in any direction and turns to the planner's yaw.

  Measured with the Lite3 on flat ground (policy at 83 Hz): forward 0.5 ->
  0.49 m/s, sideways 0.3 -> 0.24 m/s, turn 0.5 -> 0.45 rad/s, standing still
  on a zero command. The command is the setpoint's velocity plus a position
  correction.
  """
  k_pos: float = 1.5  # 1/s, position error to velocity
  k_yaw: float = 1.5  # 1/s, yaw error to turn rate
  max_speed: float = 0.8  # m/s, the policies' command limit
  max_turn: float = 0.8  # rad/s
  hold: float = 0.03  # m: stand within this of a resting setpoint
  hold_yaw: float = math.radians(3)

  def twist(self, sp_pos, sp_vel, sp_yaw, pos, yaw) -> np.ndarray:
    err = np.asarray(sp_pos[:2], float) - pos[:2]
    v = np.asarray(sp_vel[:2], float)
    vel = v + self.k_pos * err
    if float(np.hypot(*v)) < 0.02 and float(np.hypot(*err)) < self.hold:
      vel = np.zeros(2)
    speed = float(np.hypot(*vel))
    if speed > self.max_speed:
      vel *= self.max_speed / speed
    c, s = math.cos(yaw), math.sin(yaw)
    fwd, side = c * vel[0] + s * vel[1], -s * vel[0] + c * vel[1]
    yaw_err = math.remainder(sp_yaw - yaw, 2.0 * math.pi)
    turn = 0.0 if abs(yaw_err) < self.hold_yaw else float(
        np.clip(self.k_yaw * yaw_err, -self.max_turn, self.max_turn))
    return np.array([fwd, side, turn], np.float32)

