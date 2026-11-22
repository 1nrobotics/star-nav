"""Trackers: from the planner's setpoint to a body-frame velocity command.

A ground robot's locomotion policy follows a velocity (forward, sideways,
turn rate). A tracker gives it that velocity from the position setpoint and
yaw the planner sends, and the robot's pose. Each robot names its tracker
(``Robot.tracker``): ``Holonomic`` for one that walks in any direction,
``Unicycle`` for one that cannot move sideways.
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


@dataclasses.dataclass
class Unicycle:
  """Follows the setpoint with a forward speed and a turn rate only.

  The robot cannot move sideways. It turns towards where it should go (the
  setpoint's velocity plus a position correction) and drives the part of
  that velocity that lies ahead; facing more than ``turn_first`` away, it
  turns on the spot. At a resting setpoint it turns to the planner's yaw.
  """
  k_pos: float = 1.5  # 1/s, position error to velocity
  k_yaw: float = 3.0  # 1/s, heading error to turn rate
  max_speed: float = 1.0  # m/s
  max_turn: float = 2.5  # rad/s
  turn_first: float = math.radians(60)
  hold: float = 0.05  # m: stop within this of a resting setpoint ...
  release: float = 0.12  # ... until it is this far away again
  hold_yaw: float = math.radians(5)
  _holding: bool = False

  def twist(self, sp_pos, sp_vel, sp_yaw, pos, yaw) -> np.ndarray:
    err = np.asarray(sp_pos[:2], float) - pos[:2]
    v = np.asarray(sp_vel[:2], float)
    dist = float(np.hypot(*err))
    resting = float(np.hypot(*v)) < 0.02
    self._holding = resting and dist < (self.release if self._holding
                                        else self.hold)
    if self._holding:
      fwd, heading = 0.0, sp_yaw
      dead = self.hold_yaw
    else:
      vel = v + self.k_pos * err
      speed = min(float(np.hypot(*vel)), self.max_speed)
      heading = math.atan2(vel[1], vel[0])
      off = math.remainder(heading - yaw, 2.0 * math.pi)
      fwd = speed * math.cos(off) if abs(off) < self.turn_first else 0.0
      dead = 0.0
    off = math.remainder(heading - yaw, 2.0 * math.pi)
    turn = 0.0 if abs(off) < dead else float(
        np.clip(self.k_yaw * off, -self.max_turn, self.max_turn))
    return np.array([fwd, 0.0, turn], np.float32)
