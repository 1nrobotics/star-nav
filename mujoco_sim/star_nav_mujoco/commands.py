"""What a planner asks of a robot: a position setpoint or a velocity.

Every robot takes these two and nothing else. A robot's controller says
which of them it follows (``Controller.accepts`` in ``robots/``); the
simulator refuses the other.
"""

from __future__ import annotations

import dataclasses

import numpy as np


@dataclasses.dataclass
class Setpoint:
  """Where to be now (world frame), as a trajectory server publishes it."""

  pos: np.ndarray
  vel: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
  acc: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
  yaw: float = 0.0
  yaw_rate: float = 0.0

  @classmethod
  def hover(cls, pos, yaw: float = 0.0) -> 'Setpoint':
    """Stay at ``pos``."""
    return cls(pos=np.asarray(pos, dtype=float).copy(), yaw=yaw)


@dataclasses.dataclass
class Velocity:
  """A body-frame velocity, from a planner that steers the robot itself."""

  twist: np.ndarray  # forward (m/s), sideways (m/s), turn rate (rad/s)
