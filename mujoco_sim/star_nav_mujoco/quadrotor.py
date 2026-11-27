"""Quadrotor parameters and a geometric SE(3) tracking controller.

The controller follows Lee, Leok and McClamroch, "Geometric tracking control
of a quadrotor UAV on SE(3)" (CDC 2010) and plays the role of the flight
controller: it turns the planner's position setpoint
(``commands.Setpoint``: position, velocity, acceleration, yaw) into four
rotor thrusts.
"""

from __future__ import annotations

import dataclasses
import math

import numpy as np

from star_nav_mujoco import commands

GRAVITY = 9.81


@dataclasses.dataclass(frozen=True)
class Params:
  mass: float = 1.0
  inertia: tuple[float, float, float] = (0.0049, 0.0049, 0.0088)
  arm: float = 0.12  # rotor offset along x and y (X configuration)
  prop_radius: float = 0.065
  torque_coeff: float = 0.016  # yaw moment per newton of thrust
  motor_tau: float = 0.02  # first-order motor time constant (s)
  max_thrust: float = 6.0  # per rotor (N)
  drag: float = 0.15  # linear body drag (N per m/s)

  def rotor_layout(self) -> list[tuple[float, float, int]]:
    """Rotor (x, y, spin) in the body frame; spin +1 means CCW reaction."""
    a = self.arm
    return [(a, a, 1), (-a, -a, 1), (a, -a, -1), (-a, a, -1)]

  def mixer(self) -> np.ndarray:
    """Maps rotor thrusts to [collective thrust, Mx, My, Mz]."""
    return np.array([[1.0, y, -x, s * self.torque_coeff]
                     for x, y, s in self.rotor_layout()]).T


@dataclasses.dataclass(frozen=True)
class Gains:
  kp: tuple[float, float, float] = (7.0, 7.0, 10.0)
  kv: tuple[float, float, float] = (4.5, 4.5, 6.0)
  ki: tuple[float, float, float] = (0.8, 0.8, 1.5)
  kr: tuple[float, float, float] = (2.5, 2.5, 0.8)
  kw: tuple[float, float, float] = (0.18, 0.18, 0.12)
  max_tilt_deg: float = 35.0
  max_integral: float = 2.0
  integral_zone: float = 0.3  # metres


def _vee(m: np.ndarray) -> np.ndarray:
  return np.array([m[2, 1], m[0, 2], m[1, 0]])


def yaw_of(rot: np.ndarray) -> float:
  return math.atan2(rot[1, 0], rot[0, 0])


class Controller:
  """Geometric tracking controller producing per-rotor thrusts."""

  def __init__(self, params: Params, gains: Gains | None = None):
    self.p = params
    self.g = gains or Gains()
    self._mix_inv = np.linalg.inv(params.mixer())
    self._inertia = np.array(params.inertia)
    self._kp = np.array(self.g.kp) * params.mass
    self._kv = np.array(self.g.kv) * params.mass
    self._ki = np.array(self.g.ki) * params.mass
    self._kr = np.array(self.g.kr)
    self._kw = np.array(self.g.kw)
    self._tan_tilt = math.tan(math.radians(self.g.max_tilt_deg))
    self._integral = np.zeros(3)

  def reset(self) -> None:
    self._integral[:] = 0.0

  def update(
      self,
      cmd: commands.Setpoint,
      pos: np.ndarray,
      rot: np.ndarray,
      vel: np.ndarray,
      omega: np.ndarray,
      dt: float,
  ) -> np.ndarray:
    """Returns four rotor thrusts.

    Args:
      cmd: Desired state.
      pos: Position (world).
      rot: Body-to-world rotation matrix.
      vel: Linear velocity (world).
      omega: Angular velocity (body).
      dt: Time since the previous call (s).
    """
    m = self.p.mass
    e_p = pos - cmd.pos
    e_v = vel - cmd.vel
    lim = self.g.max_integral
    # Integrate only near the setpoint so large steps do not wind up.
    near = np.abs(e_p) < self.g.integral_zone
    self._integral = np.clip(self._integral + near * e_p * dt, -lim, lim)

    force = (-self._kp * e_p - self._kv * e_v - self._ki * self._integral
             + m * cmd.acc + m * GRAVITY * np.array([0.0, 0.0, 1.0]))
    # Limit tilt: keep the horizontal component inside the tilt cone.
    force[2] = max(force[2], 0.2 * m * GRAVITY)
    horiz = math.hypot(force[0], force[1])
    max_horiz = force[2] * self._tan_tilt
    if horiz > max_horiz:
      force[:2] *= max_horiz / horiz

    b3 = force / np.linalg.norm(force)
    b1c = np.array([math.cos(cmd.yaw), math.sin(cmd.yaw), 0.0])
    b2 = np.cross(b3, b1c)
    b2 /= np.linalg.norm(b2)
    rot_d = np.column_stack([np.cross(b2, b3), b2, b3])

    e_r = 0.5 * _vee(rot_d.T @ rot - rot.T @ rot_d)
    omega_d = rot.T @ rot_d @ np.array([0.0, 0.0, cmd.yaw_rate])
    e_w = omega - omega_d
    moment = (-self._kr * e_r - self._kw * e_w
              + np.cross(omega, self._inertia * omega))
    thrust = float(force @ rot[:, 2])

    rotors = self._mix_inv @ np.array([thrust, *moment])
    return np.clip(rotors, 0.0, self.p.max_thrust)
