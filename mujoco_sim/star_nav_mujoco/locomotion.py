"""What the walking robots' controllers share: a learned policy and joint PD.

A policy runs every few physics steps and gives joint angles for the
velocity it is told to follow (``twist``); a PD controller turns them into
torques on every step. ``JointPolicy`` keeps the joint indices, the policy's
timing and the PD. A robot's subclass gives the constants, the observation
and the network (``robots/lite3.py``, ``robots/t1.py``).
"""

from __future__ import annotations

import numpy as np

from star_nav_mujoco import policy as policy_lib


class JointPolicy:
  """A velocity policy every ``decimation`` steps, joint PD on every step.

  Implements ``robots.Locomotion``. A subclass sets the constants below and
  defines ``observation()`` and ``act()``; ``advance()`` is for state that
  moves with the physics (a gait clock).
  """

  title = ''  # the robot, as written in messages
  trunk = ''  # root body
  # Actuator names in the policy's joint order; must be the model's order.
  actuators: tuple[str, ...] = ()
  default_pose: np.ndarray  # standing pose the actions are added to (rad)
  action_scale = 1.0  # rad per unit of action; a number or one per joint
  kp = 0.0  # N m/rad; a number or one per joint
  kd = 0.0  # N m s/rad
  decimation = 1  # physics steps per policy step

  def __init__(self, model, data):
    self.model, self.data = model, data
    if [model.actuator(i).name for i in range(model.nu)] != list(self.actuators):
      raise ValueError(f'unexpected {self.title} actuator order')
    jids = model.actuator_trnid[:, 0]
    self._qpos = model.jnt_qposadr[jids]
    self._qvel = model.jnt_dofadr[jids]
    self._trunk = model.body(self.trunk).id
    self._root_qvel = model.jnt_dofadr[model.body_jntadr[self._trunk]]
    # Torques are clipped to the actuators' range.
    self._lo = model.actuator_ctrlrange[:, 0].copy()
    self._hi = model.actuator_ctrlrange[:, 1].copy()
    self.twist = np.zeros(3, np.float32)  # forward, sideways, turn rate
    self.reset()

  def reset(self) -> None:
    self.data.qpos[self._qpos] = self.default_pose
    self.last = np.zeros(len(self.actuators), np.float32)  # the last action
    self._target = self.default_pose.copy()
    self._steps = 0

  # -- what the observations are made of ---------------------------------------

  def gravity(self) -> np.ndarray:
    """Direction of gravity in the trunk's frame."""
    return policy_lib.projected_gravity(self.data.xquat[self._trunk])

  def omega(self) -> np.ndarray:
    """The trunk's angular velocity in its own frame (rad/s)."""
    return self.data.qvel[self._root_qvel + 3:self._root_qvel + 6]

  def joint_pos(self) -> np.ndarray:
    """Joint angles relative to the standing pose (rad)."""
    return self.data.qpos[self._qpos] - self.default_pose

  def joint_vel(self) -> np.ndarray:
    return self.data.qvel[self._qvel]

  # -- given by the robot ------------------------------------------------------

  def observation(self) -> np.ndarray:
    """The policy's input now."""
    raise NotImplementedError

  def act(self, obs: np.ndarray) -> np.ndarray:
    """The policy's output for ``obs``: one action per joint."""
    raise NotImplementedError

  def advance(self) -> None:
    """Called on every physics step, after the policy and before the PD."""

  # -- stepping ----------------------------------------------------------------

  @property
  def policy_due(self) -> bool:
    """True if the next ``before_step`` runs the policy (set ``twist`` now)."""
    return self._steps % self.decimation == 0

  def before_step(self) -> None:
    """Call before every mj_step: the policy when it is due, PD always."""
    d = self.data
    if self.policy_due:
      self.last = self.act(self.observation()).astype(np.float32)
      self._target = self.default_pose + self.last * self.action_scale
    self._steps += 1
    self.advance()
    torque = (self.kp * (self._target - d.qpos[self._qpos]) -
              self.kd * d.qvel[self._qvel])
    d.ctrl[:] = np.clip(torque, self._lo, self._hi)
