"""Tests for what the walking robots' controllers share (locomotion.py).

A two-joint model stands in for a robot, so these need no downloaded files.
"""

import mujoco
import numpy as np
import pytest

from star_nav_mujoco import locomotion

_XML = '''<mujoco>
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="trunk">
      <freejoint/>
      <geom size="0.1" mass="1"/>
      <body pos="0 0.1 0">
        <joint name="a" axis="0 1 0"/>
        <geom size="0.02" pos="0 0 -0.1" mass="0.1"/>
      </body>
      <body pos="0 -0.1 0">
        <joint name="b" axis="0 1 0"/>
        <geom size="0.02" pos="0 0 -0.1" mass="0.1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="a" joint="a" ctrlrange="-2 2"/>
    <motor name="b" joint="b" ctrlrange="-5 5"/>
  </actuator>
</mujoco>'''


class _Legs(locomotion.JointPolicy):
  title = 'Legs'
  trunk = 'trunk'
  actuators = ('a', 'b')
  default_pose = np.array([0.1, -0.1], np.float32)
  action_scale = 0.5
  kp, kd = 20.0, 0.5
  decimation = 4

  def __init__(self, model, data):
    self.seen = []  # the observations the policy was given
    self.steps = 0
    super().__init__(model, data)

  def observation(self):
    return np.concatenate([self.twist, self.joint_pos(), self.last])

  def act(self, obs):
    self.seen.append(obs)
    return np.array([1.0, -1.0])

  def advance(self):
    self.steps += 1


def _legs(cls=_Legs):
  model = mujoco.MjModel.from_xml_string(_XML)
  data = mujoco.MjData(model)
  loco = cls(model, data)
  mujoco.mj_forward(model, data)
  return model, data, loco


def test_starts_in_the_standing_pose():
  _, data, loco = _legs()
  np.testing.assert_array_equal(data.qpos[7:9], _Legs.default_pose)
  np.testing.assert_array_equal(loco.joint_pos(), [0.0, 0.0])
  np.testing.assert_array_equal(loco.last, [0.0, 0.0])
  np.testing.assert_allclose(loco.gravity(), [0.0, 0.0, -1.0], atol=1e-12)
  assert loco.policy_due


def test_the_policy_runs_every_decimation_steps_and_pd_on_every_step():
  model, data, loco = _legs()
  due = []
  for _ in range(9):
    due.append(loco.policy_due)
    loco.before_step()
    mujoco.mj_step(model, data)
  assert due == [True, False, False, False, True, False, False, False, True]
  assert len(loco.seen) == 3 and loco.steps == 9
  # The second observation has the first action in it.
  np.testing.assert_array_equal(loco.seen[1][-2:], [1.0, -1.0])


def test_the_action_moves_the_joints_to_pose_plus_scaled_action():
  model, data, loco = _legs()
  for _ in range(2000):
    loco.before_step()
    mujoco.mj_step(model, data)
  # default_pose + action * action_scale
  np.testing.assert_allclose(data.qpos[7:9], [0.6, -0.6], atol=0.01)


def test_torques_are_clipped_to_the_actuators_range():
  model, data, loco = _legs()
  data.qpos[7:9] = [-3.0, 3.0]  # far from the target: PD asks for 70 N m
  loco.before_step()
  np.testing.assert_array_equal(data.ctrl, [2.0, -5.0])


def test_the_velocity_to_follow_reaches_the_policy():
  _, _, loco = _legs()
  loco.twist = np.array([0.4, 0.0, -0.2], np.float32)
  loco.before_step()
  np.testing.assert_allclose(loco.seen[0][:3], [0.4, 0.0, -0.2])


def test_reset_puts_it_back():
  model, data, loco = _legs()
  for _ in range(3):
    loco.before_step()
    mujoco.mj_step(model, data)
  mujoco.mj_resetData(model, data)
  loco.reset()
  np.testing.assert_array_equal(data.qpos[7:9], _Legs.default_pose)
  np.testing.assert_array_equal(loco.last, [0.0, 0.0])
  assert loco.policy_due


def test_a_model_with_another_actuator_order_is_refused():
  class Swapped(_Legs):
    actuators = ('b', 'a')

  with pytest.raises(ValueError, match='unexpected Legs actuator order'):
    _legs(Swapped)
