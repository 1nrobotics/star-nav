"""Quadrotor simulator: MuJoCo physics, flight controller and depth camera."""

from __future__ import annotations

import dataclasses
import logging
import struct
import time

import mujoco
import numpy as np

from star_nav_mujoco import protocol
from star_nav_mujoco import quadrotor
from star_nav_mujoco import scene
from star_nav_mujoco import world

log = logging.getLogger('star_nav_mujoco')

# MuJoCo cameras look along -z with +y up; the optical frame used by ROS
# looks along +z with +y down.
_CAM_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])


@dataclasses.dataclass
class SimConfig:
  world: world.WorldConfig = dataclasses.field(default_factory=world.WorldConfig)
  params: quadrotor.Params = dataclasses.field(default_factory=quadrotor.Params)
  gains: quadrotor.Gains = dataclasses.field(default_factory=quadrotor.Gains)
  start: tuple[float, float, float] = (0.0, 0.0, 1.0)
  timestep: float = 0.002
  depth_range: float = 10.0  # returns beyond this are reported as 0
  cmd_timeout: float = 0.5  # hold position when commands stop (s)


class Simulator:
  """Owns the MuJoCo model and data; stepping is driven by the caller."""

  def __init__(
      self,
      cfg: SimConfig | None = None,
      render_depth: bool = True,
      obstacles: list[world.Obstacle] | None = None,
  ):
    self.cfg = cfg or SimConfig()
    if obstacles is None:
      obstacles = world.generate(
          dataclasses.replace(self.cfg.world, start=self.cfg.start[:2]))
    self.obstacles = obstacles
    xml = scene.build_xml(self.obstacles, self.cfg.params, self.cfg.start,
                          self.cfg.timestep)
    self.model = mujoco.MjModel.from_xml_string(xml)
    self.data = mujoco.MjData(self.model)
    self.controller = quadrotor.Controller(self.cfg.params, self.cfg.gains)

    m = self.model
    self._body = m.body('quad').id
    self._cam = m.camera('depth').id
    self._mocap = m.body_mocapid[m.body('setpoint').id]
    self.contact_steps = 0
    self.reset()

    self._renderer = None
    if render_depth:
      # Primitive meshes are built when the context is created. Coarser ones
      # (at most ~2 cm off on a 0.5 m pillar, below the 0.1 m map resolution)
      # make software rendering several times faster; the viewer is unaffected.
      quality = m.vis.quality
      saved = quality.numslices, quality.numstacks
      quality.numslices, quality.numstacks = 12, 6
      try:
        self._renderer = mujoco.Renderer(m, scene.CAM_HEIGHT, scene.CAM_WIDTH)
      finally:
        quality.numslices, quality.numstacks = saved
      self._renderer.enable_depth_rendering()

  # -- state -----------------------------------------------------------------

  def reset(self) -> None:
    mujoco.mj_resetData(self.model, self.data)
    hover = self.cfg.params.mass * quadrotor.GRAVITY / 4.0
    self.data.ctrl[:] = hover
    self.data.act[:] = hover
    self.controller.reset()
    self.contact_steps = 0
    self.set_command(quadrotor.Command.hover(self.cfg.start), t=0.0)
    mujoco.mj_forward(self.model, self.data)

  @property
  def time(self) -> float:
    return self.data.time

  @property
  def pos(self) -> np.ndarray:
    return self.data.qpos[0:3].copy()

  @property
  def rot(self) -> np.ndarray:
    return self.data.xmat[self._body].reshape(3, 3).copy()

  def state(self) -> protocol.State:
    d = self.data
    return protocol.State(d.time, d.qpos[0:3].copy(), d.qpos[3:7].copy(),
                          d.qvel[0:3].copy(), d.qvel[3:6].copy(),
                          self.contact_steps)

  # -- commands ----------------------------------------------------------------

  def set_command(self, cmd: quadrotor.Command, t: float | None = None) -> None:
    self._cmd = cmd
    self._cmd_time = self.data.time if t is None else t
    self.data.mocap_pos[self._mocap] = cmd.pos

  def _active_command(self) -> quadrotor.Command:
    if self.data.time - self._cmd_time > self.cfg.cmd_timeout:
      # Commands stopped: hold the last commanded position.
      return quadrotor.Command.hover(self._cmd.pos, self._cmd.yaw)
    return self._cmd

  # -- stepping ----------------------------------------------------------------

  def step(self, n: int = 1) -> None:
    m, d = self.model, self.data
    p = self.cfg.params
    for _ in range(n):
      vel = d.qvel[0:3]
      d.ctrl[:] = self.controller.update(
          self._active_command(), d.qpos[0:3], d.xmat[self._body].reshape(3, 3),
          vel, d.qvel[3:6], m.opt.timestep)
      d.xfrc_applied[self._body, 0:3] = -p.drag * vel
      mujoco.mj_step(m, d)
      if d.ncon and self._touching():
        self.contact_steps += 1

  def _touching(self) -> bool:
    bodies = self.model.geom_bodyid
    for c in self.data.contact[:self.data.ncon]:
      if bodies[c.geom1] == self._body or bodies[c.geom2] == self._body:
        return True
    return False

  # -- sensing -----------------------------------------------------------------

  def camera_pose(self) -> tuple[np.ndarray, np.ndarray]:
    """Returns the optical-frame position and quaternion (w, x, y, z)."""
    rot = self.data.cam_xmat[self._cam].reshape(3, 3) @ _CAM_TO_OPTICAL
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, rot.flatten())
    return self.data.cam_xpos[self._cam].copy(), quat

  def render_depth(self) -> protocol.Depth:
    """Renders the onboard depth camera (millimetres, 0 = no return)."""
    if self._renderer is None:
      raise RuntimeError('Simulator was created with render_depth=False')
    self._renderer.update_scene(self.data, camera=self._cam)
    depth = self._renderer.render()
    valid = (depth > 0) & (depth < self.cfg.depth_range)
    mm = np.where(valid, depth * 1000.0 + 0.5, 0).astype(np.uint16)
    pos, quat = self.camera_pose()
    f = scene.CAM_FOCAL
    intr = (f, f, scene.CAM_WIDTH / 2.0, scene.CAM_HEIGHT / 2.0)
    return protocol.Depth(self.data.time, pos, quat, intr, mm)

  def close(self) -> None:
    if self._renderer is not None:
      self._renderer.close()
      self._renderer = None


@dataclasses.dataclass
class RunConfig:
  bind: str = '127.0.0.1'
  state_port: int = 5555
  cmd_port: int = 5556
  state_rate: float = 200.0
  depth_rate: float = 30.0
  realtime_factor: float = 1.0
  viewer: bool = True
  viewer_port: int = 8080
  viewer_rate: float = 30.0
  duration: float = 0.0  # stop after this much sim time (0 = forever)


def run(sim: Simulator, rc: RunConfig) -> None:
  """Runs the simulator in (scaled) real time, serving ZeroMQ and the viewer."""
  import contextlib  # pylint: disable=g-import-not-at-top
  import zmq  # pylint: disable=g-import-not-at-top

  ctx = zmq.Context.instance()
  pub = ctx.socket(zmq.PUB)
  pub.setsockopt(zmq.SNDHWM, 20)
  pub.bind(f'tcp://{rc.bind}:{rc.state_port}')
  sub = ctx.socket(zmq.SUB)
  sub.setsockopt(zmq.SUBSCRIBE, protocol.TOPIC_CMD)
  sub.bind(f'tcp://{rc.bind}:{rc.cmd_port}')
  log.info('ZeroMQ: state/depth/world on tcp://%s:%d, commands on tcp://%s:%d',
           rc.bind, rc.state_port, rc.bind, rc.cmd_port)

  world_msg = protocol.pack_world(sim.obstacles)
  dt = sim.model.opt.timestep
  state_every = max(1, round(1.0 / (rc.state_rate * dt)))
  depth_period = 1.0 / rc.depth_rate
  next_depth = 0.0
  next_world = 0.0
  steps = 0
  stats = {'steps': 0, 'depth': 0, 'render': 0.0, 'cmds': 0}
  stats_at = time.monotonic()

  with contextlib.ExitStack() as stack:
    handle = None
    if rc.viewer:
      from star_nav_mujoco import viewer  # pylint: disable=g-import-not-at-top
      handle = viewer.launch(rc.viewer_port, stack)
      handle.sync(sim.model, sim.data)
    viewer_period = 1.0 / rc.viewer_rate
    next_view = 0.0

    wall0 = time.monotonic()
    sim0 = sim.time
    while True:
      wall = time.monotonic()
      target = sim0 + (wall - wall0) * rc.realtime_factor
      if target - sim.time > 0.25:
        # Fell behind (e.g. the host was suspended): resynchronise.
        log.warning('Simulation fell %.2fs behind real time; skipping ahead',
                    target - sim.time)
        wall0, sim0 = wall, sim.time
        target = sim.time

      while sim.time + 0.5 * dt < target:
        sim.step()
        steps += 1
        stats['steps'] += 1
        if steps % state_every == 0:
          pub.send_multipart([protocol.TOPIC_STATE,
                              protocol.pack_state(sim.state())])
        if sim.time >= next_depth:
          t0 = time.perf_counter()
          depth = sim.render_depth()
          stats['render'] += time.perf_counter() - t0
          stats['depth'] += 1
          pub.send_multipart([protocol.TOPIC_DEPTH, protocol.pack_depth(depth)],
                             copy=False)
          next_depth = max(next_depth + depth_period, sim.time)

      while sub.poll(0):
        topic, payload = sub.recv_multipart()
        if topic != protocol.TOPIC_CMD:
          continue
        try:
          _, pos, vel, acc, yaw, yaw_rate = protocol.unpack_cmd(payload)
        except (protocol.ProtocolError, struct.error) as e:
          log.warning('Dropping bad command: %s', e)
          continue
        sim.set_command(quadrotor.Command(pos, vel, acc, yaw, yaw_rate))
        stats['cmds'] += 1

      if sim.time >= next_world:
        pub.send_multipart([protocol.TOPIC_WORLD, world_msg])
        next_world = sim.time + 1.0

      if handle is not None and wall >= next_view:
        if not handle.is_running():
          break
        model, data = handle.sync(sim.model, sim.data)
        if data is not None and data is not sim.data:
          sim.data = data  # the viewer reset or reloaded the state
        next_view = wall + viewer_period

      if wall - stats_at >= 10.0:
        span = wall - stats_at
        log.info('rt=%.2f physics=%.0fHz depth=%.1fHz (%.1fms) cmds=%.0fHz '
                 'contacts=%d', stats['steps'] * dt / span,
                 stats['steps'] / span, stats['depth'] / span,
                 1000 * stats['render'] / max(stats['depth'], 1),
                 stats['cmds'] / span, sim.contact_steps)
        stats = dict.fromkeys(stats, 0)
        stats['render'] = 0.0
        stats_at = wall

      if rc.duration and sim.time >= rc.duration:
        break
      time.sleep(0.001)

  pub.close(0)
  sub.close(0)
