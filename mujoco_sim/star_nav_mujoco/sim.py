"""Robot simulator: MuJoCo physics, controller, depth camera and lidar.

The robot is one of ``robots/`` (the quadrotor by default); all take the
same commands (``commands.py``) and carry the same sensors.
"""

from __future__ import annotations

import dataclasses
import logging
import math
import struct
import time

import mujoco
import numpy as np

from star_nav_mujoco import commands
from star_nav_mujoco import house
from star_nav_mujoco import lidar as lidar_lib
from star_nav_mujoco import overlay
from star_nav_mujoco import protocol
from star_nav_mujoco import robots
from star_nav_mujoco import scene
from star_nav_mujoco import textures
from star_nav_mujoco import world

log = logging.getLogger('star_nav_mujoco')

# MuJoCo cameras look along -z with +y up; the optical frame used by ROS
# looks along +z with +y down.
_CAM_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])

@dataclasses.dataclass
class SimConfig:
  world: world.WorldConfig = dataclasses.field(default_factory=world.WorldConfig)
  start: tuple[float, float, float] = (0.0, 0.0, 1.0)
  robot: str = robots.DEFAULT  # a module of robots/
  house: bool = False  # the house (house.py) instead of the forest
  timestep: float = 0.002  # ground robots use their own (Robot.timestep)
  depth_range: float = 10.0  # returns beyond this are reported as 0
  cmd_timeout: float = 0.5  # hold position when commands stop (s)
  lidar: lidar_lib.LidarConfig | None = None  # 360-degree lidar, off by default
  draw: bool = False  # markers for the planner's paths (overlay.py)


class Simulator:
  """Owns the MuJoCo model and data; stepping is driven by the caller."""

  def __init__(
      self,
      cfg: SimConfig | None = None,
      render_depth: bool = True,
      obstacles: list[world.Obstacle] | None = None,
  ):
    self.cfg = cfg or SimConfig()
    if obstacles is None and self.cfg.house:
      obstacles = []
    if obstacles is None:
      obstacles = world.generate(
          dataclasses.replace(self.cfg.world, start=self.cfg.start[:2]))
    self.obstacles = obstacles
    self.robot = robots.get(self.cfg.robot)
    if self.cfg.lidar is not None:
      self.cfg.lidar = self.robot.mount_lidar(self.cfg.lidar)
    extra = ''
    if self.cfg.house:
      extra = house.xml(self.robot.house_wall_height)
    spec = self.robot.build(self.cfg, self.obstacles, extra)
    textures.apply(spec)
    self.model = spec.compile()
    self.data = mujoco.MjData(self.model)

    m = self.model
    self._body = m.body(self.robot.body).id
    self._robot_bodies = {b for b in range(m.nbody)
                          if m.body_rootid[b] == m.body_rootid[self._body]}
    self._ground = m.geom('ground').id
    # Where a ground robot's feet, lower legs or wheels (support_geoms) may
    # touch: the ground, ramps, stairs.
    self._walkable = {self._ground} | {
        g for g in range(m.ngeom) if m.geom(g).name.startswith(house.WALKABLE)}
    support = getattr(self.robot, 'support_geoms', ())
    self._feet = {g for g in range(m.ngeom)
                  if support and m.geom(g).name.endswith(support)}
    self._cam = m.camera(self.robot.camera).id
    self._mocap = m.body_mocapid[m.body('setpoint').id]
    self._overlay = overlay.Overlay(m) if self.cfg.draw else None
    self.controller = self.robot.controller(m, self.data, self.cfg)
    self.fallen = False
    self.contact_steps = 0
    self._contact_pairs = set()
    self.reset()

    # Sensors only see physical geoms, not visual markers (a robot's visual
    # meshes) or the robot's own legs.
    self._depth_option = mujoco.MjvOption()
    self._depth_option.geomgroup[scene.MARKER_GROUP] = 0
    self._depth_option.geomgroup[scene.ROBOT_GROUP] = 0

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

    self.lidar = None
    self._scan_data = None
    if self.cfg.lidar is not None:
      body = self._body
      if self.cfg.draw:
        # A ray cast takes longer with every body in the model, also with
        # those it ignores: the path markers made a scan several times
        # slower. The lidar scans a copy of the scene without them.
        m = self.robot.build(dataclasses.replace(self.cfg, draw=False),
                             self.obstacles, extra).compile()
        assert m.nq == self.model.nq
        self._scan_data = mujoco.MjData(m)
        body = m.body(self.robot.body).id
      self.lidar = lidar_lib.Lidar(m, self.cfg.lidar, body)
      self.cfg.lidar = self.lidar.cfg  # with the model's range

  # -- state -----------------------------------------------------------------

  def reset(self) -> None:
    mujoco.mj_resetData(self.model, self.data)
    self.contact_steps = 0
    self.fallen = False
    self._twist = None  # set_velocity: follow a velocity, not the setpoint
    self.controller.reset()
    self.set_command(commands.Setpoint.hover(self.cfg.start), t=0.0)
    mujoco.mj_forward(self.model, self.data)

  @property
  def has_depth(self) -> bool:
    return self._renderer is not None

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

  def set_command(self, cmd: commands.Setpoint, t: float | None = None) -> bool:
    """Sets the tracking target. Returns False if any field was not finite.

    Like a flight controller, non-finite fields are treated as not set: a
    non-finite position rejects the command, non-finite velocity,
    acceleration or yaw rate become zero, a non-finite yaw keeps the last one.
    """
    if not np.all(np.isfinite(cmd.pos)):
      return False
    finite = (np.all(np.isfinite(cmd.vel)) and np.all(np.isfinite(cmd.acc)) and
              math.isfinite(cmd.yaw) and math.isfinite(cmd.yaw_rate))
    if not finite:
      last_yaw = getattr(self, '_cmd', None)
      cmd = commands.Setpoint(
          np.asarray(cmd.pos, float),
          np.nan_to_num(np.asarray(cmd.vel, float), nan=0.0, posinf=0.0, neginf=0.0),
          np.nan_to_num(np.asarray(cmd.acc, float), nan=0.0, posinf=0.0, neginf=0.0),
          cmd.yaw if math.isfinite(cmd.yaw) else (last_yaw.yaw if last_yaw else 0.0),
          cmd.yaw_rate if math.isfinite(cmd.yaw_rate) else 0.0)
    self._cmd = cmd
    self._cmd_time = self.data.time if t is None else t
    self.data.mocap_pos[self._mocap] = cmd.pos
    return bool(finite)

  def set_velocity(self, twist, t: float | None = None) -> bool:
    """Sets a body-frame velocity for the robot to follow.

    Forward (m/s), sideways (m/s) and turn rate (rad/s), from a planner that
    steers the robot itself. From the first one on the robot follows these
    instead of the position setpoint, and stands when they stop. Returns
    False (and changes nothing) for a robot whose controller does not
    follow velocities (``Controller.accepts``) or a value that is not finite.
    """
    twist = np.asarray(twist, float)
    if (commands.Velocity not in self.controller.accepts or
        twist.shape != (3,) or not np.all(np.isfinite(twist))):
      return False
    self._twist = twist
    self._twist_time = self.data.time if t is None else t
    return True

  def draw(self, layer: int, points) -> bool:
    """Draws a path of the planner's (world frame) on a layer of overlay.py.

    Returns False for an unknown layer or points that are not finite. Without
    ``SimConfig.draw`` there is nothing to draw with and it does nothing.
    """
    if self._overlay is None:
      return layer in overlay.LAYERS
    return self._overlay.set(self.data, layer, points)

  def _active_command(self):
    if self._twist is not None:
      if self.data.time - self._twist_time > self.cfg.cmd_timeout:
        return commands.Velocity(np.zeros(3))  # commands stopped: stand
      return commands.Velocity(self._twist)
    if self.data.time - self._cmd_time > self.cfg.cmd_timeout:
      # Commands stopped: hold the last commanded position.
      return commands.Setpoint.hover(self._cmd.pos, self._cmd.yaw)
    return self._cmd

  # -- stepping ----------------------------------------------------------------

  def step(self, n: int = 1) -> None:
    m, d = self.model, self.data
    for _ in range(n):
      self.controller.step(self._active_command())
      mujoco.mj_step(m, d)
      if d.ncon and self._touching():
        self.contact_steps += 1
      elif self._contact_pairs:
        self._contact_pairs = set()
    if (self.robot.on_ground and not self.fallen and
        d.qpos[2] < 0.5 * self.robot.stand_height):
      self.fallen = True
      log.warning('%s fell at t=%.2f (body at %s)', self.cfg.robot, d.time,
                  np.round(d.qpos[0:3], 3))

  def _touching(self) -> bool:
    """True if the robot touches anything (a ground robot: but the ground)."""
    bodies = self.model.geom_bodyid
    robot = self._robot_bodies
    touching = False
    pairs = set()
    for c in self.data.contact[:self.data.ncon]:
      a, b = bodies[c.geom1] in robot, bodies[c.geom2] in robot
      if a == b:
        continue  # self-contact or not the robot
      if self.robot.on_ground and self._ground in (c.geom1, c.geom2):
        continue
      if ({c.geom1, c.geom2} & self._feet) and ({c.geom1, c.geom2} & self._walkable):
        continue
      touching = True
      pair = (int(c.geom1), int(c.geom2))
      pairs.add(pair)
      if pair not in self._contact_pairs:
        log.warning('Contact at t=%.3f: %s / %s at %s (body at %s, setpoint %s)',
                    self.data.time, self._geom_label(c.geom1),
                    self._geom_label(c.geom2), np.round(c.pos, 3),
                    np.round(self.data.qpos[0:3], 3), np.round(self._cmd.pos, 3))
    self._contact_pairs = pairs
    return touching

  def _geom_label(self, g: int) -> str:
    m = self.model
    name = m.geom(g).name or f'geom{g}'
    body = m.body(m.geom_bodyid[g]).name or 'world'
    kind = mujoco.mjtGeom(m.geom_type[g]).name.removeprefix('mjGEOM_').lower()
    return f'{body}:{name}({kind} size={np.round(m.geom_size[g], 3)})'


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
    self._renderer.update_scene(self.data, camera=self._cam,
                                scene_option=self._depth_option)
    depth = self._renderer.render()
    valid = (depth > 0) & (depth < self.cfg.depth_range)
    mm = np.where(valid, depth * 1000.0 + 0.5, 0).astype(np.uint16)
    pos, quat = self.camera_pose()
    f = scene.CAM_FOCAL
    intr = (f, f, scene.CAM_WIDTH / 2.0, scene.CAM_HEIGHT / 2.0)
    return protocol.Depth(self.data.time, pos, quat, intr, mm)

  def scan_lidar(self) -> lidar_lib.Scan:
    """Casts one lidar scan (points in the sensor frame)."""
    if self.lidar is None:
      raise RuntimeError('Simulator was created without a lidar')
    if self._scan_data is None:
      return self.lidar.scan(self.data)
    d = self._scan_data
    d.time = self.data.time
    d.qpos[:] = self.data.qpos
    d.mocap_pos[:] = self.data.mocap_pos[:len(d.mocap_pos)]
    d.mocap_quat[:] = self.data.mocap_quat[:len(d.mocap_quat)]
    mujoco.mj_kinematics(self.lidar.model, d)
    return self.lidar.scan(d)

  def close(self) -> None:
    if self.lidar is not None:
      self.lidar.close()
      self.lidar = None
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
  record: str = ''  # write a follow-camera video to this file


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
  sub.setsockopt(zmq.SUBSCRIBE, protocol.TOPIC_VEL)
  if sim.cfg.draw:
    sub.setsockopt(zmq.SUBSCRIBE, protocol.TOPIC_DRAW)
  sub.bind(f'tcp://{rc.bind}:{rc.cmd_port}')
  log.info('ZeroMQ: state/depth/world on tcp://%s:%d, commands on tcp://%s:%d',
           rc.bind, rc.state_port, rc.bind, rc.cmd_port)

  world_msg = protocol.pack_world(sim.obstacles)
  dt = sim.model.opt.timestep
  state_every = max(1, round(1.0 / (rc.state_rate * dt)))
  depth_period = 1.0 / rc.depth_rate
  next_depth = 0.0 if sim.has_depth else float('inf')
  lidar_period = 1.0 / sim.cfg.lidar.rate if sim.lidar else 0.0
  next_lidar = 0.0 if sim.lidar else float('inf')
  lidar_world = None  # last scan in world coordinates, for the recording
  next_world = 0.0
  steps = 0
  stats = {'steps': 0, 'depth': 0, 'render': 0.0, 'lidar': 0, 'cast': 0.0,
           'cmds': 0, 'bad_cmds': 0}
  stats_at = time.monotonic()

  with contextlib.ExitStack() as stack:
    handle = None
    if rc.viewer:
      from star_nav_mujoco import viewer  # pylint: disable=g-import-not-at-top
      handle = viewer.launch(rc.viewer_port, stack)
      handle.sync(sim.model, sim.data)
    viewer_period = 1.0 / rc.viewer_rate
    recorder = None
    if rc.record:
      from star_nav_mujoco import record  # pylint: disable=g-import-not-at-top
      recorder = stack.enter_context(
          contextlib.closing(record.Recorder(sim.model, rc.record)))
      log.info('Recording to %s', rc.record)
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
        if sim.time >= next_lidar:
          t0 = time.perf_counter()
          scan = sim.scan_lidar()
          stats['cast'] += time.perf_counter() - t0
          stats['lidar'] += 1
          pub.send_multipart([protocol.TOPIC_LIDAR,
                              protocol.pack_lidar(scan, sim.lidar.lines)],
                             copy=False)
          next_lidar = max(next_lidar + lidar_period, sim.time)
          if recorder is not None:
            lidar_world = scan.pos + scan.points @ scan.rot.T
        if recorder is not None:
          recorder.maybe_capture(sim.data, lidar_world)

      while sub.poll(0):
        topic, payload = sub.recv_multipart()
        try:
          if topic == protocol.TOPIC_VEL:
            _, twist = protocol.unpack_vel(payload)
            ok = sim.set_velocity(twist)
          elif topic == protocol.TOPIC_DRAW:
            sim.draw(*protocol.unpack_draw(payload))
            continue
          elif topic == protocol.TOPIC_CMD:
            _, pos, vel, acc, yaw, yaw_rate = protocol.unpack_cmd(payload)
            ok = sim.set_command(commands.Setpoint(pos, vel, acc, yaw, yaw_rate))
          else:
            continue
        except (protocol.ProtocolError, struct.error) as e:
          log.warning('Dropping bad command: %s', e)
          continue
        if not ok:
          stats['bad_cmds'] += 1
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
        sensors = ''
        if sim.has_depth:
          sensors += ' depth=%.1fHz (%.1fms)' % (
              stats['depth'] / span, 1000 * stats['render'] / max(stats['depth'], 1))
        if sim.lidar:
          sensors += ' lidar=%.1fHz (%.1fms)' % (
              stats['lidar'] / span, 1000 * stats['cast'] / max(stats['lidar'], 1))
        if stats['bad_cmds']:
          log.warning('%d commands not followed as sent in the last %.0fs '
                      '(non-finite fields, or a velocity for a robot that follows none)',
                      stats['bad_cmds'], span)
        log.info('rt=%.2f physics=%.0fHz%s cmds=%.0fHz contacts=%d',
                 stats['steps'] * dt / span, stats['steps'] / span, sensors,
                 stats['cmds'] / span, sim.contact_steps)
        stats = dict.fromkeys(stats, 0)
        stats['render'] = stats['cast'] = 0.0
        stats_at = wall

      if rc.duration and sim.time >= rc.duration:
        break
      time.sleep(0.001)

  pub.close(0)
  sub.close(0)
