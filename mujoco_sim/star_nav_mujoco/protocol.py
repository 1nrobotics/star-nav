"""ZeroMQ wire protocol between the simulator and the ROS bridge.

Every message is two frames: a topic name and a little-endian binary payload.
The C++ bridge (mujoco_bridge/src/protocol.h) mirrors these layouts; keep both
files in sync and bump VERSION on any change.

Simulator -> bridge (simulator binds a PUB socket, default tcp://*:5555):

  b"state"  STATE   time, position, orientation (w, x, y, z), world linear
                    velocity, body angular velocity, quadrotor contact count
  b"depth"  DEPTH   time, camera optical-frame pose in world (x right,
                    y down, z forward), intrinsics, then uint16 depth in mm
                    (row-major, 0 = no return)
  b"lidar"  LIDAR   time, sensor pose in world (x forward, z up), point
                    count, then float32 x, y, z per return in the sensor frame
  b"world"  WORLD   obstacle count, then one OBSTACLE record per obstacle

Bridge -> simulator (simulator binds a SUB socket, default tcp://*:5556):

  b"cmd"    CMD     time, position, velocity, acceleration, yaw, yaw rate
"""

from __future__ import annotations

import dataclasses
import struct

import numpy as np

from star_nav_mujoco import world

VERSION = 1

TOPIC_STATE = b'state'
TOPIC_DEPTH = b'depth'
TOPIC_LIDAR = b'lidar'
TOPIC_WORLD = b'world'
TOPIC_CMD = b'cmd'

# u16 version, u16 pad, f64 t, 3 f64 pos, 4 f64 quat, 3 f64 vel, 3 f64 omega,
# u32 contacts, u32 pad
STATE = struct.Struct('<HHd3d4d3d3dII')
# u16 version, u16 width, u16 height, u16 pad, f64 t, 3 f64 pos, 4 f64 quat,
# 4 f32 fx fy cx cy
DEPTH = struct.Struct('<HHHHd3d4d4f')
# u16 version, u16 beams, u32 count, f64 t, 3 f64 pos, 4 f64 quat
LIDAR = struct.Struct('<HHId3d4d')
# u16 version, u16 pad, u32 count
WORLD = struct.Struct('<HHI')
# u8 kind, 3 pad, 6 f32 x y z a b yaw
OBSTACLE = struct.Struct('<B3x6f')
# u16 version, u16 pad, u32 pad, f64 t, 3 f64 pos, 3 f64 vel, 3 f64 acc,
# f64 yaw, f64 yaw_rate
CMD = struct.Struct('<HHId3d3d3ddd')


class ProtocolError(ValueError):
  pass


def _check(version: int) -> None:
  if version != VERSION:
    raise ProtocolError(f'protocol version {version}, expected {VERSION}')


@dataclasses.dataclass
class State:
  t: float
  pos: np.ndarray
  quat: np.ndarray  # w, x, y, z
  vel: np.ndarray
  omega: np.ndarray
  contacts: int = 0


def pack_state(s: State) -> bytes:
  return STATE.pack(VERSION, 0, s.t, *s.pos, *s.quat, *s.vel, *s.omega,
                    s.contacts, 0)


def unpack_state(buf: bytes) -> State:
  v = STATE.unpack(buf)
  _check(v[0])
  return State(v[2], np.array(v[3:6]), np.array(v[6:10]), np.array(v[10:13]),
               np.array(v[13:16]), v[16])


@dataclasses.dataclass
class Depth:
  t: float
  pos: np.ndarray
  quat: np.ndarray  # w, x, y, z of the optical frame
  intrinsics: tuple[float, float, float, float]  # fx, fy, cx, cy
  image_mm: np.ndarray  # (height, width) uint16


def pack_depth(d: Depth) -> bytes:
  h, w = d.image_mm.shape
  head = DEPTH.pack(VERSION, w, h, 0, d.t, *d.pos, *d.quat, *d.intrinsics)
  return head + np.ascontiguousarray(d.image_mm, dtype='<u2').tobytes()


def unpack_depth(buf: bytes) -> Depth:
  v = DEPTH.unpack_from(buf)
  _check(v[0])
  w, h = v[1], v[2]
  img = np.frombuffer(buf, dtype='<u2', count=w * h, offset=DEPTH.size)
  return Depth(v[4], np.array(v[5:8]), np.array(v[8:12]), tuple(v[12:16]),
               img.reshape(h, w))


def pack_lidar(scan, beams: int) -> bytes:
  pts = np.ascontiguousarray(scan.points, dtype='<f4')
  head = LIDAR.pack(VERSION, beams, len(pts), scan.t, *scan.pos, *scan.quat)
  return head + pts.tobytes()


def unpack_lidar(buf: bytes):
  """Returns (t, pos, quat, beams, points (n, 3) float32)."""
  v = LIDAR.unpack_from(buf)
  _check(v[0])
  n = v[2]
  if len(buf) != LIDAR.size + 12 * n:
    raise ProtocolError('lidar size mismatch')
  pts = np.frombuffer(buf, dtype='<f4', count=3 * n, offset=LIDAR.size)
  return v[3], np.array(v[4:7]), np.array(v[7:11]), v[1], pts.reshape(n, 3)


def pack_world(obstacles: list[world.Obstacle]) -> bytes:
  parts = [WORLD.pack(VERSION, 0, len(obstacles))]
  parts += [OBSTACLE.pack(o.kind, o.x, o.y, o.z, o.a, o.b, o.yaw)
            for o in obstacles]
  return b''.join(parts)


def unpack_world(buf: bytes) -> list[world.Obstacle]:
  version, _, n = WORLD.unpack_from(buf)
  _check(version)
  return [world.Obstacle(*OBSTACLE.unpack_from(buf, WORLD.size + i * OBSTACLE.size))
          for i in range(n)]


def pack_cmd(t: float, pos, vel, acc, yaw: float, yaw_rate: float) -> bytes:
  return CMD.pack(VERSION, 0, 0, t, *pos, *vel, *acc, yaw, yaw_rate)


def unpack_cmd(buf: bytes):
  """Returns (t, pos, vel, acc, yaw, yaw_rate)."""
  v = CMD.unpack(buf)
  _check(v[0])
  return (v[3], np.array(v[4:7]), np.array(v[7:10]), np.array(v[10:13]),
          v[13], v[14])
