"""What the robots' learned policies share: small networks run in NumPy.

The policies are a few dense layers (and one GRU), so they need no inference
library; ``onnx_initializers`` reads their weights from an ONNX file.
"""

from __future__ import annotations

import pathlib

import mujoco
import numpy as np

_DOWN = np.array([0.0, 0.0, -1.0])


def projected_gravity(quat: np.ndarray) -> np.ndarray:
  """Direction of gravity in the frame of a body with world ``quat``."""
  out = np.empty(3)
  mujoco.mju_rotVecQuat(
      out, _DOWN, np.array([quat[0], -quat[1], -quat[2], -quat[3]]))
  return out


def elu(x: np.ndarray) -> np.ndarray:
  return np.where(x > 0, x, np.expm1(np.minimum(x, 0)))


def mlp(layers: list[tuple[np.ndarray, np.ndarray]], x: np.ndarray) -> np.ndarray:
  """Dense layers (weight, bias) with ELU between them."""
  for i, (w, b) in enumerate(layers):
    x = w @ x + b
    if i < len(layers) - 1:
      x = elu(x)
  return x


def _sigmoid(x: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-x))


def gru(w: np.ndarray, r: np.ndarray, b: np.ndarray, x: np.ndarray,
        hidden: np.ndarray) -> np.ndarray:
  """One step of an ONNX GRU (linear_before_reset); returns the new state.

  ``w`` (3n, inputs), ``r`` (3n, n) and ``b`` (6n: input then recurrent
  biases) hold the gates in the order update, reset, new.
  """
  n = len(hidden)
  gi = w @ x + b[:3 * n]
  gh = r @ hidden + b[3 * n:]
  update = _sigmoid(gi[:n] + gh[:n])
  reset = _sigmoid(gi[n:2 * n] + gh[n:2 * n])
  new = np.tanh(gi[2 * n:] + reset * gh[2 * n:])
  return (1.0 - update) * new + update * hidden


def _varint(buf: memoryview, i: int) -> tuple[int, int]:
  value = shift = 0
  while True:
    byte = buf[i]
    i += 1
    value |= (byte & 0x7F) << shift
    shift += 7
    if byte < 0x80:
      return value, i


def _fields(buf: memoryview):
  """Yields (field number, wire type, value) of a protobuf message."""
  i = 0
  while i < len(buf):
    key, i = _varint(buf, i)
    wire = key & 7
    if wire == 0:
      value, i = _varint(buf, i)
    elif wire == 2:
      n, i = _varint(buf, i)
      value = buf[i:i + n]
      i += n
    elif wire in (1, 5):
      n = 8 if wire == 1 else 4
      value = buf[i:i + n]
      i += n
    else:
      raise ValueError(f'unsupported protobuf wire type {wire}')
    yield key >> 3, wire, value


def _packed(buf: memoryview):
  """Yields each varint of a packed repeated field."""
  i = 0
  while i < len(buf):
    value, i = _varint(buf, i)
    yield value


def onnx_initializers(
    onnx: pathlib.Path) -> dict[str, tuple[tuple[int, ...], int, memoryview]]:
  """Reads the tensors stored in an ONNX file: name -> (dims, type, raw data).

  ModelProto.graph (7) -> GraphProto.initializer (5) -> TensorProto: dims (1),
  data_type (2, 1 = float32), name (8), raw_data (9). Nothing is executed.
  """
  data = memoryview(onnx.read_bytes())
  graph = next((v for f, w, v in _fields(data) if f == 7 and w == 2), None)
  if graph is None:
    raise ValueError(f'{onnx}: no graph')
  out = {}
  for field, wire, tensor in _fields(graph):
    if field != 5 or wire != 2:
      continue
    dims, name, dtype, raw = [], '', None, None
    for f, w, v in _fields(tensor):
      if f == 1:
        dims += [v] if w == 0 else list(_packed(v))
      elif f == 2:
        dtype = v
      elif f == 8:
        name = bytes(v).decode()
      elif f == 9:
        raw = v
    out[name] = (tuple(dims), dtype, raw)
  return out
