"""Procedural textures for the scene: wood floor, plaster, wood, fabric, grass.

Generated with numpy when the model is built (deterministic, no image files),
and assigned to the scene's materials. They only change how the scene looks:
the depth camera and the lidar measure geometry, so sensor data is the same.
"""

from __future__ import annotations

import mujoco
import numpy as np

SIZE = 512  # texture side (pixels)


def _noise(rng: np.random.Generator, cells: int, size: int = SIZE) -> np.ndarray:
  """Tileable smooth value noise in [0, 1]: ``cells`` x ``cells`` lattice."""
  grid = rng.random((cells, cells))
  t = np.arange(size) * cells / size
  i0 = t.astype(int)
  f = t - i0
  f = f * f * (3 - 2 * f)  # smoothstep
  i1 = (i0 + 1) % cells
  rows = grid[i0] * (1 - f)[:, None] + grid[i1] * f[:, None]
  return rows[:, i0] * (1 - f)[None, :] + rows[:, i1] * f[None, :]


def _fractal(rng: np.random.Generator, cells: int, octaves: int = 4) -> np.ndarray:
  out = np.zeros((SIZE, SIZE))
  amp, total = 1.0, 0.0
  for o in range(octaves):
    out += amp * _noise(rng, min(cells * 2 ** o, SIZE))
    total += amp
    amp *= 0.5
  return out / total


def _rgb(base, shade: np.ndarray) -> np.ndarray:
  """``base`` colour (0-1) scaled per pixel by ``shade`` (around 1)."""
  img = np.asarray(base, float)[None, None, :] * shade[..., None]
  return np.clip(img * 255.0 + 0.5, 0, 255).astype(np.uint8)


def _grain(rng: np.random.Generator, rings: float) -> np.ndarray:
  """Wood grain along the texture's u axis: warped stripes."""
  warp = _fractal(rng, 4, 3)
  y = np.arange(SIZE)[:, None] / SIZE
  stripes = 0.5 + 0.5 * np.sin(2 * np.pi * (y * rings + warp * 3.0))
  return 0.9 + 0.1 * stripes ** 3 + 0.05 * (_fractal(rng, 64, 2) - 0.5)


def floor(rng: np.random.Generator) -> np.ndarray:
  """Oak planks along x: 12 rows, boards of random length, dark seams."""
  rows = 12
  edges = np.linspace(0, SIZE, rows + 1).astype(int)
  img_shade = np.empty((SIZE, SIZE))
  grain = _grain(rng, 40.0)
  x = np.arange(SIZE)
  for r in range(rows):
    band = slice(edges[r], edges[r + 1])
    # Board ends: 2 or 3 per row (the texture tiles, so the row wraps).
    ends = np.sort(rng.choice(np.arange(32, SIZE - 32), rng.integers(2, 4),
                              replace=False))
    board = np.searchsorted(ends, x) % len(ends)
    tone = 0.88 + 0.2 * rng.random(len(ends))
    img_shade[band] = grain[band] * tone[board][None, :]
    seam = np.zeros(SIZE, bool)
    seam[ends] = seam[(ends + 1) % SIZE] = True
    img_shade[band][:, seam] *= 0.55
    img_shade[edges[r]] *= 0.55
  return _rgb((0.78, 0.6, 0.42), img_shade)


def wood(rng: np.random.Generator) -> np.ndarray:
  return _rgb((0.55, 0.38, 0.24), _grain(rng, 24.0))


def plaster(rng: np.random.Generator) -> np.ndarray:
  shade = 0.96 + 0.06 * (_fractal(rng, 8) - 0.5) + 0.03 * (rng.random((SIZE, SIZE)) - 0.5)
  return _rgb((0.92, 0.9, 0.86), shade)


def fabric(rng: np.random.Generator) -> np.ndarray:
  i = np.arange(SIZE)
  weave = (0.5 + 0.5 * np.sin(i * 2 * np.pi / 6))[:, None] * (0.5 + 0.5 * np.sin(i * 2 * np.pi / 6))[None, :]
  shade = 0.85 + 0.2 * weave + 0.08 * (_fractal(rng, 16, 2) - 0.5)
  return _rgb((0.33, 0.43, 0.6), shade)


def concrete(rng: np.random.Generator) -> np.ndarray:
  shade = 0.92 + 0.14 * (_fractal(rng, 16) - 0.5) + 0.08 * (rng.random((SIZE, SIZE)) - 0.5)
  return _rgb((0.62, 0.62, 0.64), shade)


def grass(rng: np.random.Generator) -> np.ndarray:
  """Grass with patches of earth; 1 m grid lines give a sense of scale."""
  patches = _fractal(rng, 6)
  blades = rng.random((SIZE, SIZE))
  green = np.array([0.36, 0.52, 0.26])
  earth = np.array([0.5, 0.42, 0.3])
  mix = 0.6 * np.clip((patches - 0.6) * 4, 0, 1)[..., None]
  img = (green * (1 - mix) + earth * mix) * (0.85 + 0.25 * blades)[..., None]
  line = np.zeros(SIZE, bool)
  line[:2] = True  # the texture spans 1 m (texrepeat 1 per metre)
  img[line, :] *= 0.8
  img[:, line] *= 0.8
  return np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8)


def bark(rng: np.random.Generator) -> np.ndarray:
  """Vertical ridges (along the texture's v axis)."""
  ridges = _noise(rng, 32)
  streak = np.repeat(_noise(rng, 48)[:1], SIZE, axis=0)
  shade = 0.7 + 0.35 * streak + 0.1 * (ridges - 0.5)
  return _rgb((0.45, 0.35, 0.26), shade)


# material -> (texture generator, repetitions per metre)
TEXTURES = {
    'floor': (floor, 0.5),
    'grass': (grass, 1.0),
    'wall': (plaster, 0.5),
    'wood': (wood, 1.0),
    'fabric': (fabric, 2.0),
    'stone': (concrete, 1.0),
    'pillar': (bark, 1.0),
}


def apply(spec: mujoco.MjSpec, seed: int = 0) -> None:
  """Gives the scene's materials their textures; the ground becomes grass."""
  rng = np.random.default_rng(seed)
  have = {m.name for m in spec.materials}
  for name, (make, repeat) in TEXTURES.items():
    if name not in have:
      continue
    img = make(rng)
    tex = spec.add_texture(name='tex_' + name,
                           type=mujoco.mjtTexture.mjTEXTURE_2D,
                           width=SIZE, height=SIZE, nchannel=3)
    tex.data = np.ascontiguousarray(img).tobytes()
    mat = spec.material(name)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = 'tex_' + name
    mat.texuniform = True
    mat.texrepeat = [repeat, repeat]
    mat.rgba = [1, 1, 1, 1]
  ground = spec.geom('ground')
  if ground is not None:
    ground.material = 'grass'
