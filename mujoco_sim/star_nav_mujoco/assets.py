"""Robot models and policies that are downloaded, not committed.

Each robot's files come from its upstream repository at a pinned commit and
live in ``~/.cache/star_nav/<robot>`` (or ``$STAR_NAV_<ROBOT>_DIR``).
"""

from __future__ import annotations

import hashlib
import os
import pathlib
import urllib.request


class MissingAssets(RuntimeError):
  pass


def directory(robot: str) -> pathlib.Path:
  env = os.environ.get(f'STAR_NAV_{robot.upper()}_DIR')
  if env:
    return pathlib.Path(env).expanduser()
  return pathlib.Path('~/.cache/star_nav', robot).expanduser()


def github(repo: str, commit: str) -> str:
  """Base URL of the files of ``repo`` at ``commit``."""
  return f'https://raw.githubusercontent.com/{repo}/{commit}'


def download(url: str, dest: pathlib.Path, sha256: str | None = None) -> None:
  """Downloads ``url`` to ``dest``; with ``sha256``, checks the file."""
  dest.parent.mkdir(parents=True, exist_ok=True)
  tmp = dest.with_suffix(dest.suffix + '.part')
  with urllib.request.urlopen(url, timeout=60) as r, open(tmp, 'wb') as f:
    while chunk := r.read(1 << 20):
      f.write(chunk)
  if sha256 is not None:
    digest = hashlib.sha256(tmp.read_bytes()).hexdigest()
    if digest != sha256:
      tmp.unlink()
      raise RuntimeError(f'{dest.name}: unexpected sha256 {digest}')
  tmp.replace(dest)
