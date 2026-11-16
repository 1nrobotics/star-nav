#!/usr/bin/env bash

# Headless end-to-end simulation test. Works on any Docker host (Linux, macOS,
# Windows/WSL2); exits non-zero when the exploration mission fails.

set -euo pipefail

REPO_ROOT=$(realpath "$(dirname "$(realpath "$0")")/..")
cd "${REPO_ROOT}"

docker compose run --rm --build test
