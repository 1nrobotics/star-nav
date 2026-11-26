#!/usr/bin/env bash
# Downloads the pre-computed path set of the CMU local planner (29 MB, BSD).
#
# planner/local_planner reads these files at start (planner:=cmu). They come
# from the upstream repository at a pinned commit, are checked against known
# checksums and are not part of this repository.
#
#   ./scripts/fetch_cmu_paths.sh
#   STAR_NAV_CMU_PATHS_DIR=/some/dir ./scripts/fetch_cmu_paths.sh

set -euo pipefail

commit="bf0cba71365271ebff09831a05afd78578150300"
base="https://raw.githubusercontent.com/HongbiaoZ/autonomous_exploration_development_environment/${commit}/src/local_planner/paths"
dest="${STAR_NAV_CMU_PATHS_DIR:-${HOME}/.cache/star_nav/cmu_paths}"

files=(
  "startPaths.ply 5be6026aa9a2eb028d0a5829638304a95b2674265efacb35975363d740b0318a"
  "paths.ply 262ad9308e269454a50f45baa2e7030d86ed88c32db73c679cee4973cd70a991"
  "pathList.ply 115bd9b0f8e950779b327ea20438deac2cb7b0967cade33da40c4552a28aaf7b"
  "correspondences.txt 8d2e1b6a628608dc167c8b3326b9c8478569d93a68b31b1a602406c34cb5a47e"
)

mkdir -p "${dest}"
for entry in "${files[@]}"; do
  read -r name sum <<<"${entry}"
  if [[ -f "${dest}/${name}" ]] &&
     echo "${sum}  ${dest}/${name}" | sha256sum --check --status; then
    echo "${name}: already there"
    continue
  fi
  echo "${name}: downloading"
  curl --fail --location --silent --show-error -o "${dest}/${name}.part" "${base}/${name}"
  if ! echo "${sum}  ${dest}/${name}.part" | sha256sum --check --status; then
    echo "${name}: unexpected checksum; not kept (${dest}/${name}.part)" >&2
    exit 1
  fi
  mv "${dest}/${name}.part" "${dest}/${name}"
done
echo "Ready: ${dest}"
