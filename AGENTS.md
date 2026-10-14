# AGENTS.md

This repository is being prepared as an open-source branch. Any coding agent
working here should optimize for a clean public contributor experience.

## Priorities

1. Keep the simulation workflow easy to discover and easy to run.
2. Treat real-hardware support as a documented secondary path.
3. Preserve and clarify upstream licensing instead of simplifying it away.
4. Prefer small, reviewable cleanup steps over broad rewrites.

## Repository Rules

- Do not remove upstream license files from subpackages.
- Do not assume the whole repo is MIT: this repo contains mixed licenses,
  including GPLv3 components from the planning/control stack.
- Keep root documentation public-facing and task-oriented.
- When changing launch or Docker behavior, verify the simulation path first.
- Avoid introducing new hard dependencies on proprietary VK180 tooling unless
  the feature is explicitly hardware-only.

## Recommended Workflow

1. Read [README.md](README.md) first.
2. If the change affects onboarding, update:
   - [docs/SIMULATION.md](docs/SIMULATION.md)
   - [docs/HARDWARE.md](docs/HARDWARE.md)
   - [CONTRIBUTING.md](CONTRIBUTING.md)
3. If the change affects licensing, update:
   - [LICENSE](LICENSE)
   - [docs/LICENSES.md](docs/LICENSES.md)
4. If the change affects scripted entrypoints, prefer updating the helper
   scripts in `/scripts` so docs stay short.

## Validation Expectations

- Shell scripts: run `bash -n`.
- XML/package metadata: keep package names and launch file references accurate.
- Documentation: make simulation commands copy-pasteable.
- Build changes: avoid breaking simulation-only builds when VK180 SDK is absent.
