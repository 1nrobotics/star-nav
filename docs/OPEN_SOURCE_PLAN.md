# Open-Source Branch Plan

This document tracks the repository changes needed to make `star-nav` usable as
a public-facing codebase.

## Goals

- Provide a clear, simulation-first quick start.
- Document the real-hardware workflow without blocking public users.
- Clean up repository hygiene issues that make the repo look internal-only.
- Clarify mixed licensing inherited from upstream planning/control components.

## Phase 1: Repository Foundation

- [x] Audit repository structure, launch entrypoints, and licensing signals.
- [x] Add agent collaboration guidance in `AGENTS.md`.
- [x] Add root repository hygiene files (`.gitignore`, `.editorconfig`,
      `.clang-format`).
- [x] Add root-level license and mixed-license documentation.

## Phase 2: Documentation Refresh

- [x] Rewrite the root README around simulation-first onboarding.
- [x] Add dedicated simulation documentation.
- [x] Add dedicated hardware deployment documentation.
- [x] Add contribution guidance for external collaborators.

## Phase 3: Simulation Usability

- [x] Add simple helper scripts for launching simulation and triggering
      exploration.
- [x] Make the Docker-based simulation path work without the proprietary VK180
      SDK.
- [ ] Run a full end-to-end simulation smoke test in a clean ROS Noetic
      environment.

## Phase 4: Metadata and Cleanup

- [x] Fix package license metadata for the main planning/simulation path where
      upstream GPLv3 provenance is clear.
- [x] Remove tracked editor backups, Python bytecode, and nested package build
      outputs.
- [ ] Continue normalizing README quality inside subpackages with stale or
      conflicting instructions.

## Known Follow-Ups

- Some peripheral packages still carry inherited placeholder metadata from
  upstream ROS packages.
- Hardware integration depends on proprietary VK180/Vilota tooling and cannot
  be fully validated in a public environment without those dependencies.
- A public CI workflow is still desirable after the simulation build path is
  exercised in a clean container/runtime.
