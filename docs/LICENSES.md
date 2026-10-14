# Licensing Notes

This repository is a mixed-license distribution.

## Repository-Level Summary

- The planning and control stack includes GPLv3 upstream components from
  EGO-Planner / Fast-Planner and `px4ctrl`.
- Some utility and message packages remain BSD, LGPLv3, or MIT.
- As distributed in this repository, the overall branch should be treated
  conservatively as GPLv3-compatible software with per-package exceptions
  preserved where they already exist.

The root [LICENSE](../LICENSE) file therefore publishes the
repository under GPLv3 terms for redistribution of this combined codebase.

## Important Subpackage Notes

- `px4ctrl/` already ships its own GPLv3 license file.
- `utils/DecompROS/` already ships its own BSD 3-Clause license file.
- `vk180-ros/` declares MIT in its package metadata.
- Several imported planning packages inherited `TODO` license placeholders in
  `package.xml`; for the main planning/simulation path, this branch aligns those
  placeholders with the GPLv3 license used by the upstream EGO-Planner /
  Fast-Planner repositories.

## Practical Guidance

- Keep upstream `LICENSE` files in place when moving or reusing code.
- Preserve more permissive package licenses where they are explicitly declared.
- When importing new upstream code, copy its license file or license notice into
  the repository at the same time.
