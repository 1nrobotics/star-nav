## Summary

<!-- What changes and why. Link related issues. -->

## Validation

<!-- Tick what applies; see AGENTS.md / CONTRIBUTING.md. -->

- [ ] `rostest star_nav_bringup sim_smoke.test` (or `docker compose run --rm test`)
- [ ] MuJoCo simulator or bridge changed: `pytest` in `mujoco_sim/` and `mujoco_smoke.test` (or `./scripts/run_mujoco_smoke_test.sh`)
- [ ] Shell scripts pass `bash -n`
- [ ] Docs updated (README, docs/SIMULATION.md, site/) if commands or behaviour changed
- [ ] No hardware- or vendor-SDK-specific dependencies added to `main`
- [ ] Licensing: new upstream code comes with its license notice (docs/LICENSES.md)
