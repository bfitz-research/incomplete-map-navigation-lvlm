# Documentation Index

This folder contains the cleaned public documentation for the `incomplete-map-navigation-lvlm` repository.

## Tested environment

The repository was developed and tested with:

- Ubuntu 24
- ROS 2 Jazzy
- NVIDIA Isaac Sim 5.1.0

Other versions may work, but they are not verified here.

## Recommended reading order

1. `01_repo_layout_and_environment_switching.md`  
   Explains the repository structure, the `maps/current` and `notes/current` symlinks, and the `set_current_env.sh` helper.

2. `02_shared_workflow_oracle_logger_overlays.md`  
   Covers the shared experiment utilities used by both architectures:
   - Nav2 bringup
   - oracle generation
   - trajectory logging
   - overlay generation
   - combined overlays across multiple runs

3. `03_double_lvlm_runbook.md`  
   Full terminal-by-terminal runbook for the double-LVLM workflow.

4. `04_single_global_runbook.md`  
   Full terminal-by-terminal runbook for the single-global waypoint workflow.

## Goal coordinates by environment

All experiments in a given environment use the same goal coordinate regardless of whether the run is Nav2-only, double-LVLM, or single-global LVLM.

- `env_office`: `(-15.75, 21.0)`
- `env_simple_maze`: `(2.5, 14.0)`
- `env_warehouse`: `(21.0, 1.2)`

## Notes

- The repository uses a **true-map oracle phase** followed by a **test-map trial phase**.
- The `notes/<env>/...` folder skeleton is intentionally present because the scripts expect it.
- Older internal development notes are not required for the public GitHub snapshot and are not part of this cleaned docs set.
