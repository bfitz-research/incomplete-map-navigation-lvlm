# Repository Layout and Environment Switching

This document explains the structure of the public repository and how the active environment is selected.

## Repository root

Throughout these docs, assume:

```bash
REPO=/path/to/incomplete-map-navigation-lvlm
```

## Top-level layout

```text
incomplete-map-navigation-lvlm/
├── config/
├── docs/
├── launch/
├── maps/
├── notes/
├── rviz/
└── scripts/
    └── single_global/
```

## Key folders

### `config/`

Nav2 parameter files for the main experiment conditions:

- `nav2_params_global_updates_on.yaml`
- `nav2_params_global_updates_off.yaml`

### `launch/`

Contains the Nav2 wrapper launch file:

- `nav2_bringup.launch.py`

### `maps/`

Each environment includes both a `true/` map and a `test/` map.

Examples:

- `maps/env_office/true/`
- `maps/env_office/test/`
- `maps/env_simple_maze/true/`
- `maps/env_simple_maze/test/`
- `maps/env_warehouse/true/`
- `maps/env_warehouse/test/`

### `notes/`

Per-environment runtime state and outputs. Typical structure:

```text
notes/<env>/
  debug/
  exports/
    latest/
  oracle/
  runs/
  state/
  ticks/
    latest/
```

These folders are intentionally kept in the repository because the scripts expect them.

### `scripts/`

Shared utilities plus the double-LVLM workflow:

- `run_nav2.sh`
- `set_current_env.sh`
- `compute_oracle_path.py`
- `log_executed_traj_tf.py`
- `make_overlay.py`
- `make_multi_overlay.py`
- `publish_oracle_path.py`
- `publish_executed_path.py`
- `export_crops.py`
- `export_rgb_dual.py`
- `prepare_tick_payload_once.py`
- `region_goal_picker_node.py`
- `run_lvlm_loop_semantic_blocked_regions.py`

### `scripts/single_global/`

Single-global LVLM waypoint workflow:

- `common.py`
- `prepare_single_global_tick.py`
- `run_single_global_waypoint_loop.py`
- `lvlm_single_global_waypoint_prompt.txt`

## Environment switching model

The repository does not hardcode one active environment into all commands.

Instead, it uses two symlinks:

- `maps/current`
- `notes/current`

These are updated by `scripts/set_current_env.sh`.

## Canonical command

From the repository root:

```bash
cd "$REPO"
./scripts/set_current_env.sh <env_name> <true|test>
```

Examples:

```bash
./scripts/set_current_env.sh env_simple_maze true
./scripts/set_current_env.sh env_simple_maze test
./scripts/set_current_env.sh env_office true
./scripts/set_current_env.sh env_warehouse test
```

## Verifying the active environment

```bash
readlink -f "$REPO/maps/current"
readlink -f "$REPO/notes/current"
```

## Why both `true` and `test` maps exist

The typical experiment flow is:

1. Switch to the **true** map and generate the oracle path.
2. Switch to the **test** map and run the actual trial.

This keeps the evaluation baseline separate from the incomplete prior map used during execution.

## Goal coordinates by environment

- `env_office`: `(-15.75, 21.0)`
- `env_simple_maze`: `(2.5, 14.0)`
- `env_warehouse`: `(21.0, 1.2)`
