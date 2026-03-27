# Incomplete Map Navigation with LVLM Planning

Research-code snapshot for incomplete-map navigation experiments using Nav2 plus large vision-language model (LVLM) planning.

This repository contains:

- a shared Nav2-based experiment backbone
- a **double-LVLM** workflow with a global macro planner and a local execution supervisor
- a **single-global LVLM** workflow that plans directly on the full global overlay

This is a **research/paper snapshot**, not a polished production robotics package.

## Tested environment

This code was developed and tested with:

- Ubuntu 24
- ROS 2 Jazzy
- NVIDIA Isaac Sim 5.1.0

Other versions may work, but are not verified here.

## Repository layout

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

## Important folders

- `config/`  
  Nav2 parameter files for the main experiment conditions:
  - `nav2_params_global_updates_on.yaml`
  - `nav2_params_global_updates_off.yaml`

- `launch/`  
  Nav2 bringup wrapper:
  - `nav2_bringup.launch.py`

- `maps/`  
  Environment maps. Each environment has:
  - `true/`
  - `test/`

- `notes/`  
  Runtime outputs and state, organized per environment.
  
- `rviz/`  
  Saved RViz configuration used for the tested Nav2 workflow:
  - `nav2.rviz`

- `scripts/`  
  Shared utilities and the double-LVLM workflow.

- `scripts/single_global/`  
  Single-global LVLM waypoint workflow.

## Core idea

These experiments separate navigation into two layers:

1. **Nav2 execution layer**  
   Handles localization, planning, control, and action execution.

2. **LVLM planning layer**  
   Interprets visual and map-derived context and decides higher-level route behavior.

The repository focuses on navigation under **incomplete or misleading prior maps**, where the static map may not fully match the traversable scene.

## Environment switching

The repository uses symlinks so the active environment can be changed without rewriting commands.

Use:

```bash
cd /path/to/incomplete-map-navigation-lvlm
./scripts/set_current_env.sh <env_name> <true|test>
```

Examples:

```bash
./scripts/set_current_env.sh env_simple_maze true
./scripts/set_current_env.sh env_simple_maze test
./scripts/set_current_env.sh env_office true
./scripts/set_current_env.sh env_warehouse test
```

This updates:

- `maps/current`
- `notes/current`

You can verify the active environment with:

```bash
readlink -f maps/current
readlink -f notes/current
```

## Shared workflow pieces

### Launch Nav2

```bash
cd /path/to/incomplete-map-navigation-lvlm/scripts
./run_nav2.sh
```

`run_nav2.sh` launches Nav2 using:

- `maps/current/map.yaml`
- `config/nav2_params_global_updates_on.yaml`
- `rviz/nav2.rviz`

At the moment, this script defaults to the **global-updates-on** configuration.

### Compute oracle path

```bash
python3 /path/to/incomplete-map-navigation-lvlm/scripts/compute_oracle_path.py \
  --use-start \
  --start-x 0 \
  --start-y 0 \
  --goal-x 2.5 \
  --goal-y 14.0 \
  --out /path/to/incomplete-map-navigation-lvlm/notes/env_simple_maze/oracle/oracle_path.json
```

### Log executed trajectory

```bash
python3 /path/to/incomplete-map-navigation-lvlm/scripts/log_executed_traj_tf.py \
  --out /path/to/incomplete-map-navigation-lvlm/notes/current/runs/example_mode/example_run/traj.csv \
  --hz 10.0 \
  --parent map \
  --child base_link
```

### Generate overlay and metrics

```bash
python3 /path/to/incomplete-map-navigation-lvlm/scripts/make_overlay.py \
  --map-yaml /path/to/incomplete-map-navigation-lvlm/maps/env_simple_maze/true/map.yaml \
  --oracle-json /path/to/incomplete-map-navigation-lvlm/notes/env_simple_maze/oracle/oracle_path.json \
  --traj-csv /path/to/incomplete-map-navigation-lvlm/notes/current/runs/example_mode/example_run/traj.csv \
  --out-png /path/to/incomplete-map-navigation-lvlm/notes/current/runs/example_mode/example_run/overlay.png \
  --out-metrics /path/to/incomplete-map-navigation-lvlm/notes/current/runs/example_mode/example_run/metrics.json \
  --legend
```

### Generate combined overlays across multiple runs

```bash
REPO=/path/to/incomplete-map-navigation-lvlm

python3 "$REPO/scripts/make_multi_overlay.py" \
  --runs-dir "$REPO/notes/env_warehouse/runs/lvlm_global_updates_on" \
  --map-yaml "$REPO/maps/env_warehouse/test/map.yaml" \
  --oracle-json "$REPO/notes/env_warehouse/oracle/oracle_path.json" \
  --limit 5
```

## Experiment modes

The repo is organized around two major Nav2 map-update settings:

- **global updates ON**  
  The global costmap includes live obstacle updates during execution.

- **global updates OFF**  
  The global costmap relies on the static prior map and does not apply live obstacle updates in the global map.

These two settings are part of the experimental design.

## Double-LVLM workflow

The double-LVLM pipeline uses two planner roles:

- a **global planner** for macro route reasoning
- a **local execution supervisor** for short-horizon regional behavior

Main script:

```text
scripts/run_lvlm_loop_semantic_blocked_regions.py
```

Prompt files:

```text
scripts/lvlm_global_planner_prompt_phase_bias_v5.txt
scripts/lvlm_region_prompt_phase_bias_v5.txt
```

This workflow can auto-launch:

- `export_crops.py`
- `export_rgb_dual.py`
- `region_goal_picker_node.py`

### Short example: `env_simple_maze`

```bash
REPO=/path/to/incomplete-map-navigation-lvlm

cd "$REPO"
./scripts/set_current_env.sh env_simple_maze true

cd "$REPO/scripts"
./run_nav2.sh
```

```bash
mkdir -p "$REPO/notes/env_simple_maze/oracle"

python3 "$REPO/scripts/compute_oracle_path.py" \
  --use-start \
  --start-x 0 \
  --start-y 0 \
  --goal-x 2.5 \
  --goal-y 14.0 \
  --out "$REPO/notes/env_simple_maze/oracle/oracle_path.json"
```

Then switch to the test map, relaunch Nav2, start the logger, and run:

```bash
python3 "$REPO/scripts/run_lvlm_loop_semantic_blocked_regions.py" \
  --notes-root "$REPO/notes/current" \
  --scripts-root "$REPO/scripts" \
  --goal-x 2.5 \
  --goal-y 14.0 \
  --planner-mode startup_and_replan \
  --planner-model gpt-5.4 \
  --local-model gpt-5.4 \
  --planner-prompt-file "$REPO/scripts/lvlm_global_planner_prompt_phase_bias_v5.txt" \
  --local-prompt-file "$REPO/scripts/lvlm_region_prompt_phase_bias_v5.txt" \
  --tick-hz 0.2 \
  --tf-wait 1.0 \
  --run-label lvlm_global_updates_on_sample \
  --active-goal-near-thresh 1.0 \
  --final-goal-reached-thresh 0.5 \
  --periodic-global-recheck-s 10.0 \
  --stop-on-error \
  --launch-crops \
  --launch-rgb \
  --launch-picker
```

After the run, generate overlays and metrics with `make_overlay.py`.

## Single-global LVLM workflow

The single-global workflow does not use regional box requests.

Instead, it:

- builds a full global planning image
- asks the model for one of:
  - `keep_plan`
  - `replace_plan`
  - `ratify_goal_reached`
- converts returned waypoint pixels into map-frame goals
- sends those goals to Nav2 sequentially
- maintains queue and planner state under `notes/current/state/`

Main files:

```text
scripts/single_global/common.py
scripts/single_global/prepare_single_global_tick.py
scripts/single_global/run_single_global_waypoint_loop.py
scripts/single_global/lvlm_single_global_waypoint_prompt.txt
```

### Short example

```bash
REPO=/path/to/incomplete-map-navigation-lvlm

cd "$REPO"
source /opt/ros/jazzy/setup.bash

python3 "$REPO/scripts/single_global/run_single_global_waypoint_loop.py" \
  --goal-x -15.75 \
  --goal-y 21.0 \
  --run-label lvlm_single_global_waypoint_updates_on \
  --launch-crops \
  --launch-rgb
```

## Runtime artifacts

This repository intentionally keeps the runtime folder skeleton because the scripts expect it.

Typical structure:

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

Important generated outputs include:

- `notes/current/exports/latest/`
- `notes/current/ticks/latest/`
- `notes/current/state/`
- `notes/current/runs/<MODE>/<timestamp>/`

## Python environment note

Activate an appropriate Python environment before running the LVLM scripts.

The documentation intentionally does **not** assume a machine-specific virtual environment name or path.

## Known limitations

- This is research code and reflects an experimental workflow.
- `run_nav2.sh` currently defaults to the **global-updates-on** Nav2 params file.
- Empty runtime folders are intentional and should not be removed without understanding script expectations.
- The repo is organized for reproducibility of the published experiments, not as a general-purpose robotics package.

## Documentation

The current `docs/` folder contains working notes from the project. A cleaned documentation set can be added later if desired.

## License

This project is licensed under the terms of the license in the `LICENSE` file.

## Citation

Citation information will be added after publication.


