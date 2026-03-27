# Single-Global LVLM Runbook

This document gives a complete example run for the single-global waypoint workflow.

## Example configuration

- repository root: `REPO=/path/to/incomplete-map-navigation-lvlm`
- environment: `env_simple_maze`
- goal: `(2.5, 14.0)`

Use **3 terminals**:

- Terminal 1: Nav2
- Terminal 2: oracle / logger / postprocessing
- Terminal 3: single-global runner

## Phase A: generate the oracle on the true map

### Terminal 2

Set the active environment to the true map:

```bash
REPO=/path/to/incomplete-map-navigation-lvlm

cd "$REPO"
./scripts/set_current_env.sh env_simple_maze true
```

Verify:

```bash
readlink -f "$REPO/maps/current"
readlink -f "$REPO/notes/current"
```

### Terminal 1

Launch Nav2:

```bash
cd "$REPO/scripts"
./run_nav2.sh
```

### Terminal 2

Compute the oracle path:

```bash
mkdir -p "$REPO/notes/env_simple_maze/oracle"

python3 "$REPO/scripts/compute_oracle_path.py"   --use-start   --start-x 0   --start-y 0   --goal-x 2.5   --goal-y 14.0   --out "$REPO/notes/env_simple_maze/oracle/oracle_path.json"
```

### Terminal 1

Stop Nav2 with `Ctrl+C`.

## Phase B: switch to the test map for the actual run

### Terminal 2

Set the active environment to the test map:

```bash
cd "$REPO"
./scripts/set_current_env.sh env_simple_maze test
```

Create the runtime `latest` folders:

```bash
mkdir -p "$REPO/notes/current/exports/latest"
mkdir -p "$REPO/notes/current/ticks/latest"
```

### Terminal 1

Relaunch Nav2:

```bash
cd "$REPO/scripts"
./run_nav2.sh
```

## Phase C: run the actual single-global trial

### Terminal 2

Start the trajectory logger:

```bash
MODE=lvlm_single_global_updates_on_sample
RUNSTAMP=$(date +%Y%m%d_%H%M%S)
LOGGER_DIR="$REPO/notes/current/runs/$MODE/${RUNSTAMP}_logger"

mkdir -p "$LOGGER_DIR"

python3 "$REPO/scripts/log_executed_traj_tf.py"   --out "$LOGGER_DIR/traj.csv"   --hz 10.0   --parent map   --child base_link
```

### Terminal 3

Activate your Python environment, export your OpenAI API key, and run:

```bash
source /opt/ros/jazzy/setup.bash

python3 "$REPO/scripts/single_global/run_single_global_waypoint_loop.py"   --goal-x 2.5   --goal-y 14.0   --run-label lvlm_single_global_updates_on_sample   --launch-crops   --launch-rgb   --progress-stall-window-s 18   --safe-free-threshold 245   --safe-erosion-radius-px 8   --waypoint-repair-radius-px 25   --max-repaired-waypoints-per-plan 2   --rejection-radius-px 36   --rejection-replans 3
```

Let the run finish.

## Phase D: finalize the run

### Terminal 2

Stop the logger with `Ctrl+C`.

Then copy the logged trajectory into the created run folder:

```bash
MODE=lvlm_single_global_updates_on_sample

LATEST_RUN=$(find "$REPO/notes/current/runs/$MODE" -mindepth 1 -maxdepth 1 -type d ! -name '*_logger' | sort | tail -n1)
LATEST_LOGGER=$(find "$REPO/notes/current/runs/$MODE" -mindepth 1 -maxdepth 1 -type d -name '*_logger' | sort | tail -n1)

cp "$LATEST_LOGGER/traj.csv" "$LATEST_RUN/traj.csv"
```

Generate overlay and metrics using the true map as the background:

```bash
python3 "$REPO/scripts/make_overlay.py"   --map-yaml "$REPO/maps/env_simple_maze/true/map.yaml"   --oracle-json "$REPO/notes/env_simple_maze/oracle/oracle_path.json"   --traj-csv "$LATEST_RUN/traj.csv"   --out-png "$LATEST_RUN/overlay.png"   --out-metrics "$LATEST_RUN/metrics.json"   --legend
```

## How this workflow differs from the double-LVLM setup

The single-global workflow:

- does **not** use regional box selection
- does **not** use `region_goal_picker_node.py`
- plans directly on the full global visualization
- returns one of:
  - `keep_plan`
  - `replace_plan`
  - `ratify_goal_reached`
- converts selected waypoint pixels into map-frame goals and sends them sequentially through Nav2

## Notes

- The run folder is created by `run_single_global_waypoint_loop.py` under:
  - `notes/current/runs/<run-label>/<timestamp>/`
- The separate `*_logger` folder is only for the trajectory logger.
- The final overlay and metrics are usually most interpretable when drawn on the **true** map.
