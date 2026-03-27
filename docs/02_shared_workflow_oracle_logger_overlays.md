# Shared Workflow: Oracle, Logger, and Overlays

This document covers the shared utilities used by both the double-LVLM and single-global workflows.

## Repository root

Assume:

```bash
REPO=/path/to/incomplete-map-navigation-lvlm
```

## 1. Launch Nav2

The standard helper is:

```bash
cd "$REPO/scripts"
./run_nav2.sh
```

Current behavior:

- uses `maps/current/map.yaml`
- uses `rviz/nav2.rviz`
- currently defaults to `config/nav2_params_global_updates_on.yaml`

If you want to reproduce the **global-updates-off** condition, use the alternate Nav2 parameter file instead of the default `run_nav2.sh` configuration.

## 2. Generate the oracle on the true map

Example for `env_simple_maze`:

```bash
cd "$REPO"
./scripts/set_current_env.sh env_simple_maze true
mkdir -p "$REPO/notes/env_simple_maze/oracle"
```

Launch Nav2 in one terminal, then compute the oracle in another:

```bash
python3 "$REPO/scripts/compute_oracle_path.py"   --use-start   --start-x 0   --start-y 0   --goal-x 2.5   --goal-y 14.0   --out "$REPO/notes/env_simple_maze/oracle/oracle_path.json"
```

## 2.5 Optional: capture a goal from RViz

If you want to pick a goal interactively in RViz, you can inspect the published goal pose.

In one terminal, run:

```bash
ros2 topic echo /goal_pose --once

## 3. Optional: publish the oracle path to RViz

```bash
python3 "$REPO/scripts/publish_oracle_path.py"   --in "$REPO/notes/env_simple_maze/oracle/oracle_path.json"
```

In RViz, add a **Path** display and set the topic to:

```text
/oracle_path
```

## 4. Switch to the test map for the actual run

```bash
cd "$REPO"
./scripts/set_current_env.sh env_simple_maze test

mkdir -p "$REPO/notes/current/exports/latest"
mkdir -p "$REPO/notes/current/ticks/latest"
```

## 5. Start trajectory logging

Example:

```bash
MODE=example_mode
RUNSTAMP=$(date +%Y%m%d_%H%M%S)
LOGGER_DIR="$REPO/notes/current/runs/$MODE/${RUNSTAMP}_logger"

mkdir -p "$LOGGER_DIR"

python3 "$REPO/scripts/log_executed_traj_tf.py"   --out "$LOGGER_DIR/traj.csv"   --hz 10.0   --parent map   --child base_link
```

## 5.5 Optional: send a goal from the CLI

Instead of using RViz 2D Goal Pose, you can send a Nav2 goal directly from the command line.

Example:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 14.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 6. Generate overlay and metrics for a single run


After the run is complete, make sure the final run folder has a `traj.csv`, then run:

```bash
python3 "$REPO/scripts/make_overlay.py"   --map-yaml "$REPO/maps/env_simple_maze/true/map.yaml"   --oracle-json "$REPO/notes/env_simple_maze/oracle/oracle_path.json"   --traj-csv "$REPO/notes/current/runs/example_mode/example_run/traj.csv"   --out-png "$REPO/notes/current/runs/example_mode/example_run/overlay.png"   --out-metrics "$REPO/notes/current/runs/example_mode/example_run/metrics.json"   --legend
```

### Why the true map is often the best overlay background

For paper figures and interpretation, the **true map** is often the most readable background for overlays because it reflects the actual environment geometry.

The **test map** can still be useful when you specifically want to show the incomplete prior map that the robot was given at runtime.

## 7. Generate a combined overlay across multiple runs

Example:

```bash
python3 "$REPO/scripts/make_multi_overlay.py"   --runs-dir "$REPO/notes/env_warehouse/runs/lvlm_global_updates_on"   --map-yaml "$REPO/maps/env_warehouse/test/map.yaml"   --oracle-json "$REPO/notes/env_warehouse/oracle/oracle_path.json"   --limit 5
```

## 8. Optional: publish the executed trajectory to RViz

```bash
python3 "$REPO/scripts/publish_executed_path.py"   --in "$REPO/notes/current/runs/example_mode/example_run/traj.csv"
```

## Common output locations

- oracle path:
  - `notes/<env>/oracle/oracle_path.json`

- logger output:
  - `notes/current/runs/<MODE>/<timestamp>_logger/traj.csv`

- final run artifacts:
  - `notes/current/runs/<MODE>/<timestamp>/traj.csv`
  - `notes/current/runs/<MODE>/<timestamp>/overlay.png`
  - `notes/current/runs/<MODE>/<timestamp>/metrics.json`
