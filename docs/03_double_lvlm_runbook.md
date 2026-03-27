# Double-LVLM Runbook

This document gives a complete example run for the double-LVLM workflow.

## Example configuration

- repository root: `REPO=/path/to/incomplete-map-navigation-lvlm`
- environment: `env_simple_maze`
- goal: `(2.5, 14.0)`

Use **4 terminals**:

- Terminal 1: Nav2
- Terminal 2: oracle / logger / postprocessing
- Terminal 3: oracle publisher first, then optional live global-plan watcher
- Terminal 4: double-LVLM runner

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

### Terminal 3

Optional: publish the oracle path to RViz:

```bash
python3 "$REPO/scripts/publish_oracle_path.py"   --in "$REPO/notes/env_simple_maze/oracle/oracle_path.json"
```

This terminal is useful for inspecting the live semantic output of the global LVLM planner during the run.

In RViz:

- add a **Path** display
- use topic `/oracle_path`

When finished viewing it, stop the publisher with `Ctrl+C`.

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

## Phase C: run the actual double-LVLM trial

### Terminal 2

Start the trajectory logger:

```bash
MODE=lvlm_global_updates_on_sample
RUNSTAMP=$(date +%Y%m%d_%H%M%S)
LOGGER_DIR="$REPO/notes/current/runs/$MODE/${RUNSTAMP}_logger"

mkdir -p "$LOGGER_DIR"

python3 "$REPO/scripts/log_executed_traj_tf.py"   --out "$LOGGER_DIR/traj.csv"   --hz 10.0   --parent map   --child base_link
```

### Terminal 3

Optional: watch the live global semantic plan:

```bash
watch -n 1 cat "$REPO/notes/current/state/global_plan.json"
```

If `jq` is installed:

```bash
watch -n 1 "jq . '$REPO/notes/current/state/global_plan.json'"
```

### Terminal 4

Activate your Python environment, export your OpenAI API key, and run:

```bash
python3 "$REPO/scripts/run_lvlm_loop_semantic_blocked_regions.py"   --notes-root "$REPO/notes/current"   --scripts-root "$REPO/scripts"   --goal-x 2.5   --goal-y 14.0   --planner-mode startup_and_replan   --planner-model gpt-5.4   --local-model gpt-5.4   --planner-prompt-file "$REPO/scripts/lvlm_global_planner_prompt_phase_bias_v5.txt"   --local-prompt-file "$REPO/scripts/lvlm_region_prompt_phase_bias_v5.txt"   --tick-hz 0.2   --tf-wait 1.0   --run-label lvlm_global_updates_on_sample   --active-goal-near-thresh 1.0   --final-goal-reached-thresh 0.5   --periodic-global-recheck-s 10.0   --stop-on-error   --launch-crops   --launch-rgb   --launch-picker
```

Let the run finish.

## Phase D: finalize the run

### Terminal 2

Stop the logger with `Ctrl+C`.

Then copy the logged trajectory into the created run folder:

```bash
MODE=lvlm_global_updates_on_sample

LATEST_RUN=$(find "$REPO/notes/current/runs/$MODE" -mindepth 1 -maxdepth 1 -type d ! -name '*_logger' | sort | tail -n1)
LATEST_LOGGER=$(find "$REPO/notes/current/runs/$MODE" -mindepth 1 -maxdepth 1 -type d -name '*_logger' | sort | tail -n1)

cp "$LATEST_LOGGER/traj.csv" "$LATEST_RUN/traj.csv"
```

Generate overlay and metrics using the true map as the background:

```bash
python3 "$REPO/scripts/make_overlay.py"   --map-yaml "$REPO/maps/env_simple_maze/true/map.yaml"   --oracle-json "$REPO/notes/env_simple_maze/oracle/oracle_path.json"   --traj-csv "$LATEST_RUN/traj.csv"   --out-png "$LATEST_RUN/overlay.png"   --out-metrics "$LATEST_RUN/metrics.json"   --legend
```

## Notes

- The run folder is created by `run_lvlm_loop_semantic_blocked_regions.py` under:
  - `notes/current/runs/<run-label>/<timestamp>/`
- The separate `*_logger` folder is only for the trajectory logger.
- The final overlay and metrics are usually most interpretable when drawn on the **true** map.
- If overlays or run-state visuals look inconsistent, check whether `notes/current/state/markers.json` and `notes/current/state/global_plan.json` are updating as expected. Some confusing visual results can reflect state-visibility or synchronization issues rather than planner quality alone.
- Common mode bucket names used across experiments include:
  - `nav2_global_updates_off`
  - `nav2_global_updates_on`
  - `lvlm_global_updates_off`
  - `lvlm_global_updates_on`
