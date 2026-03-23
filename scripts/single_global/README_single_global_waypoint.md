# Single Global Waypoint Planner (parallel architecture)

This folder contains a **new parallel LVLM architecture** for `table_nav2`.
It does **not** use the old local-region selection contract.

## What it does

- uses the **full global overlay** as the primary planning image
- uses **RGB** as scene-status evidence
- optionally includes the **local crop** for future global-updates-off experiments
- asks the LVLM for exactly one of:
  - `keep_plan`
  - `replace_plan`
  - `ratify_goal_reached`
- converts full-global overlay waypoint pixels into map-frame coordinates
- sends those coordinates to Nav2 one at a time
- maintains the remaining waypoint queue in:
  - `notes/current/state/single_global_waypoint_queue.json`

## New files

- `common.py`
- `prepare_single_global_tick.py`
- `run_single_global_waypoint_loop.py`
- `lvlm_single_global_waypoint_prompt.txt`

## Main runtime artifacts

Under `notes/current/exports/latest/`:
- `single_global_full_vis.png`
- `single_global_local_vis.png` (only when `--include-local-crop` is used)
- `single_global_vis_meta.json`

Under `notes/current/ticks/latest/`:
- `single_global_tick.json`

Under `notes/current/state/`:
- `single_global_waypoint_queue.json`
- `single_global_runner_state.json`
- `single_global_last_planner_response.json`

## Recommended first run mode

Start with **global costmap updates ON** and do **not** use blocked-region painting.

Suggested command pattern:

```bash
cd ~/table_nav2
source /opt/ros/jazzy/setup.bash
python3 scripts/single_global/run_single_global_waypoint_loop.py \
  --goal-x -15.75 \
  --goal-y 21.0 \
  --run-label lvlm_single_global_waypoint_updates_on \
  --launch-crops \
  --launch-rgb
```

If you already launched the exporters elsewhere, omit `--launch-crops` and `--launch-rgb`.

## Optional later mode

For a future global-updates-off experiment, add:

```bash
--include-local-crop
```

That will cause the prepared tick and planner image set to include the local crop overlay as supplemental evidence.

## Important note

This is intentionally a **fresh execution path**.
It does not call `region_goal_picker_node.py`, does not publish `/region_request`, and does not use local region boxes.


## Recent route rejection memory

This version adds a short-lived run-state file:

- `notes/current/state/recent_route_rejections.json`

It is written by the runner when:
- Nav2 rejects or fails a waypoint
- progress stalls near the active waypoint

It is then:
- included in the planner payload as `recent_route_rejections`
- drawn onto `single_global_full_vis.png` as magenta rejection regions
- decremented after each planner call
- automatically cleared on new run startup

This is intentionally temporary run-memory, not cross-run memory.


## Waypoint safety filter (v3)

This version adds deterministic waypoint validation against the raw `global_costmap.png` before any returned pixel waypoint is converted into a map-frame Nav2 goal.

New runner args:
- `--safe-free-threshold` (default `245`)
- `--safe-erosion-radius-px` (default `8`)
- `--waypoint-repair-radius-px` (default `25`)
- `--max-repaired-waypoints-per-plan` (default `2`)

Behavior:
- waypoints must land in bright traversable space on the raw global costmap
- the safe region is eroded inward to enforce clearance from black/dark inflation
- nearby unsafe waypoints may be repaired to the nearest safe pixel
- unrepaired unsafe waypoints cause that planner output to be rejected


## v4 prompt/contract update
- imports stricter map-color semantics from the older two-level prompts
- treats medium gray as not acceptable for waypoint placement, not merely less preferred
- explicitly prefers longer bright-white routes over shorter risky shortcuts
- reduces replace_plan size to 1..4 waypoints and explicitly allows 1-waypoint plans when only the next safe commitment is clear
