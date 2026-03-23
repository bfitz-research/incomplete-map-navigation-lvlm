# How to run (current quick overview)

This doc uses the **office** environment as the running example, but all runtime paths use the `maps/current` and `notes/current` symlinks so the same structure generalizes to later environments.

This document is intentionally split into:
- **baseline bringup**
- **LVLM bringup**
- **recommended current Phase-5 path**
- **debug path for the active-goal visibility issue**

---

## 0) One-time symlinks for the session

### Active map (office TEST map)
```bash
ln -sfn ~/table_nav2/maps/env_office/test ~/table_nav2/maps/current
```

### Active notes environment
```bash
ln -sfn ~/table_nav2/notes/env_office ~/table_nav2/notes/current
mkdir -p ~/table_nav2/notes/current/exports/latest
mkdir -p ~/table_nav2/notes/current/ticks/latest
mkdir -p ~/table_nav2/notes/current/state
```

If you need the TRUE map later (oracle generation), switch `maps/current` to `env_office/true`.

---

## 1) Start Nav2 + RViz
```bash
cd ~/table_nav2/scripts
./run_nav2.sh
```

Quick checks:
```bash
ros2 topic list | egrep '(^/map$|/amcl_pose|/local_costmap/costmap|/global_costmap/costmap|/scan|/clock)'
ros2 lifecycle get /planner_server
```

---

## 2) Start the continuous feeder nodes

### 2A) Map exporter(s)
The project has had more than one exporter arrangement across branches.

#### Last known stable Phase-4 command from the uploaded exporter script
```bash
python3 ~/table_nav2/scripts/export_crops.py \
  --use-sim-time \
  --hz 2.0 \
  --crop-m 10 \
  --out-px 200 \
  --out-dir ~/table_nav2/notes/current/exports
```

That writes under `notes/current/exports/latest/`:
- `local_costmap_crop.png`
- `global_costmap_crop.png`
- `static_crop.png`
- `meta.json`

#### If your current branch still uses a separate full-global exporter
Run it too:
```bash
python3 ~/table_nav2/scripts/export_global_costmap.py \
  --use-sim-time \
  --hz 1.0 \
  --out-dir ~/table_nav2/notes/current/exports
```

This writes:
- `global_costmap.png`
- `global_costmap.png.txt`

### 2B) RGB exporter
Last known dual-RGB command from the uploaded script:
```bash
python3 ~/table_nav2/scripts/export_rgb_dual.py \
  --use-sim-time \
  --hz 2.0 \
  --out-dir ~/table_nav2/notes/current/exports
```

This writes:
- `rgb_left.png`
- `rgb_right.png`

### 2C) Region goal picker
```bash
python3 ~/table_nav2/scripts/region_goal_picker_node.py
```

This is the persistent region-request listener and Nav2 actuator.

---

## 3) Build a complete payload on demand
This is the authoritative pre-model step.

```bash
python3 ~/table_nav2/scripts/prepare_tick_payload_once.py
```

What it does:
1. reads latest raw exports
2. renders / refreshes vis images
3. writes `notes/current/ticks/latest/tick.json`
4. exits

If it succeeds, you have a complete payload ready for the model call.

---

## 4) Current recommended LVLM run mode
The current recommended baseline is:
- **local model: GPT-5.4**
- **planner mode: on_demand**
- **anchors off / not part of the default baseline**
- **ultimate goal primary**

Current example command:

```bash
source ~/venvs/ros_yolo/bin/activate
export OPENAI_API_KEY='your_api_key_here'

python3 ~/table_nav2/scripts/run_lvlm_loop.py \
  --notes-root ~/table_nav2/notes/current \
  --scripts-root ~/table_nav2/scripts \
  --goal-x GOAL_X \
  --goal-y GOAL_Y \
  --planner-mode on_demand \
  --planner-model gpt-5.4 \
  --local-model gpt-5.4 \
  --planner-prompt-file ~/table_nav2/scripts/lvlm_global_planner_prompt.txt \
  --local-prompt-file ~/table_nav2/scripts/lvlm_region_prompt.txt \
  --tick-hz 0.2 \
  --tf-wait 1.0 \
  --run-label office_lvlm_g54_on_demand \
  --active-goal-near-thresh 1.0 \
  --final-goal-reached-thresh 0.5 \
  --stop-on-error
```

For a shorter test first, add:
```bash
--max-cycles 8
```

---

## 5) Manual region-request sanity test
This is still the fastest way to verify the region picker / active-goal path independent of the LVLM loop.

```bash
ros2 topic pub --once /region_request std_msgs/msg/Int32MultiArray "{data: [110, 60, 170, 120, 80, 1]}"
```

Then check:
```bash
cat ~/table_nav2/notes/current/state/markers.json
```

If your branch updates marker state correctly, you should see `active_goal` become non-null.

Then rebuild the payload:
```bash
python3 ~/table_nav2/scripts/prepare_tick_payload_once.py
```

---

## 6) Recommended watcher windows during debugging

### Watch `markers.json`
```bash
watch -n 0.2 'echo "=== markers.json ==="; cat ~/table_nav2/notes/current/state/markers.json'
```

### Watch `tick.json` mission state
```bash
watch -n 0.2 'echo "=== tick mission_state ==="; jq ".mission_state" ~/table_nav2/notes/current/ticks/latest/tick.json'
```

### Watch file update times
```bash
watch -n 0.2 'echo "=== mtimes ==="; stat -c "markers %y" ~/table_nav2/notes/current/state/markers.json; stat -c "tick    %y" ~/table_nav2/notes/current/ticks/latest/tick.json'
```

These are especially important right now because the current open bug is that `active_goal` sometimes appears to vanish from overlays even while the goal picker reports continued progress.

---

## 7) Current split of responsibilities

### Continuous background nodes
- map exporter(s)
- RGB exporter
- region goal picker

### One-shot pre-model step
- `prepare_tick_payload_once.py`

### Orchestrator / model caller
- `run_lvlm_loop.py`

### Optional / debug only
- `overlay_vis_node.py`
- `write_tick_payload.py`

---

## 8) Important current recommendation
Do **not** interpret messy behavior primarily as a planning problem until the `active_goal` visibility/state path is trusted.

Current best practice:
1. verify region picker / markers / tick state agree
2. verify overlays actually reflect that state
3. only then compare local-only vs replanning behavior
