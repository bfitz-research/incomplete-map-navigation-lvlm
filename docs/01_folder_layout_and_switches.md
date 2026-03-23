# Folder layout and what changes between configs

Project root: `~/table_nav2/`

## Canonical layout (current working picture)

```text
table_nav2/
  config/
    nav2_params_global_updates_off.yaml
    nav2_params_global_updates_on.yaml
    nav2_params_lie.yaml            # legacy / older experiments
    nav2_params.yaml                # legacy / older experiments
  launch/
    nav2_bringup.launch.py
  maps/
    env_office/
      true/   map.yaml, map.png
      test/   map.yaml, map.png
    current -> symlink to whichever map folder is active
  notes/
    env_office/
      exports/
        latest/
          local_costmap_crop.png
          global_costmap_crop.png
          static_crop.png
          meta.json
          global_costmap.png
          global_costmap.png.txt
          rgb_left.png
          rgb_right.png
          local_vis.png
          global_crop_vis.png
          global_full_vis.png
          vis_meta.json
      ticks/
        latest/
          tick.json
      state/
        markers.json
        global_plan.json            # present only when planner mode uses / stores one
      oracle/
        oracle_path.json
      runs/
        <MODE>/
          <timestamp>/
            traj.csv
            overlay.png
            metrics.json
    current -> symlink to whichever env under notes/ is active
  rviz/
    nav2.rviz
  scripts/
    run_nav2.sh
    export_crops.py
    export_global_costmap.py        # may still exist on some branches
    export_rgb.py                   # older single-camera helper
    export_rgb_dual.py
    overlay_vis_node.py             # optional live/debug preview only
    prepare_tick_payload_once.py    # authoritative one-shot payload builder
    write_tick_payload.py           # legacy/background helper; not authoritative now
    region_goal_picker_node.py
    pick_goal_from_region.py        # deprecated CLI helper
    run_lvlm_loop.py
    lvlm_global_planner_prompt.txt
    lvlm_region_prompt.txt
    (oracle/metrics helpers)
  docs/
    (these docs)
```

## Important conventions

### 1) `maps/current`
Nav2 and map-dependent scripts should read through:

```bash
~/table_nav2/maps/current
```

That symlink points to the active map folder for the session.

### 2) `notes/current`
Exporters, payload builders, overlay helpers, planner state, and markers should read/write through:

```bash
~/table_nav2/notes/current
```

That symlink points to the active environment’s notes folder.

This is what keeps the runtime environment-agnostic when you later scale beyond the office environment.

## What you change between experiment configs

### A) Map selection (TRUE vs TEST)
Use the `maps/current` symlink.

**TEST map (normal experiments):**
```bash
ln -sfn ~/table_nav2/maps/env_office/test ~/table_nav2/maps/current
readlink -f ~/table_nav2/maps/current
```

**TRUE map (oracle generation):**
```bash
ln -sfn ~/table_nav2/maps/env_office/true ~/table_nav2/maps/current
readlink -f ~/table_nav2/maps/current
```

After switching maps, restart Nav2:
```bash
cd ~/table_nav2/scripts
./run_nav2.sh
```

### B) Active environment notes root
Use the `notes/current` symlink:

```bash
ln -sfn ~/table_nav2/notes/env_office ~/table_nav2/notes/current
mkdir -p ~/table_nav2/notes/current/exports/latest
mkdir -p ~/table_nav2/notes/current/ticks/latest
mkdir -p ~/table_nav2/notes/current/state
readlink -f ~/table_nav2/notes/current
```

### C) Global obstacle updates (OFF vs ON)
In `scripts/run_nav2.sh`, set the single active params file to one of:

- `config/nav2_params_global_updates_off.yaml`
- `config/nav2_params_global_updates_on.yaml`

That selects the global obstacle-update condition for the run.

### D) Run folders (4 experiment modes)
Create once under `notes/<env>/runs/`:

1. `nav2_global_updates_off/`
2. `nav2_global_updates_on/`
3. `lvlm_global_updates_off/`
4. `lvlm_global_updates_on/`

Each trial then goes into:

```text
notes/<env>/runs/<MODE>/<timestamp>/
```

with:
- `traj.csv`
- `overlay.png`
- `metrics.json`

## Current script roles

### Authoritative runtime path for LVLM calls
These are the scripts the current recommended LVLM architecture is built around:

- `export_crops.py`
- `export_rgb_dual.py`
- `region_goal_picker_node.py`
- `prepare_tick_payload_once.py`
- `run_lvlm_loop.py`
- `lvlm_global_planner_prompt.txt`
- `lvlm_region_prompt.txt`

### Optional/debug-only helpers
- `overlay_vis_node.py`
- `write_tick_payload.py`
- `pick_goal_from_region.py`

### Important current-state note
Some older branches and docs assumed a continuous background tick-writer or a separate live overlay node was part of the authoritative model path. That is no longer the recommended architecture. The authoritative path is:

1. continuous exporters / goal picker running in background
2. `prepare_tick_payload_once.py` builds a complete snapshot on demand
3. `run_lvlm_loop.py` calls the model using that complete snapshot
