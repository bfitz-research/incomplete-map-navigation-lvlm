# Payload preparation and the current authoritative LVLM data path

This doc replaces the old framing that treated `write_tick_payload.py` as the main LVLM interface.

## Current authoritative path
The current recommended model-call path is:

1. continuous exporters / region picker keep raw state fresh
2. `prepare_tick_payload_once.py` builds a complete payload snapshot on demand
3. `run_lvlm_loop.py` reads that payload and performs the model call

This is the current authoritative sequence.

---

## Why this replaced the older timer-only approach
If overlays, RGB export, state writing, and tick writing all run on unrelated background timers, you can end up asking:
- did the overlays update yet?
- did the marker state make it into the vis images yet?
- did the latest RGB export get included?
- is the model seeing the newest package or a half-updated mix?

`prepare_tick_payload_once.py` exists to reduce that ambiguity.

---

## Script
- `~/table_nav2/scripts/prepare_tick_payload_once.py`

## Inputs it expects to already exist
Typical inputs include:
- `exports/latest/meta.json`
- `exports/latest/local_costmap_crop.png`
- `exports/latest/global_costmap_crop.png`
- `exports/latest/global_costmap.png` and sidecar (if your branch uses them)
- `exports/latest/rgb_left.png`
- optional `exports/latest/rgb_right.png`
- `state/markers.json`
- optional `state/global_plan.json`

## Outputs it writes / refreshes
Typical outputs include:
- `exports/latest/local_vis.png`
- `exports/latest/global_crop_vis.png`
- `exports/latest/global_full_vis.png`
- `exports/latest/vis_meta.json`
- `ticks/latest/tick.json`

---

## Recommended command
```bash
python3 ~/table_nav2/scripts/prepare_tick_payload_once.py
```

## Expected success output
Example:
```text
[prepare_tick_payload_once] OK: wrote complete payload -> /home/brian/table_nav2/notes/current/ticks/latest/tick.json
```

---

## Readiness interpretation
The resulting `tick.json` should include:
- a schema version
- payload references / mission state
- a readiness signal such as `ready_for_lvlm`
- any warnings if something was missing or a fallback path was used

The LVLM caller should only proceed when the payload is actually ready.

---

## Relationship to `write_tick_payload.py`
`write_tick_payload.py` is now best treated as:
- a legacy helper
- a background-debug utility
- or a convenience tool for continuous snapshots

It is **not** the recommended authoritative step immediately before a real model call anymore.

---

## Typical current call pattern
### Continuous background processes
- exporter(s)
- RGB exporter
- region goal picker

### Model-call sequence
1. `prepare_tick_payload_once.py`
2. read `tick.json`
3. call model
4. if model returns a new region, publish `/region_request`
5. repeat later

---

## Important current debugging use
Because the project currently has an `active_goal` visibility/state issue, this one-shot payload builder is also the cleanest place to test whether file-backed state made it into the latest payload.

Recommended checks after running it:
```bash
jq '.mission_state' ~/table_nav2/notes/current/ticks/latest/tick.json
jq '.global_plan_state?' ~/table_nav2/notes/current/ticks/latest/tick.json
jq '.' ~/table_nav2/notes/current/exports/latest/vis_meta.json
```
