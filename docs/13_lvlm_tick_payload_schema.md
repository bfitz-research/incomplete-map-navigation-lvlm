# LVLM tick payload schema (current practical view)

This doc updates the old `tick_v1` description to match the more recent Phase-5 usage pattern.

The exact field set can vary somewhat by branch, but the important current ideas are stable:

- the tick is built on demand immediately before a model call
- it references the current vis images and RGB images
- it includes mission state
- it may include planner/global-plan state
- it should tell you whether the payload is actually ready

---

## Canonical path
```text
~/table_nav2/notes/current/ticks/latest/tick.json
```

---

## Important top-level concepts

### `schema_version`
Identifies the tick format.

Recent branches have used newer schema names than the old `tick_v1` doc.
Do not assume the old name is still correct.

### `wall_time_iso`
Wall-clock timestamp for logs.

### `notes_root`
Resolved notes root used for this payload.

### `export`
Carries forward the latest crop/export metadata such as:
- export tick / timestamp
- render pose used by exporter
- crop bounds and pixel geometry

### `render`
Describes the vis-image render step and the current output image paths.

### `robot_state`
Best-effort robot pose in map frame.
This may come from TF or, when TF is unavailable, a fallback path may be noted in warnings.

### `mission_state`
Current mission fields, typically:
- `ultimate_goal`
- `active_goal`
- `pois`

### `global_plan_state` (optional / branch-dependent)
When planner state exists, this may include whether a global plan is present and any current planner-related state.

### `lvlm_inputs`
The image paths the model is actually expected to use.
This is one of the most important sections for the current architecture.

### `exports`
Absolute paths to the raw/vis artifacts.

### `exports_present`
Boolean presence map for those artifacts.

### `artifact_age_s`
Helpful for checking freshness / race conditions.

### `warnings`
Any warnings, fallbacks, or state-quality concerns.

---

## Current model-facing images
The current intended authoritative image set is:

### Primary local map
```text
local_vis.png
```
This is the interactive local selection domain.

### Primary global crop
```text
global_crop_vis.png
```
This is the zoomed-in regional view of the full global map around the robot. It covers the same spatial neighborhood as the local crop but reflects the broader global/static planning context.

### Primary global full
```text
global_full_vis.png
```
This is the full global context view.

### Primary RGB
```text
rgb_left.png
```
Default RGB image.

### Optional right RGB
```text
rgb_right.png
```
Optional / future on-demand image.

---

## Mission-state meaning

### `ultimate_goal`
The fixed true mission objective for the run.

### `active_goal`
The current subgoal Nav2 is following.

This field is currently especially important because the project is debugging whether the active-goal file state and overlay state always remain consistent.

### `pois`
Reserved for future points of interest.

---

## Current readiness interpretation
Some recent branches include explicit fields like:
- `ready_for_lvlm`
- `not_ready_reasons`

A model call should proceed only when the payload is actually complete enough for the current policy.

At minimum, current authoritative payloads are expected to have:
- local vis image
- global crop vis image
- global full vis image
- left RGB
- crop/export metadata
- mission state

---

## Pixel semantics still come from crop geometry
Even though the model sees vis images, the local returned region is still interpreted using the same crop geometry:

- `x_map = min_x + (u + 0.5) * meters_per_px`
- `y_map = max_y - (v + 0.5) * meters_per_px`

So the overlay styling changes the image semantics, not the underlying local pixel-to-map conversion.
