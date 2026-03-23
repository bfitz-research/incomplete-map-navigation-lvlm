# `run_lvlm_loop.py` and planner modes

This doc explains the current intended role of `run_lvlm_loop.py` in the Phase-5 architecture.

## What `run_lvlm_loop.py` does
At a high level, it:
1. resets startup state for a fresh run
2. repeatedly builds a complete payload snapshot
3. calls the local model
4. interprets one of:
   - `monitor`
   - `set_region`
   - `request_global_replan`
5. publishes region requests when needed
6. optionally calls the global replanner if the local model asks for it

---

## Current recommended mode
The current recommended baseline mode is:

```text
planner_mode = on_demand
local_model  = gpt-5.4
```

Why:
- no mandatory startup planner
- no anchor requirement
- local model reasons directly from full global + global crop + local crop
- route-level replanning remains available if the local model asks for it

---

## Planner modes

### `startup_and_replan`
Use this if you want:
- an immediate startup planning pass
- later replanning calls as needed

This is closer to the earlier two-stage architecture.

### `on_demand`
Use this for the currently recommended baseline.

Behavior:
- startup planner is skipped
- local model starts immediately
- global replanner is called only if the local model returns `request_global_replan`

### `off`
Use this if you want to fully disable planner use.

Behavior:
- no startup planner
- no replanning calls
- any local `request_global_replan` is effectively ignored / handled as planner-off logic

---

## Current command template
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

For a shorter debug test, add:
```bash
--max-cycles 8
```

---

## Current intended model responsibilities

### Local model
The local model should:
- treat `ultimate_goal` as primary
- use full global + global crop to preserve strategic direction
- use local crop for immediate feasibility
- return `monitor` when the current subgoal still looks good
- return `set_region` when a new short-horizon subgoal is needed
- return `request_global_replan` only when route-level thinking is truly needed

### Global replanner
The global replanner should:
- produce semantic route guidance only
- not produce anchors or waypoint chains by default
- help with blocked/deceptive route interpretation
- remain optional rather than mandatory in every run

---

## Why the current baseline moved away from anchors
Anchor-based planning was tested, but recent runs suggested that:
- GPT-5.4 local execution may already be strong enough to use the global maps directly
- anchors sometimes became too strong a target and pulled behavior away from the actual final-goal objective

So the current recommendation is to first stabilize the no-anchor baseline and only reintroduce richer structure if the data shows it is actually needed.
