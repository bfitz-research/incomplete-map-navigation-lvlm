# 19. Double-LVLM Prompt Bundles and Run Workflow

This document explains how to run the current **double-LVLM pipeline** with the two prompt bundles we have been testing, how to collect trajectory data, and how to generate overlays and metrics afterward.

---

## 1. Purpose

The current double-LVLM pipeline is:

- a **global LVLM planner** that produces macro route guidance
- a **local LVLM planner** that selects local rectangular regions
- `region_goal_picker_node.py` converts those regions into actual Nav2 goals
- Nav2 executes those goals
- a separate trajectory logger records the executed path
- `make_overlay.py` compares the executed path against the oracle path

We currently have **two prompt bundles** for this system:

1. **phase-bias v5**
   - stronger macro-route reasoning
   - generally the better baseline so far

2. **directional-hints v6**
   - more structured short-horizon hinting from global to local
   - experimental variant

These should be treated as **separate bundles**, not mixed casually.

---

## 2. Important files

### Core scripts
- `~/table_nav2/scripts/run_nav2.sh`
- `~/table_nav2/scripts/run_lvlm_loop_semantic_blocked_regions.py`
- `~/table_nav2/scripts/run_lvlm_loop_directional_hints_v6.py`
- `~/table_nav2/scripts/log_executed_traj_tf.py`
- `~/table_nav2/scripts/make_overlay.py`
- `~/table_nav2/scripts/compute_oracle_path.py`
- `~/table_nav2/scripts/publish_oracle_path.py`
- `~/table_nav2/scripts/publish_executed_path.py`

### Prompt bundles

#### Phase-bias v5
- `~/table_nav2/scripts/lvlm_global_planner_prompt_phase_bias_v5.txt`
- `~/table_nav2/scripts/lvlm_region_prompt_phase_bias_v5.txt`

#### Directional-hints v6
- `~/table_nav2/scripts/lvlm_global_planner_prompt_directional_hints_v6.txt`
- `~/table_nav2/scripts/lvlm_region_prompt_directional_hints_v6.txt`

---

## 3. Map switching

### True map for oracle generation
```bash
ln -sfn ~/table_nav2/maps/env_office/true ~/table_nav2/maps/current
readlink -f ~/table_nav2/maps/current
