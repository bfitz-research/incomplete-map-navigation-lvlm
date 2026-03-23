# Docs index — updated 2026-03-11 (post-Phase-4, Phase-5 in progress)

This set replaces the older Phase-4-only docs with the current picture from the recent work:

- Phase 4 plumbing is effectively complete.
- Phase 5 is in progress.
- The current recommended LVLM baseline is **GPT-5.4 local execution** with **semantic global replanning available on demand**.
- The earlier **anchor-region experiment** remains documented as part of project history, but it is no longer the default recommended operating mode.
- There is an open **active_goal visibility / state-sync bug** that should be debugged before drawing strong conclusions from behavior quality.

## Recommended read order for a new session
1. `02_high_level_goals.md`
2. `15_phase4_complete_phase5_in_progress.md`
3. `01_folder_layout_and_switches.md`
4. `03_how_to_run_overview.md`
5. `12_prepare_tick_and_payload_flow.md`
6. `13_lvlm_tick_payload_schema.md`
7. `16_run_lvlm_loop_and_planner_modes.md`
8. `17_known_issues_active_goal_visibility.md`
9. `LVLM_prompt_notes_TEMP.md`

## Core architecture / status
- `01_folder_layout_and_switches.md`
- `02_high_level_goals.md`
- `03_how_to_run_overview.md`
- `15_phase4_complete_phase5_in_progress.md`
- `16_run_lvlm_loop_and_planner_modes.md`
- `17_known_issues_active_goal_visibility.md`

## Office example operational docs (baseline Nav2 + metrics)
- `04_office_launch_nav2.md`
- `05_office_switch_maps.md`
- `06_office_oracle_compute_and_publish.md`
- `07_office_executed_trajectory_log_and_publish.md`
- `08_overlay_and_metrics.md`
- `09_running_10_trials_per_mode.md`

## LVLM plumbing / helper docs
- `10_exporter_rate_notes.md`
- `11_region_goal_picker_and_deprecated_cli.md`
- `12_prepare_tick_and_payload_flow.md`
- `13_lvlm_tick_payload_schema.md`
- `14_phase5_current_workflow_runbook.md`
- `LVLM_prompt_notes_TEMP.md`

## What was intentionally changed from the old doc set
- Old references to `write_tick_payload.py` as the authoritative LVLM path were replaced with `prepare_tick_payload_once.py`.
- Old references to `overlay_vis_node.py` as part of the authoritative model path were downgraded to optional/debug-only.
- Old docs that treated Phase 5 as “next” were updated to reflect that Phase 5 is now underway.
- New docs were added for:
  - planner modes / `run_lvlm_loop.py`
  - the current no-anchor / on-demand-replanning baseline
  - the open `active_goal` visualization bug and recommended debug procedure
