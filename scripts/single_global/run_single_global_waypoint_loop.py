#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
import sys
import time
from copy import deepcopy
from datetime import datetime
from functools import partial
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from openai import OpenAI

from common import (
    GlobalImageInfo,
    active_waypoint,
    atomic_write_json,
    build_safe_waypoint_mask,
    ensure_dir,
    euclidean_xy,
    default_recent_route_rejections,
    decrement_recent_route_rejections,
    image_file_to_data_url,
    iso_now_local,
    is_safe_waypoint_pixel,
    load_json,
    nearest_safe_waypoint_pixel,
    prune_recent_route_rejections,
    remaining_waypoints,
    summarize_waypoints_for_payload,
    waypoint_mask_debug,
    wrap_pi,
    yaw_from_quat,
)


def compute_relative_geometry(robot_state: Optional[Dict[str, Any]], target_xy: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not robot_state or not target_xy:
        return None
    try:
        rx = float(robot_state["x_map"])
        ry = float(robot_state["y_map"])
        ryaw = float(robot_state["yaw_rad"])
        tx = float(target_xy.get("x", target_xy.get("x_map")))
        ty = float(target_xy.get("y", target_xy.get("y_map")))
    except Exception:
        return None
    dx = tx - rx
    dy = ty - ry
    dist = (dx * dx + dy * dy) ** 0.5
    bearing_global = 0.0 if dist < 1e-9 else __import__("math").atan2(dy, dx)
    bearing_relative = wrap_pi(bearing_global - ryaw)
    return {
        "x_map": tx,
        "y_map": ty,
        "dx_m": round(dx, 4),
        "dy_m": round(dy, 4),
        "distance_m": round(dist, 4),
        "bearing_global_deg": round(__import__("math").degrees(bearing_global), 2),
        "bearing_relative_deg": round(__import__("math").degrees(bearing_relative), 2),
    }


class SingleGlobalWaypointRunner(Node):
    def __init__(self, args):
        super().__init__("single_global_waypoint_runner")
        self.args = args
        self.notes_root = Path(args.notes_root).expanduser().resolve()
        self.this_dir = Path(__file__).resolve().parent
        self.state_dir = self.notes_root / "state"
        self.ticks_dir = self.notes_root / "ticks" / "latest"
        self.runs_root = self.notes_root / "runs" / args.run_label
        ensure_dir(self.state_dir)
        ensure_dir(self.ticks_dir)
        ensure_dir(self.runs_root)

        self.run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.runs_root / self.run_ts
        ensure_dir(self.run_dir)
        self.cycle_log_path = self.run_dir / "single_global_cycles.jsonl"

        self.markers_path = self.state_dir / "markers.json"
        self.queue_path = self.state_dir / "single_global_waypoint_queue.json"
        self.runner_state_path = self.state_dir / "single_global_runner_state.json"
        self.planner_state_path = self.state_dir / "single_global_last_planner_response.json"
        self.rejections_path = self.state_dir / "recent_route_rejections.json"
        self.tick_path = self.ticks_dir / "single_global_tick.json"
        self.prepare_script = self.this_dir / "prepare_single_global_tick.py"

        api_key = os.environ.get("OPENAI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not set in the environment")
        self.client = OpenAI(api_key=api_key)
        prompt_path = Path(args.prompt_file).expanduser().resolve()
        self.prompt_text = prompt_path.read_text(encoding="utf-8")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(self, NavigateToPose, args.action_name)

        self.helper_procs: List[Tuple[str, subprocess.Popen, Any]] = []
        self.latest_goal_seq = 0
        self.active_goal_handle = None
        self.active_goal_seq: Optional[int] = None
        self.pending_replan: Optional[Tuple[str, str]] = None
        self.goal_ratified = False
        self.stop_reason: Optional[str] = None
        self.last_planner_call_wall_time: Optional[float] = None
        self.last_tick: Optional[Dict[str, Any]] = None
        self.last_nav_event: Optional[Dict[str, Any]] = None
        self.goal_progress_reference_distance_m: Optional[float] = None
        self.last_goal_progress_wall_time: Optional[float] = None
        self.last_queue_replace_wall_time: Optional[float] = None
        self._safe_mask_cache: Optional[Dict[str, Any]] = None

    def _helper_log_dir(self) -> Path:
        p = self.run_dir / "helper_logs"
        ensure_dir(p)
        return p

    def _start_helper(self, name: str, command: str):
        log_path = self._helper_log_dir() / f"{name}.log"
        log_f = open(log_path, "a", encoding="utf-8")
        proc = subprocess.Popen(["bash", "-lc", command], stdout=log_f, stderr=subprocess.STDOUT)
        self.helper_procs.append((name, proc, log_f))
        print(f"[single_global_runner] Launched {name} helper (pid={proc.pid}) -> {log_path}")

    def launch_optional_helpers(self):
        exports_dir = self.notes_root / "exports"
        ensure_dir(exports_dir)
        if self.args.launch_crops:
            cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"python3 {self.args.scripts_root / 'export_crops.py'} "
                f"--hz {self.args.export_hz} "
                f"--global-full-hz {self.args.global_full_hz} "
                f"--crop-m {self.args.crop_m} "
                f"--out-px {self.args.out_px} "
                f"--out-dir {exports_dir}"
            )
            self._start_helper("export_crops", cmd)
        if self.args.launch_rgb:
            cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"python3 {self.args.scripts_root / 'export_rgb_dual.py'} "
                f"--hz {self.args.rgb_hz} "
                f"--out-dir {exports_dir}"
            )
            self._start_helper("export_rgb_dual", cmd)
        if self.helper_procs and self.args.helper_warmup_s > 0.0:
            print(f"[single_global_runner] Waiting {self.args.helper_warmup_s:.1f}s for helper startup...")
            time.sleep(self.args.helper_warmup_s)

    def stop_optional_helpers(self):
        for name, proc, log_f in reversed(self.helper_procs):
            try:
                if proc.poll() is None:
                    print(f"[single_global_runner] Stopping {name} helper (pid={proc.pid})")
                    proc.terminate()
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=2.0)
            finally:
                try:
                    log_f.close()
                except Exception:
                    pass
        self.helper_procs.clear()

    def shutdown(self):
        self.stop_optional_helpers()
        self._write_runner_state({
            "schema_version": "single_global_runner_state_v1",
            "wall_time_iso": iso_now_local(),
            "status": "stopped",
            "stop_reason": self.stop_reason,
        })

    def _default_markers(self) -> Dict[str, Any]:
        return {
            "ultimate_goal": {"x": float(self.args.goal_x), "y": float(self.args.goal_y)},
            "active_goal": None,
            "pois": [],
        }

    def _initial_queue_state(self) -> Dict[str, Any]:
        return {
            "schema_version": "single_global_waypoint_queue_v1",
            "plan_instance_id": None,
            "created_wall_time_iso": iso_now_local(),
            "last_planner_action": None,
            "goal_ratified": False,
            "waypoints": [],
        }

    def reset_startup_state(self):
        atomic_write_json(self.markers_path, self._default_markers())
        atomic_write_json(self.queue_path, self._initial_queue_state())
        if self.runner_state_path.exists():
            self.runner_state_path.unlink()
        if self.planner_state_path.exists():
            self.planner_state_path.unlink()
        atomic_write_json(self.rejections_path, default_recent_route_rejections())
        self.last_planner_call_wall_time = None
        self.goal_ratified = False
        self.stop_reason = None
        self.last_nav_event = None
        self.goal_progress_reference_distance_m = None
        self.last_goal_progress_wall_time = None
        print(f"[single_global_runner] Reset startup state under {self.state_dir}")

    def load_queue_state(self) -> Dict[str, Any]:
        return load_json(self.queue_path, default=self._initial_queue_state())

    def save_queue_state(self, obj: Dict[str, Any]):
        atomic_write_json(self.queue_path, obj)

    def _write_runner_state(self, obj: Dict[str, Any]):
        atomic_write_json(self.runner_state_path, obj)

    def _write_planner_state(self, obj: Dict[str, Any]):
        atomic_write_json(self.planner_state_path, obj)

    def load_recent_rejections(self) -> Dict[str, Any]:
        return prune_recent_route_rejections(load_json(self.rejections_path, default=None))

    def save_recent_rejections(self, obj: Dict[str, Any]):
        atomic_write_json(self.rejections_path, prune_recent_route_rejections(obj))

    def _lookup_waypoint_by_id(self, waypoint_id: str) -> Optional[Dict[str, Any]]:
        queue_obj = self.load_queue_state()
        for wp in queue_obj.get("waypoints", []):
            if str(wp.get("id")) == str(waypoint_id):
                return wp
        return None

    def _global_info_for_runtime(self) -> Optional[GlobalImageInfo]:
        tick = self.last_tick if isinstance(self.last_tick, dict) else load_json(self.tick_path, default=None)
        if not isinstance(tick, dict):
            return None
        geom = tick.get("global_image_geometry") or {}
        try:
            return GlobalImageInfo(
                w=int(geom["w"]),
                h=int(geom["h"]),
                res=float(geom["res"]),
                ox=float(geom["origin_x"]),
                oy=float(geom["origin_y"]),
            )
        except Exception:
            return None

    def _safe_mask_from_tick(self, tick: Dict[str, Any]):
        exports = tick.get("exports") or {}
        path_s = exports.get("global_costmap_png")
        if not path_s:
            raise RuntimeError("tick missing exports.global_costmap_png for waypoint safety validation")
        path = Path(path_s)
        try:
            mt = path.stat().st_mtime
        except Exception as e:
            raise RuntimeError(f"global costmap image unavailable for waypoint safety validation: {path} ({e})")
        cache = self._safe_mask_cache
        cache_key = (str(path), float(mt), int(self.args.safe_free_threshold), int(self.args.safe_erosion_radius_px))
        if isinstance(cache, dict) and cache.get("key") == cache_key:
            return cache["mask"]
        mask = build_safe_waypoint_mask(
            path,
            free_threshold=int(self.args.safe_free_threshold),
            erosion_radius_px=int(self.args.safe_erosion_radius_px),
        )
        self._safe_mask_cache = {"key": cache_key, "mask": mask}
        return mask

    def _sanitize_waypoints(self, tick: Dict[str, Any], parsed: Dict[str, Any]) -> Dict[str, Any]:
        if parsed.get("action") != "replace_plan":
            return {"waypoints": parsed.get("waypoints", []), "repair_count": 0, "repairs": [], "rejected": False}
        mask = self._safe_mask_from_tick(tick)
        repaired_waypoints: List[Dict[str, Any]] = []
        repairs: List[Dict[str, Any]] = []
        repair_count = 0
        for idx, wp in enumerate(parsed.get("waypoints", []), start=1):
            u = int(wp["u"])
            v = int(wp["v"])
            if is_safe_waypoint_pixel(mask, u, v):
                repaired_waypoints.append(dict(wp))
                continue
            nearest = nearest_safe_waypoint_pixel(mask, u, v, max_radius_px=int(self.args.waypoint_repair_radius_px))
            if nearest is None:
                self.add_recent_rejection_region(
                    reason="unsafe_waypoint_candidate",
                    center_u=u,
                    center_v=v,
                    radius_px=max(int(self.args.rejection_radius_px), int(self.args.safe_erosion_radius_px) * 2),
                    label=f"unsafe_wp_{idx}",
                    replans_remaining=max(1, int(self.args.rejection_replans)),
                )
                dbg = waypoint_mask_debug(mask, u, v)
                raise RuntimeError(
                    f"Planner waypoint {idx} landed in unsafe global-costmap space and no safe repair was found within "
                    f"{self.args.waypoint_repair_radius_px}px: {dbg}"
                )
            uu, vv, rr = nearest
            repair_count += 1
            if repair_count > int(self.args.max_repaired_waypoints_per_plan):
                self.add_recent_rejection_region(
                    reason="too_many_waypoint_repairs_needed",
                    center_u=u,
                    center_v=v,
                    radius_px=max(int(self.args.rejection_radius_px), int(self.args.safe_erosion_radius_px) * 2),
                    label=f"repair_limit_{idx}",
                    replans_remaining=max(1, int(self.args.rejection_replans)),
                )
                raise RuntimeError(
                    f"Planner plan required {repair_count} waypoint repairs, which exceeds "
                    f"--max-repaired-waypoints-per-plan={self.args.max_repaired_waypoints_per_plan}"
                )
            repaired = dict(wp)
            repaired["u"] = int(uu)
            repaired["v"] = int(vv)
            repaired_waypoints.append(repaired)
            repairs.append({
                "index": idx,
                "label": wp.get("label", f"wp_{idx}"),
                "from_u": int(u),
                "from_v": int(v),
                "to_u": int(uu),
                "to_v": int(vv),
                "repair_radius_px": int(rr),
            })
        return {"waypoints": repaired_waypoints, "repair_count": repair_count, "repairs": repairs, "rejected": False}

    def add_recent_rejection_region(
        self,
        *,
        reason: str,
        waypoint_id: Optional[str] = None,
        center_u: Optional[int] = None,
        center_v: Optional[int] = None,
        radius_px: Optional[int] = None,
        label: Optional[str] = None,
        replans_remaining: Optional[int] = None,
    ):
        wp = self._lookup_waypoint_by_id(waypoint_id) if waypoint_id else None
        if (center_u is None or center_v is None) and wp is not None:
            try:
                center_u = int(wp["u"])
                center_v = int(wp["v"])
            except Exception:
                center_u = None
                center_v = None
        if (center_u is None or center_v is None) and wp is not None:
            info = self._global_info_for_runtime()
            if info is not None:
                try:
                    px = info.map_to_px(float(wp["x_map"]), float(wp["y_map"]))
                except Exception:
                    px = None
                if px is not None:
                    center_u, center_v = px
        if center_u is None or center_v is None:
            return
        radius_px = int(self.args.rejection_radius_px if radius_px is None else radius_px)
        replans_remaining = int(self.args.rejection_replans if replans_remaining is None else replans_remaining)
        obj = self.load_recent_rejections()
        rej = {
            "id": f"rej_{int(time.time()*1000)}",
            "kind": "recent_failed_region",
            "reason": reason,
            "label": label or waypoint_id or reason,
            "center_u": int(center_u),
            "center_v": int(center_v),
            "radius_px": int(radius_px),
            "replans_remaining": int(replans_remaining),
            "created_wall_time_iso": iso_now_local(),
        }
        obj.setdefault("rejections", []).append(rej)
        self.save_recent_rejections(obj)

    def consume_replan_budget(self):
        self.save_recent_rejections(decrement_recent_route_rejections(self.load_recent_rejections()))

    def _update_markers_active_goal(self, goal_xy: Optional[Dict[str, float]]):
        markers = load_json(self.markers_path, default=self._default_markers())
        markers.setdefault("ultimate_goal", {"x": float(self.args.goal_x), "y": float(self.args.goal_y)})
        markers.setdefault("pois", [])
        markers["active_goal"] = None if goal_xy is None else {"x": float(goal_xy["x"]), "y": float(goal_xy["y"])}
        atomic_write_json(self.markers_path, markers)

    def run_prepare_tick(self) -> Dict[str, Any]:
        cmd = [sys.executable, str(self.prepare_script), "--notes-root", str(self.notes_root), "--tf-wait", str(self.args.tf_wait)]
        if self.args.include_local_crop:
            cmd.append("--include-local-crop")
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        if result.returncode != 0:
            raise RuntimeError(f"prepare_single_global_tick.py failed with rc={result.returncode}")
        tick = load_json(self.tick_path)
        if not isinstance(tick, dict):
            raise RuntimeError(f"Failed to load tick from {self.tick_path}")
        if not tick.get("ready_for_lvlm", False):
            raise RuntimeError(f"Prepared tick not ready_for_lvlm: {tick.get('not_ready_reasons', [])}")
        self.last_tick = tick
        return tick

    def _robot_pose_map(self) -> Optional[Dict[str, float]]:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
            return {"x_map": float(x), "y_map": float(y), "yaw_rad": float(yaw)}
        except TransformException:
            return None

    def build_image_content(self, tick: Dict[str, Any]):
        planner_inputs = tick["planner_inputs"]
        content = [
            {"type": "input_text", "text": "Image 1: full global overlay (PRIMARY route-planning image)."},
            {"type": "input_image", "image_url": image_file_to_data_url(Path(planner_inputs["primary_global_full_png"])), "detail": "high"},
            {"type": "input_text", "text": "Image 2: left RGB scene-status image."},
            {"type": "input_image", "image_url": image_file_to_data_url(Path(planner_inputs["primary_rgb_png"])), "detail": "high"},
        ]
        local_crop = planner_inputs.get("optional_local_crop_png")
        if local_crop:
            content.extend([
                {"type": "input_text", "text": "Image 3: optional local crop image (supplemental near-field evidence only)."},
                {"type": "input_image", "image_url": image_file_to_data_url(Path(local_crop)), "detail": "high"},
            ])
        return content

    def call_json_model(self, tick: Dict[str, Any], payload_text: str):
        content = [
            {"type": "input_text", "text": self.prompt_text},
            {"type": "input_text", "text": payload_text},
        ]
        content.extend(self.build_image_content(tick))
        return self.client.responses.create(
            model=self.args.model,
            input=[{"role": "user", "content": content}],
            text={"format": {"type": "json_object"}},
        )

    def extract_text_output(self, response) -> str:
        text = getattr(response, "output_text", None)
        if text:
            return text
        pieces = []
        for item in getattr(response, "output", []) or []:
            for part in getattr(item, "content", []) or []:
                t = getattr(part, "text", None)
                if t:
                    pieces.append(t)
        if pieces:
            return "\n".join(pieces)
        raise RuntimeError("Could not extract text output from Responses API result")

    def build_planner_payload_text(self, tick: Dict[str, Any], runner_state: Dict[str, Any], trigger: str, trigger_reason: str) -> str:
        payload = {
            "planner_mode": "single_global_waypoint_queue",
            "trigger": trigger,
            "trigger_reason": trigger_reason,
            "run_label": self.args.run_label,
            "mission_state": tick.get("mission_state", {}),
            "robot_state": tick.get("robot_state", {}),
            "queue_state": tick.get("queue_state", {}),
            "runner_state": runner_state,
            "recent_route_rejections": self.load_recent_rejections(),
            "last_nav_event": self.last_nav_event,
            "global_image_geometry": tick.get("global_image_geometry", {}),
            "planner_contract": {
                "allowed_actions": ["keep_plan", "replace_plan", "ratify_goal_reached"],
                "replace_plan_waypoints": {
                    "coordinate_system": "full_global_overlay_pixels",
                    "fields": ["u", "v", "label"],
                    "u_axis": "right",
                    "v_axis": "down",
                    "max_waypoints": 4,
                },
                "queue_replace_policy": "replace remaining queue only; never append",
            },
        }
        return (
            "SINGLE GLOBAL WAYPOINT PLANNER PAYLOAD\n\n"
            + json.dumps(payload, indent=2)
            + "\n\nPlanner emphasis: use the smallest safe number of waypoints. One waypoint is acceptable when only the next safe commitment is clear. Usually prefer 2 to 4 waypoints, never more than 4. Prefer longer bright-white routes over shorter gray/dark shortcuts. Medium gray, dark gray, black, and mixed edge pixels are not acceptable waypoint targets."
            + "\n\nReturn only valid JSON matching the single-global waypoint schema."
        )

    def parse_planner_response(self, raw_text: str, tick: Dict[str, Any]) -> Dict[str, Any]:
        obj = json.loads(raw_text)
        if not isinstance(obj, dict):
            raise RuntimeError("Planner output must be a JSON object")
        action = str(obj.get("action", "")).strip()
        if action not in ("keep_plan", "replace_plan", "ratify_goal_reached"):
            raise RuntimeError(f"Invalid planner action: {action}")
        reason = str(obj.get("reason", "")).strip()
        waypoints = obj.get("waypoints", [])
        if not isinstance(waypoints, list):
            raise RuntimeError("Planner waypoints must be a list")
        if action in ("keep_plan", "ratify_goal_reached") and len(waypoints) != 0:
            raise RuntimeError(f"Planner returned non-empty waypoints for action={action}")
        if action == "replace_plan":
            if not (1 <= len(waypoints) <= 4):
                raise RuntimeError(f"replace_plan must return 1..4 waypoints, got {len(waypoints)}")
            geom = tick.get("global_image_geometry") or {}
            w = int(geom.get("w", 0))
            h = int(geom.get("h", 0))
            for idx, wp in enumerate(waypoints):
                if not isinstance(wp, dict):
                    raise RuntimeError(f"Waypoint {idx} is not an object")
                for key in ("u", "v", "label"):
                    if key not in wp:
                        raise RuntimeError(f"Waypoint {idx} missing key {key}")
                u = int(wp["u"])
                v = int(wp["v"])
                if u < 0 or v < 0 or u >= w or v >= h:
                    raise RuntimeError(f"Waypoint {idx} out of bounds: {(u, v)} not within {(w, h)}")
        return {"action": action, "reason": reason, "waypoints": waypoints, "raw_model_output": raw_text}

    def plan_once(self, tick: Dict[str, Any], runner_state: Dict[str, Any], trigger: str, trigger_reason: str) -> Dict[str, Any]:
        self.consume_replan_budget()
        response = self.call_json_model(tick=tick, payload_text=self.build_planner_payload_text(tick=tick, runner_state=runner_state, trigger=trigger, trigger_reason=trigger_reason))
        raw_text = self.extract_text_output(response)
        parsed = self.parse_planner_response(raw_text, tick)
        safety_filter = self._sanitize_waypoints(tick, parsed)
        parsed["waypoints"] = safety_filter["waypoints"]
        planner_state = {
            "schema_version": "single_global_last_planner_response_v1",
            "created_wall_time_iso": iso_now_local(),
            "planner_model": self.args.model,
            "trigger": trigger,
            "trigger_reason": trigger_reason,
            "action": parsed["action"],
            "reason": parsed["reason"],
            "waypoints": parsed["waypoints"],
            "raw_model_output": parsed["raw_model_output"],
            "waypoint_safety_filter": safety_filter,
        }
        self._write_planner_state(planner_state)
        self.last_planner_call_wall_time = time.time()
        return planner_state

    def _build_plan_instance_id(self) -> str:
        return datetime.now().strftime("plan_%Y%m%d_%H%M%S")

    def _global_info_from_tick(self, tick: Dict[str, Any]) -> GlobalImageInfo:
        geom = tick.get("global_image_geometry") or {}
        return GlobalImageInfo(
            w=int(geom["w"]),
            h=int(geom["h"]),
            res=float(geom["res"]),
            ox=float(geom["origin_x"]),
            oy=float(geom["origin_y"]),
        )

    def install_new_queue_from_waypoints(self, tick: Dict[str, Any], planner_state: Dict[str, Any]):
        info = self._global_info_from_tick(tick)
        plan_instance_id = self._build_plan_instance_id()
        waypoints_out = []
        for idx, wp in enumerate(planner_state["waypoints"], start=1):
            u = int(wp["u"])
            v = int(wp["v"])
            x_map, y_map = info.px_to_map(u, v)
            waypoints_out.append(
                {
                    "id": f"{plan_instance_id}_wp_{idx:02d}",
                    "label": str(wp.get("label", f"wp_{idx:02d}")),
                    "u": u,
                    "v": v,
                    "x_map": round(x_map, 6),
                    "y_map": round(y_map, 6),
                    "status": "pending",
                }
            )
        queue_obj = {
            "schema_version": "single_global_waypoint_queue_v1",
            "plan_instance_id": plan_instance_id,
            "created_wall_time_iso": iso_now_local(),
            "last_planner_action": planner_state["action"],
            "last_planner_reason": planner_state["reason"],
            "goal_ratified": False,
            "waypoints": waypoints_out,
        }
        self.save_queue_state(queue_obj)
        self.last_queue_replace_wall_time = time.time()
        self.goal_progress_reference_distance_m = None
        self.last_goal_progress_wall_time = time.time()

    def _cancel_active_goal_if_any(self):
        if self.active_goal_handle is None:
            return
        try:
            future = self.active_goal_handle.cancel_goal_async()
            future.add_done_callback(lambda f: None)
        except Exception:
            pass

    def apply_planner_state(self, tick: Dict[str, Any], planner_state: Dict[str, Any]):
        action = planner_state["action"]
        if action == "keep_plan":
            return
        if action == "ratify_goal_reached":
            self.goal_ratified = True
            self._cancel_active_goal_if_any()
            queue_obj = self.load_queue_state()
            queue_obj["goal_ratified"] = True
            queue_obj["last_planner_action"] = action
            self.save_queue_state(queue_obj)
            self._update_markers_active_goal(None)
            self.stop_reason = "planner_ratified_goal_reached"
            return
        if action == "replace_plan":
            self._cancel_active_goal_if_any()
            self.active_goal_handle = None
            self.active_goal_seq = None
            self.install_new_queue_from_waypoints(tick=tick, planner_state=planner_state)
            self._update_markers_active_goal(None)
            return
        raise RuntimeError(f"Unhandled planner action: {action}")

    def _wait_for_action_server(self):
        start = time.time()
        while time.time() - start < self.args.action_wait_s:
            if self.action_client.wait_for_server(timeout_sec=0.5):
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _send_nav_goal_async(self, waypoint: Dict[str, Any]):
        if not self._wait_for_action_server():
            raise RuntimeError(f"Action server {self.args.action_name} not available")
        self.latest_goal_seq += 1
        goal_seq = self.latest_goal_seq
        self.active_goal_seq = goal_seq
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(waypoint["x_map"])
        goal_msg.pose.pose.position.y = float(waypoint["y_map"])
        goal_msg.pose.pose.orientation.w = 1.0
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(partial(self._on_goal_response, goal_seq, waypoint["id"], waypoint["x_map"], waypoint["y_map"]))

    def _on_goal_response(self, goal_seq: int, waypoint_id: str, x_map: float, y_map: float, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.last_nav_event = {"phase": "goal_response", "ok": False, "error": f"Exception waiting for goal response: {e}", "goal_seq": goal_seq, "waypoint_id": waypoint_id}
            self.add_recent_rejection_region(reason="goal_response_exception", waypoint_id=waypoint_id)
            self.pending_replan = ("nav2_failed_waypoint", f"exception waiting for goal response for {waypoint_id}: {e}")
            return
        if not goal_handle.accepted:
            self.last_nav_event = {"phase": "goal_response", "ok": False, "error": "Goal rejected by Nav2", "goal_seq": goal_seq, "waypoint_id": waypoint_id}
            self.add_recent_rejection_region(reason="goal_rejected", waypoint_id=waypoint_id)
            self.pending_replan = ("nav2_failed_waypoint", f"Nav2 rejected waypoint {waypoint_id}")
            return
        self.active_goal_handle = goal_handle
        self.last_nav_event = {"phase": "goal_accepted", "ok": True, "goal_seq": goal_seq, "waypoint_id": waypoint_id, "goal": {"x": x_map, "y": y_map}}
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._on_nav_result, goal_seq, waypoint_id, x_map, y_map, goal_handle))

    def _set_waypoint_status(self, waypoint_id: str, status: str):
        queue_obj = self.load_queue_state()
        for wp in queue_obj.get("waypoints", []):
            if wp.get("id") == waypoint_id:
                wp["status"] = status
                break
        self.save_queue_state(queue_obj)

    def _on_nav_result(self, goal_seq: int, waypoint_id: str, x_map: float, y_map: float, goal_handle, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as e:
            self.last_nav_event = {"phase": "nav2_result", "ok": False, "error": f"Exception waiting for Nav2 result: {e}", "goal_seq": goal_seq, "waypoint_id": waypoint_id}
            self.add_recent_rejection_region(reason="nav2_result_exception", waypoint_id=waypoint_id)
            self.pending_replan = ("nav2_failed_waypoint", f"Nav2 result exception for {waypoint_id}: {e}")
            return
        stale = goal_seq != self.active_goal_seq
        self.last_nav_event = {"phase": "nav2_result", "ok": True, "goal_seq": goal_seq, "waypoint_id": waypoint_id, "nav2_status": status, "stale_result": stale, "goal": {"x": x_map, "y": y_map}}
        if stale:
            return
        if self.active_goal_handle == goal_handle:
            self.active_goal_handle = None
            self.active_goal_seq = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_waypoint_status(waypoint_id, "done")
            self._update_markers_active_goal(None)
            self.goal_progress_reference_distance_m = None
            self.last_goal_progress_wall_time = time.time()
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_waypoint_status(waypoint_id, "cancelled")
        else:
            self._set_waypoint_status(waypoint_id, "failed")
            self.add_recent_rejection_region(reason=f"nav2_status_{status}", waypoint_id=waypoint_id)
            self.pending_replan = ("nav2_failed_waypoint", f"Nav2 returned status {status} for waypoint {waypoint_id}")
            self._update_markers_active_goal(None)

    def dispatch_next_waypoint_if_needed(self, runner_state: Dict[str, Any]):
        if self.goal_ratified:
            return
        if self.active_goal_handle is not None:
            return
        queue_obj = self.load_queue_state()
        remaining = remaining_waypoints(queue_obj)
        if not remaining:
            return
        next_wp = remaining[0]
        if str(next_wp.get("status")) != "active":
            next_wp["status"] = "active"
            self.save_queue_state(queue_obj)
        self._update_markers_active_goal({"x": float(next_wp["x_map"]), "y": float(next_wp["y_map"])})
        self.goal_progress_reference_distance_m = runner_state.get("distance_to_active_waypoint_m")
        self.last_goal_progress_wall_time = time.time()
        self._send_nav_goal_async(next_wp)

    def compute_runner_state(self, tick: Dict[str, Any]) -> Dict[str, Any]:
        queue_obj = self.load_queue_state()
        mission = tick.get("mission_state", {})
        robot_state = tick.get("robot_state", {})
        remaining = remaining_waypoints(queue_obj)
        active_wp = active_waypoint(queue_obj)
        if active_wp is None and remaining:
            active_wp = remaining[0]
        dist_to_active = euclidean_xy(robot_state, active_wp)
        dist_to_ultimate = euclidean_xy(robot_state, mission.get("ultimate_goal"))
        if dist_to_active is not None:
            if self.goal_progress_reference_distance_m is None:
                self.goal_progress_reference_distance_m = dist_to_active
                self.last_goal_progress_wall_time = time.time()
            elif dist_to_active < self.goal_progress_reference_distance_m - self.args.progress_epsilon_m:
                self.goal_progress_reference_distance_m = dist_to_active
                self.last_goal_progress_wall_time = time.time()
        seconds_since_progress = None if self.last_goal_progress_wall_time is None else round(time.time() - self.last_goal_progress_wall_time, 3)
        seconds_since_planner = None if self.last_planner_call_wall_time is None else round(time.time() - self.last_planner_call_wall_time, 3)
        near_last_remaining_waypoint = bool(active_wp and len(remaining) <= 1 and dist_to_active is not None and dist_to_active <= self.args.near_last_waypoint_thresh_m)
        hard_goal_stop = bool(self.args.hard_goal_stop_thresh_m > 0.0 and dist_to_ultimate is not None and dist_to_ultimate <= self.args.hard_goal_stop_thresh_m)
        return {
            "schema_version": "single_global_runner_state_v1",
            "wall_time_iso": iso_now_local(),
            "active_waypoint": None if active_wp is None else {
                "id": active_wp.get("id"),
                "label": active_wp.get("label"),
                "x_map": active_wp.get("x_map"),
                "y_map": active_wp.get("y_map"),
                "u": active_wp.get("u"),
                "v": active_wp.get("v"),
                "status": active_wp.get("status"),
            },
            "remaining_waypoint_count": len(remaining),
            "remaining_waypoints": summarize_waypoints_for_payload(queue_obj),
            "distance_to_active_waypoint_m": None if dist_to_active is None else round(dist_to_active, 4),
            "distance_to_ultimate_goal_m": None if dist_to_ultimate is None else round(dist_to_ultimate, 4),
            "active_waypoint_geometry": compute_relative_geometry(robot_state, active_wp),
            "ultimate_goal_geometry": compute_relative_geometry(robot_state, mission.get("ultimate_goal")),
            "seconds_since_progress": seconds_since_progress,
            "seconds_since_last_planner_call": seconds_since_planner,
            "planner_interval_s": float(self.args.planner_interval_s),
            "progress_stall_window_s": float(self.args.progress_stall_window_s),
            "progress_epsilon_m": float(self.args.progress_epsilon_m),
            "near_last_remaining_waypoint": near_last_remaining_waypoint,
            "active_goal_in_flight": self.active_goal_handle is not None,
            "goal_ratified": self.goal_ratified,
            "hard_goal_stop": hard_goal_stop,
            "recent_route_rejection_count": len(self.load_recent_rejections().get("rejections", [])),
        }

    def determine_planner_trigger(self, runner_state: Dict[str, Any]) -> Optional[Tuple[str, str]]:
        if self.goal_ratified:
            return None
        if self.pending_replan is not None:
            trigger = self.pending_replan
            self.pending_replan = None
            return trigger
        if runner_state["hard_goal_stop"]:
            self.goal_ratified = True
            self.stop_reason = "hard_goal_stop_threshold"
            return None
        if runner_state["remaining_waypoint_count"] == 0 and not runner_state["active_goal_in_flight"]:
            return ("queue_empty", "no remaining queue is available")
        if runner_state["near_last_remaining_waypoint"]:
            return ("near_last_remaining_waypoint", "robot is near the last remaining queued waypoint")
        s_progress = runner_state.get("seconds_since_progress")
        if s_progress is not None and s_progress >= self.args.progress_stall_window_s:
            active_wp = runner_state.get("active_waypoint")
            if isinstance(active_wp, dict):
                self.add_recent_rejection_region(reason="progress_stalled", waypoint_id=active_wp.get("id"))
            return ("progress_stalled", f"distance to active waypoint has not improved enough for {s_progress}s")
        s_planner = runner_state.get("seconds_since_last_planner_call")
        if self.last_planner_call_wall_time is None:
            return ("startup", "initial planning call")
        if s_planner is not None and s_planner >= self.args.planner_interval_s:
            return ("periodic_recheck", f"periodic replanning interval of {self.args.planner_interval_s}s elapsed")
        return None

    def log_line(self, obj: Dict[str, Any]):
        with self.cycle_log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(obj) + "\n")

    def spin_briefly(self, seconds: float):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self.launch_optional_helpers()
        self.reset_startup_state()
        cycle_idx = 0
        while rclpy.ok():
            if self.args.max_cycles is not None and cycle_idx >= self.args.max_cycles:
                self.stop_reason = "max_cycles_reached"
                break
            cycle_start = time.time()
            cycle_log: Dict[str, Any] = {
                "phase": "single_global_cycle",
                "cycle_index": cycle_idx,
                "cycle_wall_time_iso": iso_now_local(),
                "planner_model": self.args.model,
                "run_label": self.args.run_label,
                "status": "started",
            }
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
                tick = self.run_prepare_tick()
                runner_state = self.compute_runner_state(tick)
                self._write_runner_state(runner_state)
                cycle_log["runner_state"] = deepcopy(runner_state)
                cycle_log["last_nav_event"] = deepcopy(self.last_nav_event)
                trigger = self.determine_planner_trigger(runner_state)
                if trigger is not None:
                    planner_state = self.plan_once(tick=tick, runner_state=runner_state, trigger=trigger[0], trigger_reason=trigger[1])
                    cycle_log["planner_state"] = planner_state
                    self.apply_planner_state(tick=tick, planner_state=planner_state)
                    runner_state = self.compute_runner_state(tick)
                    self._write_runner_state(runner_state)
                if self.goal_ratified:
                    cycle_log["status"] = "goal_ratified"
                    self.log_line(cycle_log)
                    break
                self.dispatch_next_waypoint_if_needed(runner_state)
                cycle_log["status"] = "ok"
            except Exception as e:
                cycle_log["status"] = "error"
                cycle_log["error"] = str(e)
                self.log_line(cycle_log)
                print(f"[single_global_runner] Cycle {cycle_idx} ERROR: {e}", file=sys.stderr)
                if self.args.stop_on_error:
                    raise
            self.log_line(cycle_log)
            cycle_idx += 1
            elapsed = time.time() - cycle_start
            remaining = max(0.0, (1.0 / self.args.tick_hz) - elapsed)
            self.spin_briefly(remaining)
        if self.stop_reason is None:
            self.stop_reason = "goal_ratified" if self.goal_ratified else "loop_exit"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--notes-root", default="~/table_nav2/notes/current")
    ap.add_argument("--scripts-root", type=Path, default=Path("~/table_nav2/scripts").expanduser())
    ap.add_argument("--goal-x", type=float, required=True)
    ap.add_argument("--goal-y", type=float, required=True)
    ap.add_argument("--model", default="gpt-5.4")
    ap.add_argument("--prompt-file", default=str((Path(__file__).resolve().parent / "lvlm_single_global_waypoint_prompt.txt")))
    ap.add_argument("--action-name", default="/navigate_to_pose")
    ap.add_argument("--tick-hz", type=float, default=0.5)
    ap.add_argument("--tf-wait", type=float, default=1.0)
    ap.add_argument("--planner-interval-s", type=float, default=12.0)
    ap.add_argument("--progress-stall-window-s", type=float, default=8.0)
    ap.add_argument("--progress-epsilon-m", type=float, default=0.15)
    ap.add_argument("--near-last-waypoint-thresh-m", type=float, default=1.5)
    ap.add_argument("--hard-goal-stop-thresh-m", type=float, default=0.0)
    ap.add_argument("--action-wait-s", type=float, default=10.0)
    ap.add_argument("--run-label", default="lvlm_single_global_waypoint")
    ap.add_argument("--max-cycles", type=int, default=None)
    ap.add_argument("--stop-on-error", action="store_true")
    ap.add_argument("--include-local-crop", action="store_true")
    ap.add_argument("--launch-crops", action="store_true")
    ap.add_argument("--launch-rgb", action="store_true")
    ap.add_argument("--helper-warmup-s", type=float, default=3.0)
    ap.add_argument("--export-hz", type=float, default=0.5)
    ap.add_argument("--global-full-hz", type=float, default=0.5)
    ap.add_argument("--crop-m", type=float, default=10.0)
    ap.add_argument("--out-px", type=int, default=200)
    ap.add_argument("--rgb-hz", type=float, default=0.5)
    ap.add_argument("--safe-free-threshold", type=int, default=245)
    ap.add_argument("--safe-erosion-radius-px", type=int, default=8)
    ap.add_argument("--waypoint-repair-radius-px", type=int, default=25)
    ap.add_argument("--max-repaired-waypoints-per-plan", type=int, default=2)
    ap.add_argument("--rejection-radius-px", type=int, default=36)
    ap.add_argument("--rejection-replans", type=int, default=3)
    args = ap.parse_args()

    rclpy.init()
    node = SingleGlobalWaypointRunner(args)
    try:
        node.run()
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
