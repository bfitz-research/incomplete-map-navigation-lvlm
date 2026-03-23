#!/usr/bin/env python3
import argparse
import base64
import io
import json
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from PIL import Image

from openai import OpenAI


def iso_now_local() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def atomic_write_json(path: Path, obj: dict):
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def image_file_to_data_url(path: Path, retries: int = 6, sleep_s: float = 0.08) -> str:
    suffix = path.suffix.lower()
    if suffix == ".png":
        mime = "image/png"
    elif suffix in (".jpg", ".jpeg"):
        mime = "image/jpeg"
    elif suffix == ".webp":
        mime = "image/webp"
    else:
        raise ValueError(f"Unsupported image type for data URL: {path}")

    last_err = None
    for _ in range(retries):
        try:
            raw = path.read_bytes()
            with Image.open(io.BytesIO(raw)) as im:
                im.verify()
            b64 = base64.b64encode(raw).decode("ascii")
            return f"data:{mime};base64,{b64}"
        except Exception as e:
            last_err = e
            time.sleep(sleep_s)

    raise RuntimeError(f"Failed to read a valid image from {path}: {last_err}")


def euclidean_xy(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> Optional[float]:
    if not a or not b:
        return None

    if "x" in a and "y" in a:
        ax, ay = float(a["x"]), float(a["y"])
    elif "x_map" in a and "y_map" in a:
        ax, ay = float(a["x_map"]), float(a["y_map"])
    else:
        return None

    if "x" in b and "y" in b:
        bx, by = float(b["x"]), float(b["y"])
    elif "x_map" in b and "y_map" in b:
        bx, by = float(b["x_map"]), float(b["y_map"])
    else:
        return None

    return math.hypot(ax - bx, ay - by)


def wrap_angle_rad(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def compute_relative_geometry(robot_state: Optional[Dict[str, Any]], target_xy: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not robot_state or not target_xy:
        return None
    if "x_map" not in robot_state or "y_map" not in robot_state or "yaw_rad" not in robot_state:
        return None
    if "x" not in target_xy or "y" not in target_xy:
        return None

    rx = float(robot_state["x_map"])
    ry = float(robot_state["y_map"])
    ryaw = float(robot_state["yaw_rad"])
    tx = float(target_xy["x"])
    ty = float(target_xy["y"])

    dx = tx - rx
    dy = ty - ry
    dist = math.hypot(dx, dy)
    bearing_global = math.atan2(dy, dx)
    bearing_relative = wrap_angle_rad(bearing_global - ryaw)

    return {
        "x_map": tx,
        "y_map": ty,
        "dx_m": round(dx, 4),
        "dy_m": round(dy, 4),
        "distance_m": round(dist, 4),
        "bearing_global_deg": round(math.degrees(bearing_global), 2),
        "bearing_relative_deg": round(math.degrees(bearing_relative), 2),
    }


def same_goal(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]], eps: float = 1e-6) -> bool:
    if not a or not b:
        return False
    if "x" not in a or "y" not in a or "x" not in b or "y" not in b:
        return False
    return abs(float(a["x"]) - float(b["x"])) <= eps and abs(float(a["y"]) - float(b["y"])) <= eps


class RegionPublisher(Node):
    def __init__(self):
        super().__init__("lvlm_region_request_publisher")
        self.pub = self.create_publisher(Int32MultiArray, "/region_request", 10)

    def publish_region(self, region_data):
        msg = Int32MultiArray()
        msg.data = [int(x) for x in region_data]
        self.pub.publish(msg)
        self.get_logger().info(f"Published /region_request: {msg.data}")


class LVLMLoopRunner:
    def __init__(self, args):
        self.args = args
        self.notes_root = Path(os.path.expanduser(args.notes_root)).resolve()
        self.scripts_root = Path(os.path.expanduser(args.scripts_root)).resolve()
        self.state_dir = self.notes_root / "state"
        self.ticks_dir = self.notes_root / "ticks" / "latest"
        self.runs_root = self.notes_root / "runs" / args.run_label

        ensure_dir(self.state_dir)
        ensure_dir(self.ticks_dir)
        ensure_dir(self.runs_root)

        self.run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.runs_root / self.run_ts
        ensure_dir(self.run_dir)

        self.markers_path = self.state_dir / "markers.json"
        self.global_plan_path = self.state_dir / "global_plan.json"
        self.blocked_regions_path = self.state_dir / "blocked_regions.json"
        self.tick_path = self.ticks_dir / "tick.json"
        self.prepare_script = self.scripts_root / "prepare_tick_payload_once.py"
        self.cycle_log_path = self.run_dir / "lvlm_cycles.jsonl"

        api_key = os.environ.get("OPENAI_API_KEY", "").strip()
        if not api_key:
            raise RuntimeError("OPENAI_API_KEY is not set in the environment")
        self.client = OpenAI(api_key=api_key)

        planner_prompt_path = Path(os.path.expanduser(args.planner_prompt_file)).resolve()
        local_prompt_path = Path(os.path.expanduser(args.local_prompt_file)).resolve()
        self.planner_prompt_text = planner_prompt_path.read_text(encoding="utf-8")
        self.local_prompt_text = local_prompt_path.read_text(encoding="utf-8")

        self.prev_active_goal: Optional[Dict[str, Any]] = None
        self.prev_dist_to_active_goal_m: Optional[float] = None
        self.prev_dist_to_ultimate_goal_m: Optional[float] = None
        self.last_published_region: Optional[list[int]] = None
        self.last_region_publish_wall_time: Optional[float] = None
        self.last_goal_progress_wall_time: Optional[float] = None
        self.helper_procs: list[tuple[str, subprocess.Popen, Any]] = []

        rclpy.init()
        self.publisher = RegionPublisher()

    def _helper_out_dir(self) -> Path:
        return self.notes_root / "exports"

    def _helper_log_dir(self) -> Path:
        p = self.run_dir / "helper_logs"
        ensure_dir(p)
        return p

    def _start_helper(self, name: str, command: str):
        log_path = self._helper_log_dir() / f"{name}.log"
        log_f = open(log_path, "a", encoding="utf-8")
        proc = subprocess.Popen(["bash", "-lc", command], stdout=log_f, stderr=subprocess.STDOUT)
        self.helper_procs.append((name, proc, log_f))
        print(f"[run_lvlm_loop] Launched {name} helper (pid={proc.pid}) -> {log_path}")

    def launch_optional_helpers(self):
        out_dir = self._helper_out_dir()
        ensure_dir(out_dir)

        if self.args.launch_crops:
            cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"python3 {self.scripts_root / 'export_crops.py'} "
                f"--hz {self.args.export_hz} "
                f"--global-full-hz {self.args.global_full_hz} "
                f"--crop-m {self.args.crop_m} "
                f"--out-px {self.args.out_px} "
                f"--out-dir {out_dir}"
            )
            self._start_helper("export_crops", cmd)

        if self.args.launch_rgb:
            cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"python3 {self.scripts_root / 'export_rgb_dual.py'} "
                f"--hz {self.args.rgb_hz} "
                f"--out-dir {out_dir}"
            )
            self._start_helper("export_rgb_dual", cmd)

        if self.args.launch_picker:
            cmd = (
                f"source /opt/ros/jazzy/setup.bash && "
                f"python3 {self.scripts_root / 'region_goal_picker_node.py'}"
            )
            self._start_helper("region_goal_picker_node", cmd)

        if self.helper_procs and self.args.helper_warmup_s > 0.0:
            print(f"[run_lvlm_loop] Waiting {self.args.helper_warmup_s:.1f}s for helper startup...")
            time.sleep(self.args.helper_warmup_s)

    def stop_optional_helpers(self):
        for name, proc, log_f in reversed(self.helper_procs):
            try:
                if proc.poll() is None:
                    print(f"[run_lvlm_loop] Stopping {name} helper (pid={proc.pid})")
                    proc.terminate()
                    try:
                        proc.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        print(f"[run_lvlm_loop] Killing {name} helper (pid={proc.pid})")
                        proc.kill()
                        proc.wait(timeout=2.0)
            except Exception as e:
                print(f"[run_lvlm_loop] Warning: failed to stop helper {name}: {e}", file=sys.stderr)
            finally:
                try:
                    log_f.close()
                except Exception:
                    pass
        self.helper_procs.clear()

    def shutdown(self):
        self.stop_optional_helpers()
        try:
            self.publisher.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def reset_startup_state(self):
        markers = {
            "ultimate_goal": {
                "x": float(self.args.goal_x),
                "y": float(self.args.goal_y),
            },
            "active_goal": None,
            "pois": [],
        }
        atomic_write_json(self.markers_path, markers)

        if self.global_plan_path.exists():
            self.global_plan_path.unlink()
        atomic_write_json(self.blocked_regions_path, {"schema_version": "blocked_regions_v1", "regions": []})

        self.prev_active_goal = None
        self.prev_dist_to_active_goal_m = None
        self.prev_dist_to_ultimate_goal_m = None
        self.last_published_region = None
        self.last_region_publish_wall_time = None
        self.last_goal_progress_wall_time = None

        print(f"[run_lvlm_loop] Reset markers.json -> {self.markers_path}")
        print("[run_lvlm_loop] Cleared startup state (global_plan removed, active_goal reset, pois cleared)")

    def run_prepare_payload(self) -> Dict[str, Any]:
        cmd = [
            sys.executable,
            str(self.prepare_script),
            "--notes-root",
            str(self.notes_root),
            "--tf-wait",
            str(self.args.tf_wait),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.stdout.strip():
            print(result.stdout.strip())
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr)
        if result.returncode != 0:
            raise RuntimeError(f"prepare_tick_payload_once.py failed with rc={result.returncode}")

        tick = load_json(self.tick_path)
        if not tick.get("ready_for_lvlm", False):
            raise RuntimeError(
                f"Prepared tick is not ready_for_lvlm. Reasons: {tick.get('not_ready_reasons', [])}"
            )
        return tick

    def load_global_plan(self) -> Optional[Dict[str, Any]]:
        if not self.global_plan_path.exists():
            return None
        try:
            obj = load_json(self.global_plan_path)
            if not isinstance(obj, dict):
                return None
            return obj
        except Exception:
            return None

    def save_global_plan(self, plan_obj: Dict[str, Any]):
        atomic_write_json(self.global_plan_path, plan_obj)


    def load_blocked_regions_state(self) -> Dict[str, Any]:
        if not self.blocked_regions_path.exists():
            return {"schema_version": "blocked_regions_v1", "regions": []}
        try:
            obj = load_json(self.blocked_regions_path)
            if not isinstance(obj, dict):
                return {"schema_version": "blocked_regions_v1", "regions": []}
            regions = obj.get("regions", [])
            if not isinstance(regions, list):
                regions = []
            clean = []
            for r in regions:
                if not isinstance(r, dict):
                    continue
                try:
                    clean.append({
                        "x_min": float(r["x_min"]),
                        "y_min": float(r["y_min"]),
                        "x_max": float(r["x_max"]),
                        "y_max": float(r["y_max"]),
                        "reason": str(r.get("reason", "")).strip(),
                        "source": str(r.get("source", "local")).strip(),
                        "created_wall_time_iso": r.get("created_wall_time_iso"),
                    })
                except Exception:
                    continue
            return {"schema_version": "blocked_regions_v1", "regions": clean}
        except Exception:
            return {"schema_version": "blocked_regions_v1", "regions": []}

    def save_blocked_regions_state(self, obj: Dict[str, Any]):
        atomic_write_json(self.blocked_regions_path, obj)

    def local_crop_box_to_map_box(self, tick: Dict[str, Any], local_box: list[int]) -> Optional[Dict[str, Any]]:
        crop = tick.get("export", {}).get("crop", {}) or {}
        bounds = crop.get("bounds_render", {}) or {}
        try:
            min_x = float(bounds["min_x"])
            max_y = float(bounds["max_y"])
            mpp = float(crop["meters_per_px"])
            x1, y1, x2, y2 = [int(v) for v in local_box]
        except Exception:
            return None

        xa = min_x + x1 * mpp
        xb = min_x + (x2 + 1) * mpp
        ya = max_y - (y1 + 1) * mpp
        yb = max_y - y2 * mpp
        return {
            "x_min": float(min(xa, xb)),
            "y_min": float(min(ya, yb)),
            "x_max": float(max(xa, xb)),
            "y_max": float(max(ya, yb)),
        }

    def persist_blocked_region_from_local_box(self, tick: Dict[str, Any], local_box: list[int], reason: str, source: str = "local_model"):
        mb = self.local_crop_box_to_map_box(tick, local_box)
        if mb is None:
            return None

        state = self.load_blocked_regions_state()
        regions = state.get("regions", [])
        cx = 0.5 * (mb["x_min"] + mb["x_max"])
        cy = 0.5 * (mb["y_min"] + mb["y_max"])

        deduped = []
        replaced = False
        for r in regions:
            rcx = 0.5 * (r["x_min"] + r["x_max"])
            rcy = 0.5 * (r["y_min"] + r["y_max"])
            if math.hypot(cx - rcx, cy - rcy) < 1.0:
                if not replaced:
                    deduped.append({
                        **mb,
                        "reason": reason,
                        "source": source,
                        "created_wall_time_iso": iso_now_local(),
                    })
                    replaced = True
            else:
                deduped.append(r)

        if not replaced:
            deduped.append({
                **mb,
                "reason": reason,
                "source": source,
                "created_wall_time_iso": iso_now_local(),
            })

        deduped = deduped[-8:]
        obj = {"schema_version": "blocked_regions_v1", "regions": deduped}
        self.save_blocked_regions_state(obj)
        return obj

    def global_plan_cooldown_remaining_s(self, global_plan: Optional[Dict[str, Any]]) -> Optional[float]:
        if not global_plan:
            return None
        created = global_plan.get("created_wall_time_unix")
        cooldown = global_plan.get("applied_cooldown_s")
        if created is None or cooldown is None:
            return None
        remaining = float(created) + float(cooldown) - time.time()
        return round(max(0.0, remaining), 3)

    def global_plan_age_s(self, global_plan: Optional[Dict[str, Any]]) -> Optional[float]:
        if not global_plan:
            return None
        created = global_plan.get("created_wall_time_unix")
        if created is None:
            return None
        return round(max(0.0, time.time() - float(created)), 3)

    def should_trigger_periodic_recheck(self, global_plan: Optional[Dict[str, Any]]) -> bool:
        interval = float(self.args.periodic_global_recheck_s)
        if self.args.planner_mode == "off" or interval <= 0.0:
            return False
        if global_plan is None:
            return False
        age = self.global_plan_age_s(global_plan)
        if age is None or age < interval:
            return False
        cooldown_remaining = self.global_plan_cooldown_remaining_s(global_plan)
        if cooldown_remaining is not None and cooldown_remaining > 0.0:
            return False
        return True

    def compute_runner_state(self, tick: Dict[str, Any], global_plan: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        mission = tick.get("mission_state", {})
        robot_state = tick.get("robot_state", {})
        active_goal = mission.get("active_goal")
        ultimate_goal = mission.get("ultimate_goal")

        dist_to_active_goal_m = euclidean_xy(robot_state, active_goal)
        dist_to_ultimate_goal_m = euclidean_xy(robot_state, ultimate_goal)
        active_goal_changed = not same_goal(active_goal, self.prev_active_goal)

        prev_dist = self.prev_dist_to_active_goal_m
        delta_active_goal_distance_m = None
        if (
            dist_to_active_goal_m is not None
            and prev_dist is not None
            and not active_goal_changed
        ):
            delta_active_goal_distance_m = prev_dist - dist_to_active_goal_m

        prev_ultimate = self.prev_dist_to_ultimate_goal_m
        delta_ultimate_goal_distance_m = None
        if dist_to_ultimate_goal_m is not None and prev_ultimate is not None:
            delta_ultimate_goal_distance_m = prev_ultimate - dist_to_ultimate_goal_m
            if delta_ultimate_goal_distance_m > float(self.args.goal_progress_epsilon_m):
                self.last_goal_progress_wall_time = time.time()

        has_active_goal = active_goal is not None
        final_goal_reached = (
            dist_to_ultimate_goal_m is not None
            and dist_to_ultimate_goal_m <= float(self.args.final_goal_reached_thresh)
        )

        near_active_goal = (
            has_active_goal
            and dist_to_active_goal_m is not None
            and dist_to_active_goal_m <= float(self.args.active_goal_near_thresh)
        )

        must_choose_new_region = False
        if not final_goal_reached:
            if not has_active_goal:
                must_choose_new_region = True
            elif near_active_goal:
                must_choose_new_region = True

        seconds_since_last_region_publish = None
        if self.last_region_publish_wall_time is not None:
            seconds_since_last_region_publish = round(time.time() - self.last_region_publish_wall_time, 3)

        seconds_since_goal_progress = None
        if self.last_goal_progress_wall_time is not None:
            seconds_since_goal_progress = round(time.time() - self.last_goal_progress_wall_time, 3)

        blocked_regions_state = self.load_blocked_regions_state()
        return {
            "has_active_goal": has_active_goal,
            "active_goal_changed_since_last_cycle": active_goal_changed,
            "distance_robot_to_active_goal_m": None if dist_to_active_goal_m is None else round(dist_to_active_goal_m, 4),
            "previous_distance_robot_to_active_goal_m": None if prev_dist is None else round(prev_dist, 4),
            "delta_active_goal_distance_m": None if delta_active_goal_distance_m is None else round(delta_active_goal_distance_m, 4),
            "distance_robot_to_ultimate_goal_m": None if dist_to_ultimate_goal_m is None else round(dist_to_ultimate_goal_m, 4),
            "previous_distance_robot_to_ultimate_goal_m": None if prev_ultimate is None else round(prev_ultimate, 4),
            "delta_ultimate_goal_distance_m": None if delta_ultimate_goal_distance_m is None else round(delta_ultimate_goal_distance_m, 4),
            "seconds_since_goal_progress": seconds_since_goal_progress,
            "goal_geometry": compute_relative_geometry(robot_state, ultimate_goal),
            "near_active_goal": bool(near_active_goal),
            "final_goal_reached": bool(final_goal_reached),
            "must_choose_new_region": bool(must_choose_new_region),
            "last_published_region": self.last_published_region,
            "seconds_since_last_region_publish": seconds_since_last_region_publish,
            "active_goal_near_thresh_m": float(self.args.active_goal_near_thresh),
            "final_goal_reached_thresh_m": float(self.args.final_goal_reached_thresh),
            "global_plan_present": global_plan is not None,
            "global_plan_strategy": None if global_plan is None else global_plan.get("global_strategy"),
            "global_plan_cooldown_remaining_s": self.global_plan_cooldown_remaining_s(global_plan),
            "global_plan_age_s": self.global_plan_age_s(global_plan),
            "periodic_global_recheck_s": float(self.args.periodic_global_recheck_s),
            "blocked_region_count": len(blocked_regions_state.get("regions", [])),
        }

    def update_cycle_memory(self, tick: Dict[str, Any], runner_state: Dict[str, Any]):
        mission = tick.get("mission_state", {})
        self.prev_active_goal = mission.get("active_goal")
        self.prev_dist_to_active_goal_m = runner_state.get("distance_robot_to_active_goal_m")
        self.prev_dist_to_ultimate_goal_m = runner_state.get("distance_robot_to_ultimate_goal_m")

    def build_image_content(self, tick: Dict[str, Any]):
        lvlm_inputs = tick["lvlm_inputs"]

        local_vis = Path(lvlm_inputs["primary_local_map_png"])
        global_crop_vis = Path(lvlm_inputs["primary_global_crop_png"])
        global_full_vis = Path(lvlm_inputs["primary_global_full_png"])
        rgb_left = Path(lvlm_inputs["primary_rgb_png"])

        return [
            {"type": "input_text", "text": "Image 1: local overlay map (primary local execution image)."},
            {"type": "input_image", "image_url": image_file_to_data_url(local_vis), "detail": "high"},
            {"type": "input_text", "text": "Image 2: global crop overlay."},
            {"type": "input_image", "image_url": image_file_to_data_url(global_crop_vis), "detail": "high"},
            {"type": "input_text", "text": "Image 3: full global overlay."},
            {"type": "input_image", "image_url": image_file_to_data_url(global_full_vis), "detail": "high"},
            {"type": "input_text", "text": "Image 4: left RGB camera view."},
            {"type": "input_image", "image_url": image_file_to_data_url(rgb_left), "detail": "high"},
        ]

    def call_json_model(self, model_name: str, prompt_text: str, payload_text: str, tick: Dict[str, Any]):
        content = [
            {"type": "input_text", "text": prompt_text},
            {"type": "input_text", "text": payload_text},
        ]
        content.extend(self.build_image_content(tick))

        return self.client.responses.create(
            model=model_name,
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

    def build_planner_payload_text(
        self,
        tick: Dict[str, Any],
        runner_state: Dict[str, Any],
        trigger: str,
        previous_global_plan: Optional[Dict[str, Any]],
        trigger_reason: str,
    ) -> str:
        payload = {
            "planner_mode": True,
            "trigger": trigger,
            "trigger_reason": trigger_reason,
            "run_label": self.args.run_label,
            "planner_model": self.args.planner_model,
            "mission_state": tick.get("mission_state", {}),
            "robot_state": tick.get("robot_state", {}),
            "runner_state": runner_state,
            "current_global_plan": previous_global_plan,
            "blocked_regions_state": self.load_blocked_regions_state(),
            "export": {
                "export_tick": tick.get("export", {}).get("export_tick"),
                "export_timestamp": tick.get("export", {}).get("export_timestamp"),
                "crop": tick.get("export", {}).get("crop"),
            },
            "planning_note": {
                "objective": "Produce semantic route-level guidance only. Do not output anchors or waypoints.",
            },
        }
        return (
            "GLOBAL REPLANNING PAYLOAD\n\n"
            + json.dumps(payload, indent=2)
            + "\n\nReturn only valid JSON matching the replanner schema."
        )

    def parse_global_plan_response(self, raw_text: str) -> Dict[str, Any]:
        obj = json.loads(raw_text)
        required = [
            "action",
            "reason",
            "global_strategy",
            "plan_summary",
            "local_execution_guidance",
            "avoid_strategy",
            "route_phase",
            "replan_trigger_hint",
            "plan_cooldown_s",
        ]
        missing = [k for k in required if k not in obj]
        if missing:
            raise RuntimeError(f"Global replanner response missing keys: {missing}")

        action = str(obj["action"]).strip()
        if action != "set_global_plan":
            raise RuntimeError(f"Global replanner action must be set_global_plan, got: {obj['action']}")

        cooldown = int(obj["plan_cooldown_s"])
        cooldown = max(5, min(30, cooldown))

        return {
            "action": "set_global_plan",
            "reason": str(obj["reason"]).strip(),
            "global_strategy": str(obj["global_strategy"]).strip(),
            "plan_summary": str(obj["plan_summary"]).strip(),
            "local_execution_guidance": str(obj["local_execution_guidance"]).strip(),
            "avoid_strategy": str(obj["avoid_strategy"]).strip(),
            "route_phase": str(obj["route_phase"]).strip(),
            "replan_trigger_hint": str(obj["replan_trigger_hint"]).strip(),
            "requested_cooldown_s": int(obj["plan_cooldown_s"]),
            "applied_cooldown_s": cooldown,
            "raw_model_output": raw_text,
        }

    def plan_global_once(
        self,
        tick: Dict[str, Any],
        runner_state: Dict[str, Any],
        trigger: str,
        trigger_reason: str,
        previous_global_plan: Optional[Dict[str, Any]],
    ) -> Dict[str, Any]:
        response = self.call_json_model(
            model_name=self.args.planner_model,
            prompt_text=self.planner_prompt_text,
            payload_text=self.build_planner_payload_text(
                tick=tick,
                runner_state=runner_state,
                trigger=trigger,
                previous_global_plan=previous_global_plan,
                trigger_reason=trigger_reason,
            ),
            tick=tick,
        )
        raw_text = self.extract_text_output(response)
        parsed = self.parse_global_plan_response(raw_text)

        plan_obj = {
            "schema_version": "global_plan_v5_semantic_only_periodic",
            "created_wall_time_iso": iso_now_local(),
            "created_wall_time_unix": time.time(),
            "planner_model": self.args.planner_model,
            "trigger": trigger,
            "trigger_reason": trigger_reason,
            "reason": parsed["reason"],
            "global_strategy": parsed["global_strategy"],
            "plan_summary": parsed["plan_summary"],
            "local_execution_guidance": parsed["local_execution_guidance"],
            "avoid_strategy": parsed["avoid_strategy"],
            "route_phase": parsed["route_phase"],
            "replan_trigger_hint": parsed["replan_trigger_hint"],
            "requested_cooldown_s": parsed["requested_cooldown_s"],
            "applied_cooldown_s": parsed["applied_cooldown_s"],
            "raw_model_output": parsed["raw_model_output"],
        }
        self.save_global_plan(plan_obj)
        return plan_obj

    def build_local_payload_text(
        self,
        tick: Dict[str, Any],
        runner_state: Dict[str, Any],
        global_plan: Optional[Dict[str, Any]],
    ) -> str:
        payload = {
            "local_execution_mode": True,
            "run_label": self.args.run_label,
            "local_model": self.args.local_model,
            "mission_state": tick.get("mission_state", {}),
            "robot_state": tick.get("robot_state", {}),
            "runner_state": runner_state,
            "global_plan": global_plan,
            "blocked_regions_state": self.load_blocked_regions_state(),
            "export": {
                "export_tick": tick.get("export", {}).get("export_tick"),
                "export_timestamp": tick.get("export", {}).get("export_timestamp"),
                "crop": tick.get("export", {}).get("crop"),
            },
            "local_image_bounds": {
                "width_px": 200,
                "height_px": 200,
                "valid_x_range": [0, 199],
                "valid_y_range": [0, 199],
            },
            "artifact_age_s": {
                "local_vis_png": tick.get("artifact_age_s", {}).get("local_vis_png"),
                "global_crop_vis_png": tick.get("artifact_age_s", {}).get("global_crop_vis_png"),
                "global_full_vis_png": tick.get("artifact_age_s", {}).get("global_full_vis_png"),
                "rgb_left_png": tick.get("artifact_age_s", {}).get("rgb_left_png"),
            },
            "response_contract": {
                "return_json_only": True,
                "required_keys": [
                    "action",
                    "reason",
                    "x1",
                    "y1",
                    "x2",
                    "y2",
                    "free_thresh",
                    "stride",
                    "blocked_x1",
                    "blocked_y1",
                    "blocked_x2",
                    "blocked_y2",
                ],
                "actions": ["monitor", "set_region", "request_global_replan"],
            },
        }
        return (
            "LOCAL EXECUTION PAYLOAD\n\n"
            + json.dumps(payload, indent=2)
            + "\n\nReturn only valid JSON matching the local-executor schema."
        )

    def parse_local_response(self, raw_text: str, tick: Dict[str, Any], runner_state: Dict[str, Any]):
        obj = json.loads(raw_text)
        required = ["action", "reason", "x1", "y1", "x2", "y2", "free_thresh", "stride", "blocked_x1", "blocked_y1", "blocked_x2", "blocked_y2"]
        missing = [k for k in required if k not in obj]
        if missing:
            raise RuntimeError(f"Local model response missing keys: {missing}")

        action = str(obj["action"]).strip()
        reason = str(obj["reason"]).strip()

        if action not in ("monitor", "set_region", "request_global_replan"):
            raise RuntimeError(f"Unsupported local action: {action}")
        if not reason:
            raise RuntimeError("Local model response has empty reason")

        out_px = int(tick.get("export", {}).get("crop", {}).get("out_px", 200))
        max_idx = out_px - 1

        blocked_region = None
        blocked_vals = [obj.get("blocked_x1"), obj.get("blocked_y1"), obj.get("blocked_x2"), obj.get("blocked_y2")]
        if any(v is not None for v in blocked_vals):
            if any(v is None for v in blocked_vals):
                raise RuntimeError("Blocked region fields must be all-null or all-present")
            bx1 = int(obj["blocked_x1"]); by1 = int(obj["blocked_y1"]); bx2 = int(obj["blocked_x2"]); by2 = int(obj["blocked_y2"])
            if not (0 <= bx1 <= max_idx and 0 <= bx2 <= max_idx and 0 <= by1 <= max_idx and 0 <= by2 <= max_idx):
                raise RuntimeError(f"Blocked region out of bounds for local image 0..{max_idx}")
            if bx1 == bx2 or by1 == by2:
                raise RuntimeError("Blocked region has zero width or zero height")
            blocked_region = [bx1, by1, bx2, by2]

        if action in ("monitor", "request_global_replan"):
            if action == "monitor" and runner_state.get("must_choose_new_region", False):
                raise RuntimeError("Model returned monitor even though must_choose_new_region=true")
            return obj, action, None, blocked_region

        for k in ["x1", "y1", "x2", "y2", "free_thresh", "stride"]:
            if obj[k] is None:
                raise RuntimeError(f"Local model returned set_region but {k} is null")

        x1 = int(obj["x1"])
        y1 = int(obj["y1"])
        x2 = int(obj["x2"])
        y2 = int(obj["y2"])
        free_thresh = int(obj["free_thresh"])
        stride = int(obj["stride"])

        if not (0 <= x1 <= max_idx and 0 <= x2 <= max_idx and 0 <= y1 <= max_idx and 0 <= y2 <= max_idx):
            raise RuntimeError(f"Region out of bounds for local image 0..{max_idx}")
        if x1 == x2 or y1 == y2:
            raise RuntimeError("Region has zero width or zero height")
        if stride < 1:
            raise RuntimeError("stride must be >= 1")

        return obj, action, [x1, y1, x2, y2, free_thresh, stride], blocked_region

    def log_line(self, obj: dict):
        with self.cycle_log_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(obj))
            f.write("\n")

    def sleep_until_next_cycle(self, start_wall: float):
        period = 1.0 / float(self.args.tick_hz)
        elapsed = time.time() - start_wall
        remaining = period - elapsed
        if remaining > 0:
            time.sleep(remaining)

    def run(self):
        self.launch_optional_helpers()
        self.reset_startup_state()

        if self.args.planner_mode == "startup_and_replan":
            startup_tick = self.run_prepare_payload()
            startup_state = self.compute_runner_state(startup_tick, None)

            startup_plan = self.plan_global_once(
                tick=startup_tick,
                runner_state=startup_state,
                trigger="startup",
                trigger_reason="initial global plan before local execution",
                previous_global_plan=None,
            )
            self.log_line({
                "phase": "startup_global_plan",
                "wall_time_iso": iso_now_local(),
                "planner_model": self.args.planner_model,
                "global_plan": startup_plan,
                "status": "ok",
            })
            print(f"[run_lvlm_loop] Startup global plan set: {startup_plan['global_strategy']}")
        else:
            print(f"[run_lvlm_loop] Startup planner skipped (planner_mode={self.args.planner_mode})")

        cycle_idx = 0
        while True:
            if self.args.max_cycles is not None and cycle_idx >= self.args.max_cycles:
                print("[run_lvlm_loop] Reached max_cycles, stopping.")
                break

            cycle_start = time.time()
            cycle_log: Dict[str, Any] = {
                "phase": "local_cycle",
                "cycle_index": cycle_idx,
                "cycle_wall_time_iso": iso_now_local(),
                "planner_model": self.args.planner_model,
                "local_model": self.args.local_model,
                "planner_mode": self.args.planner_mode,
                "run_label": self.args.run_label,
                "status": "started",
            }

            try:
                tick = self.run_prepare_payload()
                global_plan = self.load_global_plan()
                runner_state = self.compute_runner_state(tick, global_plan)

                cycle_log["tick_schema_version"] = tick.get("schema_version")
                cycle_log["tick_wall_time_iso"] = tick.get("wall_time_iso")
                cycle_log["tick_export_tick"] = tick.get("export", {}).get("export_tick")
                cycle_log["runner_state"] = runner_state
                cycle_log["global_plan"] = global_plan

                if runner_state["final_goal_reached"]:
                    cycle_log["status"] = "final_goal_reached"
                    self.log_line(cycle_log)
                    print("[run_lvlm_loop] Final goal threshold reached, stopping.")
                    break

                if self.should_trigger_periodic_recheck(global_plan):
                    periodic_plan = self.plan_global_once(
                        tick=tick,
                        runner_state=runner_state,
                        trigger="periodic_recheck",
                        trigger_reason=f"periodic global recheck after {self.args.periodic_global_recheck_s}s",
                        previous_global_plan=global_plan,
                    )
                    cycle_log["periodic_recheck_plan"] = periodic_plan
                    global_plan = periodic_plan
                    runner_state = self.compute_runner_state(tick, global_plan)
                    cycle_log["runner_state_after_periodic_recheck"] = runner_state
                    print(f"[run_lvlm_loop] Cycle {cycle_idx}: periodic global recheck -> {periodic_plan['global_strategy']}")

                response = self.call_json_model(
                    model_name=self.args.local_model,
                    prompt_text=self.local_prompt_text,
                    payload_text=self.build_local_payload_text(
                        tick=tick,
                        runner_state=runner_state,
                        global_plan=global_plan,
                    ),
                    tick=tick,
                )
                raw_text = self.extract_text_output(response)
                parsed_obj, action, region, blocked_region = self.parse_local_response(raw_text, tick, runner_state)

                cycle_log["raw_model_output"] = raw_text
                cycle_log["parsed_model_output"] = parsed_obj
                cycle_log["local_action"] = action

                if blocked_region is not None:
                    blocked_state = self.persist_blocked_region_from_local_box(
                        tick=tick,
                        local_box=blocked_region,
                        reason=parsed_obj.get("reason", "local model marked blocked region"),
                        source="local_model",
                    )
                    cycle_log["blocked_region_local"] = blocked_region
                    cycle_log["blocked_regions_state"] = blocked_state
                    print(f"[run_lvlm_loop] Cycle {cycle_idx}: stored blocked region {blocked_region}")

                if action == "monitor":
                    cycle_log["status"] = "monitor"
                    print(f"[run_lvlm_loop] Cycle {cycle_idx}: monitor ({parsed_obj.get('reason')})")

                elif action == "set_region":
                    assert region is not None
                    self.publisher.publish_region(region)
                    rclpy.spin_once(self.publisher, timeout_sec=0.05)
                    self.last_published_region = region
                    self.last_region_publish_wall_time = time.time()
                    cycle_log["status"] = "published_region"
                    cycle_log["published_region"] = region
                    print(f"[run_lvlm_loop] Cycle {cycle_idx}: published region {region}")

                elif action == "request_global_replan":
                    if self.args.planner_mode == "off":
                        cycle_log["status"] = "replan_requested_but_planner_off"
                        print(f"[run_lvlm_loop] Cycle {cycle_idx}: global replan requested but planner is off")
                    else:
                        cooldown_remaining = runner_state.get("global_plan_cooldown_remaining_s")
                        if cooldown_remaining is not None and cooldown_remaining > 0.0:
                            cycle_log["status"] = "replan_requested_but_cooldown_active"
                            cycle_log["cooldown_remaining_s"] = cooldown_remaining
                            print(
                                f"[run_lvlm_loop] Cycle {cycle_idx}: global replan requested but cooldown active "
                                f"({cooldown_remaining}s remaining)"
                            )
                        else:
                            new_plan = self.plan_global_once(
                                tick=tick,
                                runner_state=runner_state,
                                trigger="local_model_request",
                                trigger_reason=parsed_obj.get("reason", "local model requested global replan"),
                                previous_global_plan=global_plan,
                            )
                            cycle_log["status"] = "global_replan_set"
                            cycle_log["new_global_plan"] = new_plan
                            print(f"[run_lvlm_loop] Cycle {cycle_idx}: set new global plan {new_plan['global_strategy']}")

                self.update_cycle_memory(tick, runner_state)

            except Exception as e:
                cycle_log["status"] = "error"
                cycle_log["error"] = str(e)
                self.log_line(cycle_log)
                print(f"[run_lvlm_loop] Cycle {cycle_idx} ERROR: {e}", file=sys.stderr)
                if self.args.stop_on_error:
                    raise

            self.log_line(cycle_log)
            cycle_idx += 1
            self.sleep_until_next_cycle(cycle_start)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--notes-root", default="~/table_nav2/notes/current")
    ap.add_argument("--scripts-root", default="~/table_nav2/scripts")
    ap.add_argument("--goal-x", type=float, required=True)
    ap.add_argument("--goal-y", type=float, required=True)

    ap.add_argument("--planner-mode", choices=["startup_and_replan", "on_demand", "off"], default="startup_and_replan")
    ap.add_argument("--planner-model", default="gpt-5.4")
    ap.add_argument("--local-model", "--model", dest="local_model", default="gpt-5.4")

    ap.add_argument("--planner-prompt-file", required=True)
    ap.add_argument("--local-prompt-file", "--prompt-file", dest="local_prompt_file", required=True)

    ap.add_argument("--tick-hz", type=float, default=0.2)
    ap.add_argument("--tf-wait", type=float, default=1.0)
    ap.add_argument("--run-label", default="lvlm_run")
    ap.add_argument("--max-cycles", type=int, default=None)
    ap.add_argument("--stop-on-error", action="store_true")
    ap.add_argument("--active-goal-near-thresh", type=float, default=1.0)
    ap.add_argument("--final-goal-reached-thresh", type=float, default=0.5)
    ap.add_argument("--periodic-global-recheck-s", type=float, default=10.0)
    ap.add_argument("--goal-progress-epsilon-m", type=float, default=0.15)

    ap.add_argument("--launch-crops", action="store_true")
    ap.add_argument("--launch-rgb", action="store_true")
    ap.add_argument("--launch-picker", action="store_true")
    ap.add_argument("--helper-warmup-s", type=float, default=3.0)
    ap.add_argument("--export-hz", type=float, default=0.5)
    ap.add_argument("--global-full-hz", type=float, default=0.5)
    ap.add_argument("--crop-m", type=float, default=10.0)
    ap.add_argument("--out-px", type=int, default=200)
    ap.add_argument("--rgb-hz", type=float, default=0.5)
    args = ap.parse_args()

    runner = LVLMLoopRunner(args)
    try:
        runner.run()
    finally:
        runner.shutdown()


if __name__ == "__main__":
    main()
