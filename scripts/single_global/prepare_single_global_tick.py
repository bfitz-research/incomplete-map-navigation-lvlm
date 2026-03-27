#!/usr/bin/env python3
import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from PIL import ImageDraw

from common import (
    GlobalImageInfo,
    active_waypoint,
    artifact_ages,
    atomic_write_json,
    draw_arrow,
    draw_numbered_waypoint_queue,
    draw_recent_route_rejections,
    draw_x,
    prune_recent_route_rejections,
    ensure_dir,
    iso_now_local,
    load_crop_bounds,
    load_json,
    map_to_crop_px,
    open_image_rgb_retry,
    summarize_waypoints_for_payload,
    yaw_from_quat,
)


class PayloadPrepNode(Node):
    def __init__(self, notes_root: Path, tf_wait_s: float, include_local_crop: bool):
        super().__init__("single_global_tick_prep")
        self.notes_root = notes_root
        self.tf_wait_s = float(tf_wait_s)
        self.include_local_crop = bool(include_local_crop)

        self.exports_latest = self.notes_root / "exports" / "latest"
        self.state_dir = self.notes_root / "state"
        self.ticks_dir = self.notes_root / "ticks" / "latest"
        ensure_dir(self.ticks_dir)

        self.meta_path = self.exports_latest / "meta.json"
        self.markers_path = self.state_dir / "markers.json"
        self.queue_path = self.state_dir / "single_global_waypoint_queue.json"
        self.runner_state_path = self.state_dir / "single_global_runner_state.json"
        self.planner_state_path = self.state_dir / "single_global_last_planner_response.json"
        self.rejections_path = self.state_dir / "recent_route_rejections.json"

        self.global_costmap_path = self.exports_latest / "global_costmap.png"
        self.global_sidecar_path = self.exports_latest / "global_costmap.png.txt"
        self.local_costmap_crop_path = self.exports_latest / "local_costmap_crop.png"
        self.rgb_left_path = self.exports_latest / "rgb_left.png"
        self.rgb_right_path = self.exports_latest / "rgb_right.png"

        self.out_global_full = self.exports_latest / "single_global_full_vis.png"
        self.out_local_vis = self.exports_latest / "single_global_local_vis.png"
        self.out_vis_meta = self.exports_latest / "single_global_vis_meta.json"
        self.out_tick = self.ticks_dir / "single_global_tick.json"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_robot_pose_map(self):
        deadline = time.time() + self.tf_wait_s
        last_err = None
        while time.time() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                q = tf.transform.rotation
                yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
                return {"x_map": float(x), "y_map": float(y), "yaw_rad": float(yaw), "source": "tf"}, None
            except TransformException as e:
                last_err = str(e)
                rclpy.spin_once(self, timeout_sec=0.05)
        return None, last_err

    def render_global_overlay(
        self,
        robot_state: Dict[str, Any],
        markers: Dict[str, Any],
        queue_state: Dict[str, Any],
        rejections_state: Dict[str, Any],
    ) -> Optional[GlobalImageInfo]:
        if not (self.global_costmap_path.exists() and self.global_sidecar_path.exists()):
            return None

        info = GlobalImageInfo.from_sidecar(self.global_sidecar_path)
        if info is None:
            return None

        img = open_image_rgb_retry(self.global_costmap_path)
        draw = ImageDraw.Draw(img)

        rp = info.map_to_px(float(robot_state["x_map"]), float(robot_state["y_map"]))
        if rp is not None:
            draw_arrow(draw, rp[0], rp[1], float(robot_state["yaw_rad"]), length=26, color=(255, 0, 0), width=4)

        ug = markers.get("ultimate_goal")
        if isinstance(ug, dict) and "x" in ug and "y" in ug:
            p = info.map_to_px(float(ug["x"]), float(ug["y"]))
            if p is not None:
                draw_x(draw, p[0], p[1], size=14, color=(255, 140, 0), width=4)

        ag = active_waypoint(queue_state)
        if ag is not None:
            p = info.map_to_px(float(ag["x_map"]), float(ag["y_map"]))
            if p is not None:
                draw_x(draw, p[0], p[1], size=10, color=(0, 255, 0), width=3)

        draw_numbered_waypoint_queue(
            img=img,
            global_info=info,
            robot_xy=(float(robot_state["x_map"]), float(robot_state["y_map"])),
            queue_state=queue_state,
        )
        draw_recent_route_rejections(img=img, global_info=info, rejections_state=rejections_state)
        img.save(self.out_global_full)
        return info

    def render_local_overlay(
        self,
        robot_state: Dict[str, Any],
        markers: Dict[str, Any],
        queue_state: Dict[str, Any],
        meta: Dict[str, Any],
    ) -> bool:
        if not (self.include_local_crop and self.local_costmap_crop_path.exists()):
            return False
        cb = load_crop_bounds(meta)
        if cb is None:
            return False
        img = open_image_rgb_retry(self.local_costmap_crop_path)
        draw = ImageDraw.Draw(img)
        rp = map_to_crop_px(float(robot_state["x_map"]), float(robot_state["y_map"]), cb)
        if rp is not None:
            draw_arrow(draw, rp[0], rp[1], float(robot_state["yaw_rad"]), length=22, color=(255, 0, 0), width=4)
        ug = markers.get("ultimate_goal")
        if isinstance(ug, dict) and "x" in ug and "y" in ug:
            p = map_to_crop_px(float(ug["x"]), float(ug["y"]), cb)
            if p is not None:
                draw_x(draw, p[0], p[1], size=10, color=(255, 140, 0), width=4)
        for wp in queue_state.get("waypoints", []):
            if str(wp.get("status")) not in ("pending", "active"):
                continue
            p = map_to_crop_px(float(wp["x_map"]), float(wp["y_map"]), cb)
            if p is None:
                continue
            draw_x(
                draw,
                p[0],
                p[1],
                size=7 if str(wp.get("status")) == "active" else 5,
                color=(0, 255, 0) if str(wp.get("status")) == "active" else (0, 220, 255),
                width=3,
            )
        img.save(self.out_local_vis)
        return True

    def run_once(self) -> int:
        meta = load_json(self.meta_path)
        if not isinstance(meta, dict):
            print(f"[prepare_single_global_tick] ERROR: missing/invalid meta.json: {self.meta_path}", file=sys.stderr)
            return 2

        markers = load_json(self.markers_path, default={"ultimate_goal": None, "active_goal": None, "pois": []})
        queue_state = load_json(self.queue_path, default={"schema_version": "single_global_waypoint_queue_v1", "waypoints": []})
        runner_state = load_json(self.runner_state_path, default=None)
        planner_state = load_json(self.planner_state_path, default=None)
        rejections_state = prune_recent_route_rejections(load_json(self.rejections_path, default=None))

        robot_state, tf_err = self.get_robot_pose_map()
        if robot_state is None:
            robot_render = meta.get("robot_pose_render", {"x": 0.0, "y": 0.0, "yaw_rad": 0.0})
            robot_state = {
                "x_map": float(robot_render.get("x", 0.0)),
                "y_map": float(robot_render.get("y", 0.0)),
                "yaw_rad": float(robot_render.get("yaw_rad", 0.0)),
                "source": "meta_fallback",
            }

        global_info = self.render_global_overlay(robot_state=robot_state, markers=markers, queue_state=queue_state, rejections_state=rejections_state)
        ok_local = self.render_local_overlay(robot_state=robot_state, markers=markers, queue_state=queue_state, meta=meta)

        vis_meta = {
            "rendered_at_wall_time_iso": iso_now_local(),
            "rendered": {
                "single_global_full_vis": bool(global_info is not None),
                "single_global_local_vis": bool(ok_local),
            },
            "outputs": {
                "single_global_full_vis_png": str(self.out_global_full),
                "single_global_local_vis_png": str(self.out_local_vis),
            },
        }
        atomic_write_json(self.out_vis_meta, vis_meta)

        exports = {
            "meta_json": self.meta_path,
            "markers_json": self.markers_path,
            "queue_json": self.queue_path,
            "runner_state_json": self.runner_state_path,
            "planner_state_json": self.planner_state_path,
            "recent_route_rejections_json": self.rejections_path,
            "global_costmap_png": self.global_costmap_path,
            "global_costmap_sidecar_txt": self.global_sidecar_path,
            "single_global_full_vis_png": self.out_global_full,
            "single_global_local_vis_png": self.out_local_vis,
            "single_global_vis_meta_json": self.out_vis_meta,
            "rgb_left_png": self.rgb_left_path,
            "rgb_right_png": self.rgb_right_path,
            "local_costmap_crop_png": self.local_costmap_crop_path,
        }
        exports_present = {k: p.exists() for k, p in exports.items()}
        ready_reasons = []
        if not exports_present["single_global_full_vis_png"]:
            ready_reasons.append("missing single_global_full_vis.png")
        if not exports_present["rgb_left_png"]:
            ready_reasons.append("missing rgb_left.png")
        if self.include_local_crop and not exports_present["single_global_local_vis_png"]:
            ready_reasons.append("local crop requested but single_global_local_vis.png missing")

        tick = {
            "schema_version": "single_global_tick_v1",
            "wall_time_iso": iso_now_local(),
            "ready_for_lvlm": len(ready_reasons) == 0,
            "not_ready_reasons": ready_reasons,
            "notes_root": str(self.notes_root),
            "robot_state": robot_state,
            "mission_state": {
                "ultimate_goal": markers.get("ultimate_goal"),
                "active_goal": markers.get("active_goal"),
                "pois": markers.get("pois", []),
            },
            "queue_state": {
                "schema_version": queue_state.get("schema_version"),
                "plan_instance_id": queue_state.get("plan_instance_id"),
                "remaining_count": len([wp for wp in queue_state.get("waypoints", []) if str(wp.get("status")) in ("pending", "active")]),
                "active_waypoint": active_waypoint(queue_state),
                "remaining_waypoints": summarize_waypoints_for_payload(queue_state),
                "last_planner_action": queue_state.get("last_planner_action"),
                "goal_ratified": bool(queue_state.get("goal_ratified", False)),
            },
            "runner_state": runner_state,
            "last_planner_response": planner_state,
            "recent_route_rejections": rejections_state,
            "planner_inputs": {
                "primary_global_full_png": str(self.out_global_full),
                "primary_rgb_png": str(self.rgb_left_path),
                "optional_rgb_right_png": str(self.rgb_right_path),
                "optional_local_crop_png": str(self.out_local_vis) if ok_local else None,
                "input_policy": {
                    "global_overlay_primary": True,
                    "rgb_scene_status": True,
                    "local_crop_optional": bool(self.include_local_crop),
                    "ultimate_goal_color": "orange",
                    "active_goal_color": "green",
                    "queued_waypoint_color": "cyan",
                    "robot_color": "red",
                    "recent_rejection_region_color": "magenta",
                },
            },
            "global_image_geometry": None if global_info is None else {
                "w": global_info.w,
                "h": global_info.h,
                "res": global_info.res,
                "origin_x": global_info.ox,
                "origin_y": global_info.oy,
                "pixel_contract": {
                    "u_axis": "right",
                    "v_axis": "down",
                    "u_range": [0, global_info.w - 1],
                    "v_range": [0, global_info.h - 1],
                },
            },
            "exports": {k: str(v) for k, v in exports.items()},
            "exports_present": exports_present,
            "artifact_age_s": artifact_ages(exports),
            "warnings": [] if tf_err is None else [f"TF map<-base_link unavailable; used meta fallback: {tf_err}"],
        }
        atomic_write_json(self.out_tick, tick)
        print(f"[prepare_single_global_tick] wrote {self.out_tick}")
        return 0


def main():
    ap = argparse.ArgumentParser()
    repo_root = Path(__file__).resolve().parents[2]
    ap.add_argument("--notes-root", default=str(repo_root / "notes" / "current"))
    ap.add_argument("--tf-wait", type=float, default=1.0)
    ap.add_argument("--include-local-crop", action="store_true")
    args = ap.parse_args()

    notes_root = Path(args.notes_root).expanduser().resolve()
    rclpy.init()
    node = PayloadPrepNode(notes_root=notes_root, tf_wait_s=args.tf_wait, include_local_crop=bool(args.include_local_crop))
    try:
        rc = node.run_once()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
