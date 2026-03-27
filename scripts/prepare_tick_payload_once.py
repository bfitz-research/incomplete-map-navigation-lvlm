#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
import time
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException
from PIL import Image, ImageDraw


def load_json(path: str) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def atomic_write_json(path: str, obj: dict):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def get_mtime(path: str) -> Optional[float]:
    try:
        return os.path.getmtime(path)
    except Exception:
        return None


def iso_now_local():
    return datetime.now().astimezone().isoformat(timespec="seconds")


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_mkdir(path: str):
    os.makedirs(path, exist_ok=True)


def parse_global_sidecar_txt(path: str) -> Optional[Dict[str, Any]]:
    try:
        d = {}
        with open(path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.startswith("w="):
                    parts = line.split()
                    for p in parts:
                        if p.startswith("w="):
                            d["w"] = int(p.split("=")[1])
                        if p.startswith("h="):
                            d["h"] = int(p.split("=")[1])
                        if p.startswith("res="):
                            d["res"] = float(p.split("=")[1])
                if line.startswith("origin=(") and line.endswith(")"):
                    inside = line[len("origin=("):-1]
                    ox, oy = inside.split(",")[:2]
                    d["ox"] = float(ox)
                    d["oy"] = float(oy)
        if all(k in d for k in ("w", "h", "res", "ox", "oy")):
            return d
        return None
    except Exception:
        return None


def open_image_rgb_retry(path: str, retries: int = 5, sleep_s: float = 0.08) -> Image.Image:
    last_err = None
    for _ in range(retries):
        try:
            return Image.open(path).convert("RGB")
        except Exception as e:
            last_err = e
            time.sleep(sleep_s)
    raise last_err


class CropBounds:
    def __init__(self, min_x: float, max_x: float, min_y: float, max_y: float, out_px: int, meters_per_px: float):
        self.min_x = float(min_x)
        self.max_x = float(max_x)
        self.min_y = float(min_y)
        self.max_y = float(max_y)
        self.out_px = int(out_px)
        self.meters_per_px = float(meters_per_px)


def load_crop_bounds(meta: Dict[str, Any]) -> Optional[CropBounds]:
    try:
        crop = meta["crop"]
        b = crop["bounds_render"]
        return CropBounds(
            min_x=float(b["min_x"]),
            max_x=float(b["max_x"]),
            min_y=float(b["min_y"]),
            max_y=float(b["max_y"]),
            out_px=int(crop["out_px"]),
            meters_per_px=float(crop["meters_per_px"]),
        )
    except Exception:
        return None


def map_to_crop_px(x: float, y: float, cb: CropBounds) -> Optional[Tuple[int, int]]:
    mpp = cb.meters_per_px
    u = (x - cb.min_x) / mpp - 0.5
    v = (cb.max_y - y) / mpp - 0.5
    if u < 0 or v < 0 or u >= cb.out_px or v >= cb.out_px:
        return None
    return int(round(u)), int(round(v))


def map_to_global_full_px(x: float, y: float, info: Dict[str, Any]) -> Optional[Tuple[int, int]]:
    ox, oy, res = info["ox"], info["oy"], info["res"]
    w, h = info["w"], info["h"]
    u = (x - ox) / res
    v_from_bottom = (y - oy) / res
    v = (h - 1) - v_from_bottom
    if u < 0 or v < 0 or u >= w or v >= h:
        return None
    return int(round(u)), int(round(v))


def draw_x(draw: ImageDraw.ImageDraw, u: int, v: int, size: int, color, width: int = 3):
    draw.line((u - size, v - size, u + size, v + size), fill=color, width=width)
    draw.line((u - size, v + size, u + size, v - size), fill=color, width=width)


def draw_circle(draw: ImageDraw.ImageDraw, u: int, v: int, r: int, outline, width: int = 3, fill=None):
    bbox = (u - r, v - r, u + r, v + r)
    draw.ellipse(bbox, outline=outline, width=width, fill=fill)




def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def draw_map_box_on_crop(draw: ImageDraw.ImageDraw, x_min: float, y_min: float, x_max: float, y_max: float, cb: CropBounds, outline=(255, 64, 0), width: int = 3):
    p1 = map_to_crop_px(x_min, y_max, cb)
    p2 = map_to_crop_px(x_max, y_min, cb)
    if not p1 or not p2:
        return
    x1, y1 = p1
    x2, y2 = p2
    left = clamp(min(x1, x2), 0, cb.out_px - 1)
    right = clamp(max(x1, x2), 0, cb.out_px - 1)
    top = clamp(min(y1, y2), 0, cb.out_px - 1)
    bottom = clamp(max(y1, y2), 0, cb.out_px - 1)
    if left >= right or top >= bottom:
        return
    draw.rectangle((left, top, right, bottom), outline=outline, width=width)


def draw_map_box_on_full_global(draw: ImageDraw.ImageDraw, x_min: float, y_min: float, x_max: float, y_max: float, info: Dict[str, Any], outline=(255, 64, 0), width: int = 3):
    p1 = map_to_global_full_px(x_min, y_max, info)
    p2 = map_to_global_full_px(x_max, y_min, info)
    if not p1 or not p2:
        return
    left = min(p1[0], p2[0]); right = max(p1[0], p2[0])
    top = min(p1[1], p2[1]); bottom = max(p1[1], p2[1])
    if left >= right or top >= bottom:
        return
    draw.rectangle((left, top, right, bottom), outline=outline, width=width)

def draw_arrow(draw: ImageDraw.ImageDraw, u: int, v: int, yaw_rad: float, length: int, color, width: int = 4):
    tip_x = u + int(round(math.cos(yaw_rad) * length))
    tip_y = v - int(round(math.sin(yaw_rad) * length))

    base_dist = max(10, int(length * 0.55))
    half_width = max(7, int(length * 0.28))

    base_cx = u - int(round(math.cos(yaw_rad) * base_dist * 0.15))
    base_cy = v + int(round(math.sin(yaw_rad) * base_dist * 0.15))

    px = -math.sin(yaw_rad)
    py = -math.cos(yaw_rad)

    left_x = base_cx + int(round(px * half_width))
    left_y = base_cy + int(round(py * half_width))
    right_x = base_cx - int(round(px * half_width))
    right_y = base_cy - int(round(py * half_width))

    draw.polygon([(tip_x, tip_y), (left_x, left_y), (right_x, right_y)], fill=color)


def draw_blind_wedge(img: Image.Image, center: Tuple[int, int], color=(0, 0, 0, 40)):
    return img


def artifact_ages(exports_dict: dict):
    now = time.time()
    ages = {}
    for k, p in exports_dict.items():
        mt = get_mtime(p)
        ages[k] = None if mt is None else round(now - mt, 3)
    return ages


class PayloadPrepNode(Node):
    def __init__(self, notes_root: str, tf_wait_s: float):
        super().__init__("prepare_tick_payload_once")
        self.notes_root = os.path.expanduser(notes_root)
        self.tf_wait_s = float(tf_wait_s)

        self.exports_latest = os.path.join(self.notes_root, "exports", "latest")
        self.state_path = os.path.join(self.notes_root, "state", "markers.json")
        self.global_plan_path = os.path.join(self.notes_root, "state", "global_plan.json")
        self.blocked_regions_path = os.path.join(self.notes_root, "state", "blocked_regions.json")
        self.ticks_latest_dir = os.path.join(self.notes_root, "ticks", "latest")

        safe_mkdir(self.exports_latest)
        safe_mkdir(os.path.dirname(self.state_path))
        safe_mkdir(self.ticks_latest_dir)

        self.meta_path = os.path.join(self.exports_latest, "meta.json")
        self.local_path = os.path.join(self.exports_latest, "local_costmap_crop.png")
        self.global_crop_path = os.path.join(self.exports_latest, "global_costmap_crop.png")
        self.global_full_path = os.path.join(self.exports_latest, "global_costmap.png")
        self.global_full_sidecar = self.global_full_path + ".txt"
        self.rgb_left_path = os.path.join(self.exports_latest, "rgb_left.png")
        self.rgb_right_path = os.path.join(self.exports_latest, "rgb_right.png")

        self.out_local = os.path.join(self.exports_latest, "local_vis.png")
        self.out_global_crop = os.path.join(self.exports_latest, "global_crop_vis.png")
        self.out_global_full = os.path.join(self.exports_latest, "global_full_vis.png")
        self.out_vis_meta = os.path.join(self.exports_latest, "vis_meta.json")
        self.tick_out_path = os.path.join(self.ticks_latest_dir, "tick.json")

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
                return {"x_map": float(x), "y_map": float(y), "yaw_rad": float(yaw)}, None
            except TransformException as e:
                last_err = str(e)
                rclpy.spin_once(self, timeout_sec=0.02)
        return None, last_err

    def render_crop(self, in_path: str, out_path: str, cb: CropBounds, robot_px, ryaw: float, markers: Dict[str, Any]) -> bool:
        if not os.path.exists(in_path):
            return False

        img = open_image_rgb_retry(in_path)
        img = draw_blind_wedge(img, robot_px, color=(0, 0, 0, 40))
        d = ImageDraw.Draw(img)

        draw_arrow(d, robot_px[0], robot_px[1], ryaw, length=22, color=(255, 0, 0), width=4)

        for br in markers.get("_blocked_regions_for_render", []):
            try:
                draw_map_box_on_crop(d, float(br["x_min"]), float(br["y_min"]), float(br["x_max"]), float(br["y_max"]), cb, outline=(255, 64, 0), width=3)
            except Exception:
                pass

        ug = markers.get("ultimate_goal", None)
        if ug and "x" in ug and "y" in ug:
            p = map_to_crop_px(float(ug["x"]), float(ug["y"]), cb)
            if p:
                draw_x(d, p[0], p[1], size=10, color=(255, 140, 0), width=4)

        ag = markers.get("active_goal", None)
        if ag and "x" in ag and "y" in ag:
            p = map_to_crop_px(float(ag["x"]), float(ag["y"]), cb)
            if p:
                draw_x(d, p[0], p[1], size=7, color=(0, 255, 0), width=3)

        for poi in markers.get("pois", []):
            try:
                x = float(poi["x"])
                y = float(poi["y"])
            except Exception:
                continue
            p = map_to_crop_px(x, y, cb)
            if p:
                draw_circle(d, p[0], p[1], r=6, outline=(0, 255, 0), width=3)

        img.save(out_path)
        return True

    def render_global_full(self, rx: float, ry: float, ryaw: float, markers: Dict[str, Any]) -> bool:
        if not (os.path.exists(self.global_full_path) and os.path.exists(self.global_full_sidecar)):
            return False

        info = parse_global_sidecar_txt(self.global_full_sidecar)
        if not info:
            return False

        img = open_image_rgb_retry(self.global_full_path)
        d = ImageDraw.Draw(img)

        rp = map_to_global_full_px(rx, ry, info)
        if rp:
            draw_arrow(d, rp[0], rp[1], ryaw, length=26, color=(255, 0, 0), width=4)

        for br in markers.get("_blocked_regions_for_render", []):
            try:
                draw_map_box_on_full_global(d, float(br["x_min"]), float(br["y_min"]), float(br["x_max"]), float(br["y_max"]), info, outline=(255, 64, 0), width=3)
            except Exception:
                pass

        ug = markers.get("ultimate_goal", None)
        if ug and "x" in ug and "y" in ug:
            p = map_to_global_full_px(float(ug["x"]), float(ug["y"]), info)
            if p:
                draw_x(d, p[0], p[1], size=14, color=(255, 140, 0), width=4)

        ag = markers.get("active_goal", None)
        if ag and "x" in ag and "y" in ag:
            p = map_to_global_full_px(float(ag["x"]), float(ag["y"]), info)
            if p:
                draw_x(d, p[0], p[1], size=10, color=(0, 255, 0), width=3)

        for poi in markers.get("pois", []):
            try:
                x = float(poi["x"])
                y = float(poi["y"])
            except Exception:
                continue
            p = map_to_global_full_px(x, y, info)
            if p:
                draw_circle(d, p[0], p[1], r=8, outline=(0, 255, 0), width=3)

        img.save(self.out_global_full)
        return True

    def run_once(self) -> int:
        meta = load_json(self.meta_path)
        if not meta:
            print(f"[prepare_tick_payload_once] ERROR: missing/invalid meta.json: {self.meta_path}", file=sys.stderr)
            return 2

        cb = load_crop_bounds(meta)
        if cb is None:
            print("[prepare_tick_payload_once] ERROR: failed to parse crop bounds from meta.json", file=sys.stderr)
            return 2

        markers = load_json(self.state_path) or {"ultimate_goal": None, "active_goal": None, "pois": []}
        global_plan = load_json(self.global_plan_path)
        blocked_regions_state = load_json(self.blocked_regions_path) or {"schema_version": "blocked_regions_v1", "regions": []}
        blocked_regions = blocked_regions_state.get("regions", []) if isinstance(blocked_regions_state, dict) else []
        markers["_blocked_regions_for_render"] = blocked_regions

        robot_render = meta.get("robot_pose_render", {"x": 0.0, "y": 0.0, "yaw_rad": 0.0})
        rx = float(robot_render.get("x", 0.0))
        ry = float(robot_render.get("y", 0.0))
        ryaw = float(robot_render.get("yaw_rad", 0.0))

        robot_px = map_to_crop_px(rx, ry, cb)
        if robot_px is None:
            robot_px = (cb.out_px // 2, cb.out_px // 2)

        ok_local = self.render_crop(self.local_path, self.out_local, cb, robot_px, ryaw, markers)
        ok_global_crop = self.render_crop(self.global_crop_path, self.out_global_crop, cb, robot_px, ryaw, markers)
        ok_global_full = self.render_global_full(rx, ry, ryaw, markers)

        vis_meta = {
            "export_tick": meta.get("tick"),
            "export_timestamp": meta.get("timestamp"),
            "rendered_at_wall_time": time.time(),
            "rendered": {
                "local_vis": bool(ok_local),
                "global_crop_vis": bool(ok_global_crop),
                "global_full_vis": bool(ok_global_full),
            },
            "outputs": {
                "local_vis_png": self.out_local,
                "global_crop_vis_png": self.out_global_crop,
                "global_full_vis_png": self.out_global_full,
            },
            "state_path": self.state_path,
            "global_plan_path": self.global_plan_path,
        }
        atomic_write_json(self.out_vis_meta, vis_meta)

        robot_state, tf_err = self.get_robot_pose_map()
        if robot_state is None:
            robot_state = {
                "x_map": rx,
                "y_map": ry,
                "yaw_rad": ryaw,
                "source": "meta_fallback",
            }

        exports = {
            "meta_json": self.meta_path,
            "vis_meta_json": self.out_vis_meta,
            "markers_json": self.state_path,
            "global_plan_json": self.global_plan_path,
            "local_vis_png": self.out_local,
            "global_crop_vis_png": self.out_global_crop,
            "global_full_vis_png": self.out_global_full,
            "rgb_left_png": self.rgb_left_path,
            "rgb_right_png": self.rgb_right_path,
            "local_costmap_crop_png": self.local_path,
            "global_costmap_crop_png": self.global_crop_path,
            "global_costmap_png": self.global_full_path,
            "static_crop_png": os.path.join(self.exports_latest, "static_crop.png"),
            "global_map_png": os.path.join(self.exports_latest, "global_map.png"),
        }

        exports_present = {k: os.path.exists(v) for k, v in exports.items()}
        missing_required = []
        required = [
            "meta_json",
            "markers_json",
            "vis_meta_json",
            "local_vis_png",
            "global_crop_vis_png",
            "global_full_vis_png",
            "rgb_left_png",
        ]
        for k in required:
            if not exports_present.get(k, False):
                missing_required.append(k)

        tick = {
            "schema_version": "tick_once_v4_semantic_only_blocked_regions_no_wedge_orange_goal",
            "wall_time_iso": iso_now_local(),
            "notes_root": self.notes_root,
            "ready_for_lvlm": len(missing_required) == 0,
            "not_ready_reasons": [f"missing_required_export:{k}" for k in missing_required],
            "export": {
                "export_tick": meta.get("tick", None),
                "export_timestamp": meta.get("timestamp", None),
                "robot_pose_render": meta.get("robot_pose_render", None),
                "crop": meta.get("crop", None),
            },
            "render": vis_meta,
            "robot_state": robot_state,
            "mission_state": {
                "ultimate_goal": markers.get("ultimate_goal", None),
                "active_goal": markers.get("active_goal", None),
                "pois": markers.get("pois", []),
            },
            "blocked_regions_state": {
                "count": len(blocked_regions),
                "regions": blocked_regions,
            },
            "global_plan_state": {
                "present": global_plan is not None,
                "summary": None if global_plan is None else {
                    "global_strategy": global_plan.get("global_strategy"),
                    "route_phase": global_plan.get("route_phase"),
                    "reason": global_plan.get("reason"),
                },
            },
            "lvlm_inputs": {
                "primary_local_map_png": exports["local_vis_png"],
                "primary_global_crop_png": exports["global_crop_vis_png"],
                "primary_global_full_png": exports["global_full_vis_png"],
                "primary_rgb_png": exports["rgb_left_png"],
                "optional_rgb_right_png": exports["rgb_right_png"],
                "input_policy": {
                    "rgb_default_camera": "left",
                    "rgb_right_on_demand": True,
                    "maps_use_vis_outputs": True,
                    "anchor_markers_drawn": False,
                    "ultimate_goal_color": "orange",
                    "active_goal_color": "green",
                    "blind_wedge_drawn": False,
                    "blocked_regions_drawn": True,
                },
            },
            "exports": exports,
            "exports_present": exports_present,
            "artifact_age_s": artifact_ages(exports),
            "warnings": [],
        }

        if tf_err:
            tick["warnings"].append(f"TF map<-base_link unavailable; used meta fallback: {tf_err}")

        atomic_write_json(self.tick_out_path, tick)

        if tick["ready_for_lvlm"]:
            print(f"[prepare_tick_payload_once] OK: wrote complete payload -> {self.tick_out_path}")
            return 0

        print(f"[prepare_tick_payload_once] NOT READY: {tick['not_ready_reasons']} -> {self.tick_out_path}", file=sys.stderr)
        return 3


def main():
    ap = argparse.ArgumentParser()
    default_notes_root = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "notes",
        "current",
    )
    ap.add_argument("--notes-root", default=default_notes_root)
    ap.add_argument("--tf-wait", type=float, default=0.25)
    args = ap.parse_args()

    rclpy.init()
    node = PayloadPrepNode(notes_root=args.notes_root, tf_wait_s=args.tf_wait)
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
