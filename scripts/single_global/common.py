#!/usr/bin/env python3
import base64
import io
import json
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from PIL import Image, ImageDraw, ImageFont, ImageFilter


def iso_now_local() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def ensure_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)


def load_json(path: Path, default: Optional[Any] = None) -> Any:
    try:
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def atomic_write_json(path: Path, obj: Any) -> None:
    ensure_dir(path.parent)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def get_mtime(path: Path) -> Optional[float]:
    try:
        return path.stat().st_mtime
    except Exception:
        return None


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




def build_safe_waypoint_mask(
    global_costmap_path: Path,
    *,
    free_threshold: int = 245,
    erosion_radius_px: int = 8,
) -> Image.Image:
    with Image.open(global_costmap_path) as im:
        gray = im.convert("L")
    mask = gray.point(lambda p: 255 if int(p) >= int(free_threshold) else 0, mode="L")
    if int(erosion_radius_px) > 0:
        size = int(erosion_radius_px) * 2 + 1
        mask = mask.filter(ImageFilter.MinFilter(size=size))
    return mask


def is_safe_waypoint_pixel(mask: Image.Image, u: int, v: int) -> bool:
    w, h = mask.size
    if u < 0 or v < 0 or u >= w or v >= h:
        return False
    return int(mask.getpixel((int(u), int(v)))) > 0


def nearest_safe_waypoint_pixel(mask: Image.Image, u: int, v: int, max_radius_px: int = 25) -> Optional[Tuple[int, int, int]]:
    w, h = mask.size
    if w <= 0 or h <= 0:
        return None
    u = int(u)
    v = int(v)
    if is_safe_waypoint_pixel(mask, u, v):
        return (u, v, 0)
    best = None
    best_d2 = None
    max_r = max(0, int(max_radius_px))
    for r in range(1, max_r + 1):
        min_u = max(0, u - r)
        max_u = min(w - 1, u + r)
        min_v = max(0, v - r)
        max_v = min(h - 1, v + r)
        for vv in range(min_v, max_v + 1):
            for uu in range(min_u, max_u + 1):
                if abs(uu - u) != r and abs(vv - v) != r:
                    continue
                if int(mask.getpixel((uu, vv))) <= 0:
                    continue
                d2 = (uu - u) * (uu - u) + (vv - v) * (vv - v)
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (uu, vv, r)
        if best is not None:
            return best
    return None


def waypoint_mask_debug(mask: Image.Image, u: int, v: int) -> Dict[str, Any]:
    w, h = mask.size
    return {
        "u": int(u),
        "v": int(v),
        "in_bounds": bool(0 <= int(u) < w and 0 <= int(v) < h),
        "safe": bool(is_safe_waypoint_pixel(mask, int(u), int(v))),
        "mask_w": int(w),
        "mask_h": int(h),
    }

def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def euclidean_xy(a: Optional[Dict[str, Any]], b: Optional[Dict[str, Any]]) -> Optional[float]:
    if not a or not b:
        return None
    try:
        ax = float(a.get("x", a.get("x_map")))
        ay = float(a.get("y", a.get("y_map")))
        bx = float(b.get("x", b.get("x_map")))
        by = float(b.get("y", b.get("y_map")))
    except Exception:
        return None
    return math.hypot(ax - bx, ay - by)


@dataclass
class CropBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    out_px: int
    meters_per_px: float


@dataclass
class GlobalImageInfo:
    w: int
    h: int
    res: float
    ox: float
    oy: float

    @classmethod
    def from_sidecar(cls, path: Path) -> Optional["GlobalImageInfo"]:
        try:
            vals: Dict[str, Any] = {}
            with path.open("r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if line.startswith("w="):
                        for part in line.split():
                            k, v = part.split("=", 1)
                            vals[k] = v
                    elif line.startswith("origin="):
                        inner = line[len("origin="):].strip().strip("()")
                        ox, oy = inner.split(",")
                        vals["ox"] = ox
                        vals["oy"] = oy
            return cls(
                w=int(vals["w"]),
                h=int(vals["h"]),
                res=float(vals["res"]),
                ox=float(vals["ox"]),
                oy=float(vals["oy"]),
            )
        except Exception:
            return None

    def map_to_px(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        u = (x - self.ox) / self.res
        v_from_bottom = (y - self.oy) / self.res
        v = (self.h - 1) - v_from_bottom
        if u < 0 or v < 0 or u >= self.w or v >= self.h:
            return None
        return int(round(u)), int(round(v))

    def px_to_map(self, u: int, v: int) -> Tuple[float, float]:
        x = self.ox + (float(u) + 0.5) * self.res
        y = self.oy + (float(self.h - 1 - v) + 0.5) * self.res
        return x, y


def load_crop_bounds(meta: Dict[str, Any]) -> Optional[CropBounds]:
    try:
        crop = meta["crop"]
        bounds = crop["bounds_render"]
        return CropBounds(
            min_x=float(bounds["min_x"]),
            max_x=float(bounds["max_x"]),
            min_y=float(bounds["min_y"]),
            max_y=float(bounds["max_y"]),
            out_px=int(crop["out_px"]),
            meters_per_px=float(crop["meters_per_px"]),
        )
    except Exception:
        return None


def map_to_crop_px(x: float, y: float, cb: CropBounds) -> Optional[Tuple[int, int]]:
    u = (x - cb.min_x) / cb.meters_per_px - 0.5
    v = (cb.max_y - y) / cb.meters_per_px - 0.5
    if u < 0 or v < 0 or u >= cb.out_px or v >= cb.out_px:
        return None
    return int(round(u)), int(round(v))


def open_image_rgb_retry(path: Path, retries: int = 6, sleep_s: float = 0.08) -> Image.Image:
    last_err = None
    for _ in range(retries):
        try:
            with Image.open(path) as im:
                return im.convert("RGB")
        except Exception as e:
            last_err = e
            time.sleep(sleep_s)
    raise RuntimeError(f"Failed to open image {path}: {last_err}")


def draw_x(draw: ImageDraw.ImageDraw, u: int, v: int, size: int, color, width: int = 3):
    draw.line((u - size, v - size, u + size, v + size), fill=color, width=width)
    draw.line((u - size, v + size, u + size, v - size), fill=color, width=width)


def draw_circle(draw: ImageDraw.ImageDraw, u: int, v: int, r: int, outline, width: int = 3, fill=None):
    draw.ellipse((u - r, v - r, u + r, v + r), outline=outline, width=width, fill=fill)




def default_recent_route_rejections() -> Dict[str, Any]:
    return {
        "schema_version": "recent_route_rejections_v1",
        "updated_wall_time_iso": iso_now_local(),
        "replan_counter": 0,
        "rejections": [],
    }


def prune_recent_route_rejections(obj: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    base = default_recent_route_rejections()
    if not isinstance(obj, dict):
        return base
    replans = int(obj.get("replan_counter", 0))
    kept: List[Dict[str, Any]] = []
    for rej in obj.get("rejections", []):
        if not isinstance(rej, dict):
            continue
        remaining = rej.get("replans_remaining")
        try:
            remaining = int(remaining)
        except Exception:
            remaining = None
        if remaining is not None and remaining <= 0:
            continue
        kept.append(rej)
    return {
        "schema_version": "recent_route_rejections_v1",
        "updated_wall_time_iso": iso_now_local(),
        "replan_counter": replans,
        "rejections": kept,
    }


def decrement_recent_route_rejections(obj: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    pruned = prune_recent_route_rejections(obj)
    out = {
        "schema_version": "recent_route_rejections_v1",
        "updated_wall_time_iso": iso_now_local(),
        "replan_counter": int(pruned.get("replan_counter", 0)) + 1,
        "rejections": [],
    }
    for rej in pruned.get("rejections", []):
        item = dict(rej)
        try:
            item["replans_remaining"] = int(item.get("replans_remaining", 0)) - 1
        except Exception:
            item["replans_remaining"] = 0
        if item["replans_remaining"] > 0:
            out["rejections"].append(item)
    return out


def draw_recent_route_rejections(
    img: Image.Image,
    global_info: GlobalImageInfo,
    rejections_state: Optional[Dict[str, Any]],
) -> None:
    if not isinstance(rejections_state, dict):
        return
    draw = ImageDraw.Draw(img, "RGBA")
    font = load_default_font()
    for idx, rej in enumerate(rejections_state.get("rejections", []), start=1):
        if not isinstance(rej, dict):
            continue
        try:
            u = int(rej["center_u"])
            v = int(rej["center_v"])
            r = int(rej.get("radius_px", 30))
        except Exception:
            continue
        if u < 0 or v < 0 or u >= global_info.w or v >= global_info.h:
            continue
        draw.ellipse((u - r, v - r, u + r, v + r), outline=(255, 0, 255, 220), width=3, fill=(255, 0, 255, 35))
        if font is not None:
            ttl = rej.get("replans_remaining")
            ttl_s = "?" if ttl is None else str(ttl)
            label = f"rej{idx}:{ttl_s}"
            tw = 6 * len(label)
            draw.rectangle((u + r + 4, v - 10, u + r + 10 + tw, v + 6), fill=(255, 255, 255, 220))
            draw.text((u + r + 6, v - 9), label, fill=(120, 0, 120), font=font)


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


def remaining_waypoints(queue_state: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not isinstance(queue_state, dict):
        return []
    waypoints = queue_state.get("waypoints", [])
    if not isinstance(waypoints, list):
        return []
    return [wp for wp in waypoints if str(wp.get("status", "pending")) in ("pending", "active")]


def active_waypoint(queue_state: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for wp in remaining_waypoints(queue_state):
        if str(wp.get("status")) == "active":
            return wp
    return None


def load_default_font():
    try:
        return ImageFont.load_default()
    except Exception:
        return None


def draw_numbered_waypoint_queue(
    img: Image.Image,
    global_info: GlobalImageInfo,
    robot_xy: Optional[Tuple[float, float]],
    queue_state: Optional[Dict[str, Any]],
) -> None:
    draw = ImageDraw.Draw(img)
    font = load_default_font()
    remaining = remaining_waypoints(queue_state)
    if not remaining:
        return

    pts: List[Tuple[int, int]] = []
    if robot_xy is not None:
        rp = global_info.map_to_px(robot_xy[0], robot_xy[1])
        if rp:
            pts.append(rp)

    for wp in remaining:
        try:
            p = global_info.map_to_px(float(wp["x_map"]), float(wp["y_map"]))
        except Exception:
            p = None
        if p:
            pts.append(p)

    if len(pts) >= 2:
        draw.line(pts, fill=(0, 220, 255), width=4)

    for idx, wp in enumerate(remaining, start=1):
        try:
            p = global_info.map_to_px(float(wp["x_map"]), float(wp["y_map"]))
        except Exception:
            p = None
        if not p:
            continue
        u, v = p
        status = str(wp.get("status", "pending"))
        if status == "active":
            fill = (0, 255, 0)
            outline = (0, 120, 0)
            r = 11
        else:
            fill = (0, 220, 255)
            outline = (0, 120, 150)
            r = 9
        draw_circle(draw, u, v, r=r, outline=outline, width=3, fill=fill)
        if font is not None:
            text = str(idx)
            tw = 6 * len(text)
            th = 11
            draw.rectangle((u + 10, v - 12, u + 10 + tw + 4, v - 12 + th + 2), fill=(255, 255, 255))
            draw.text((u + 12, v - 11), text, fill=(0, 0, 0), font=font)


def summarize_waypoints_for_payload(queue_state: Optional[Dict[str, Any]], limit: int = 6) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for wp in remaining_waypoints(queue_state)[:limit]:
        item = {
            "id": wp.get("id"),
            "label": wp.get("label"),
            "u": wp.get("u"),
            "v": wp.get("v"),
            "x_map": wp.get("x_map"),
            "y_map": wp.get("y_map"),
            "status": wp.get("status"),
        }
        out.append(item)
    return out


def artifact_ages(path_map: Dict[str, Path]) -> Dict[str, Optional[float]]:
    now = time.time()
    out: Dict[str, Optional[float]] = {}
    for key, path in path_map.items():
        mt = get_mtime(path)
        out[key] = None if mt is None else round(now - mt, 3)
    return out
