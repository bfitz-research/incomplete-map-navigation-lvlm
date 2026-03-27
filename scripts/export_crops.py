#!/usr/bin/env python3
import argparse
import json
import math
import os
import time
from datetime import datetime
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterNotDeclaredException
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

from PIL import Image


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def atomic_save_pil_image(img: Image.Image, path: str):
    tmp = path + ".tmp.png"
    img.save(tmp)
    os.replace(tmp, path)


def atomic_write_text(path: str, text: str):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
    os.replace(tmp, path)


def render_crop_vectorized(
    grid: OccupancyGrid,
    tf_to_grid: Optional[Tuple[float, float, float]],  # (tx, ty, yaw) render_frame -> grid_frame
    min_x: float, max_x: float,
    min_y: float, max_y: float,
    out_px: int,
    mode: str,
) -> np.ndarray:
    info = grid.info
    res = float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)
    Wg = int(info.width)
    Hg = int(info.height)

    data = np.asarray(grid.data, dtype=np.int16).reshape((Hg, Wg))

    W = int(out_px)
    H = int(out_px)
    meters_per_px = (max_x - min_x) / W

    u = np.arange(W, dtype=np.float32)
    v = np.arange(H, dtype=np.float32)
    x_r = min_x + (u + 0.5) * meters_per_px
    y_r = max_y - (v + 0.5) * meters_per_px

    Xr, Yr = np.meshgrid(x_r, y_r)

    if tf_to_grid is not None:
        tx, ty, yaw = tf_to_grid
        c = math.cos(yaw)
        s = math.sin(yaw)
        Xf = tx + c * Xr - s * Yr
        Yf = ty + s * Xr + c * Yr
    else:
        Xf = Xr
        Yf = Yr

    ix = np.floor((Xf - ox) / res).astype(np.int32)
    iy = np.floor((Yf - oy) / res).astype(np.int32)

    inside = (ix >= 0) & (iy >= 0) & (ix < Wg) & (iy < Hg)
    out = np.full((H, W), 127, dtype=np.float32)

    sampled = data[iy[inside], ix[inside]].astype(np.float32)

    if mode == "static":
        known = sampled >= 0
        pix = np.full(sampled.shape, 127.0, dtype=np.float32)
        pix[known] = 255.0 - (sampled[known] * (255.0 / 100.0))
        out[inside] = np.clip(pix, 0.0, 255.0)
    else:
        known = sampled >= 0
        pix = np.full(sampled.shape, 127.0, dtype=np.float32)
        pix[known] = 255.0 - sampled[known]
        out[inside] = np.clip(pix, 0.0, 255.0)

    return out.astype(np.uint8)


def costmap_to_u8(grid: OccupancyGrid) -> np.ndarray:
    """
    Match export_global_costmap.py behavior:
      unknown (-1) -> 127
      known -> 255 - value (scaled if 0..100)
    """
    W = int(grid.info.width)
    H = int(grid.info.height)
    data = np.asarray(grid.data, dtype=np.int16).reshape((H, W))

    out = np.full((H, W), 127, dtype=np.float32)
    known = data >= 0
    if not np.any(known):
        return out.astype(np.uint8)

    vmax = float(np.max(data[known]))
    if vmax <= 100.0:
        val = data.astype(np.float32) * (255.0 / 100.0)
    else:
        val = data.astype(np.float32)

    pix = 255.0 - np.clip(val, 0.0, 255.0)
    out[known] = pix[known]
    return out.astype(np.uint8)


class CropExporter(Node):
    def __init__(
        self,
        map_topic: str,
        local_costmap_topic: str,
        global_costmap_topic: str,
        render_frame: str,
        robot_frame: str,
        out_dir: str,
        crop_m: float,
        out_px: int,
        hz: float,
        global_full_hz: float,
        use_sim_time: bool,
        save_history: bool,
        verbose_skips: bool,
    ):
        super().__init__("crop_exporter")

        try:
            self.declare_parameter("use_sim_time", use_sim_time)
        except ParameterAlreadyDeclaredException:
            pass
        try:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        except ParameterNotDeclaredException:
            self.declare_parameter("use_sim_time", use_sim_time)
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])

        self.map_topic = map_topic
        self.local_costmap_topic = local_costmap_topic
        self.global_costmap_topic = global_costmap_topic
        self.render_frame = render_frame
        self.robot_frame = robot_frame
        self.out_dir = os.path.expanduser(out_dir)
        self.crop_m = float(crop_m)
        self.out_px = int(out_px)
        self.hz = float(hz)
        self.global_full_hz = float(global_full_hz) if global_full_hz > 0 else 0.0
        self.save_history = bool(save_history)
        self.verbose_skips = bool(verbose_skips)

        os.makedirs(self.out_dir, exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, "latest"), exist_ok=True)

        self.map_msg: Optional[OccupancyGrid] = None
        self.local_costmap_msg: Optional[OccupancyGrid] = None
        self.global_costmap_msg: Optional[OccupancyGrid] = None

        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        cost_qos = QoSProfile(depth=1)
        cost_qos.durability = DurabilityPolicy.VOLATILE
        cost_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, map_qos)
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self.on_local_costmap, cost_qos)
        self.create_subscription(OccupancyGrid, self.global_costmap_topic, self.on_global_costmap, cost_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tick = 0
        self.last_save_wall = None
        self.last_full_global_save_wall = 0.0
        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

        self.get_logger().info(f"use_sim_time={use_sim_time}")
        self.get_logger().info(f"render_frame={self.render_frame} robot_frame={self.robot_frame}")
        self.get_logger().info(f"map_topic={self.map_topic} (TRANSIENT_LOCAL subscriber)")
        self.get_logger().info(f"local_costmap_topic={self.local_costmap_topic}")
        self.get_logger().info(f"global_costmap_topic={self.global_costmap_topic}")
        self.get_logger().info(f"crop={self.crop_m}m out_px={self.out_px} hz={self.hz}")
        self.get_logger().info(f"global_full_hz={self.global_full_hz}")
        self.get_logger().info(f"out_dir={self.out_dir} (writes exports/latest/)")
        self.get_logger().info(f"save_history={self.save_history}")

    def on_map(self, msg: OccupancyGrid):
        self.map_msg = msg

    def on_local_costmap(self, msg: OccupancyGrid):
        self.local_costmap_msg = msg

    def on_global_costmap(self, msg: OccupancyGrid):
        self.global_costmap_msg = msg

    def tf_render_to(self, target_frame: str) -> Optional[Tuple[float, float, float]]:
        if target_frame == self.render_frame:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, self.render_frame, rclpy.time.Time())
        except TransformException:
            return None
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return (tx, ty, yaw)

    def robot_pose_in_render(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(self.render_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    def write_outputs(self, base_dir: str, static_img: Image.Image, local_img: Image.Image, global_img: Image.Image, meta: dict):
        os.makedirs(base_dir, exist_ok=True)

        atomic_save_pil_image(static_img, os.path.join(base_dir, "static_crop.png"))
        atomic_save_pil_image(local_img, os.path.join(base_dir, "local_costmap_crop.png"))
        atomic_save_pil_image(global_img, os.path.join(base_dir, "global_costmap_crop.png"))

        meta_path = os.path.join(base_dir, "meta.json")
        tmp = meta_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
            f.write("\n")
        os.replace(tmp, meta_path)

    def write_full_global_outputs(self, base_dir: str, msg: OccupancyGrid):
        os.makedirs(base_dir, exist_ok=True)

        u8 = costmap_to_u8(msg)
        u8 = np.flipud(u8)

        out_path = os.path.join(base_dir, "global_costmap.png")
        img = Image.fromarray(u8, mode="L")
        atomic_save_pil_image(img, out_path)

        sidecar = out_path + ".txt"
        stamp = msg.header.stamp
        sidecar_text = (
            f"topic={self.global_costmap_topic}\n"
            f"frame_id={msg.header.frame_id}\n"
            f"stamp={stamp.sec}.{stamp.nanosec}\n"
            f"w={msg.info.width} h={msg.info.height} res={msg.info.resolution}\n"
            f"origin=({msg.info.origin.position.x},{msg.info.origin.position.y})\n"
            f"saved_wall={datetime.now().isoformat()}\n"
        )
        atomic_write_text(sidecar, sidecar_text)

    def maybe_write_full_global(self, latest_dir: str):
        if self.global_costmap_msg is None:
            return
        if self.global_full_hz <= 0.0:
            return

        now = time.time()
        period = 1.0 / self.global_full_hz
        if now - self.last_full_global_save_wall < period * 0.8:
            return

        self.write_full_global_outputs(latest_dir, self.global_costmap_msg)
        self.last_full_global_save_wall = now

    def on_timer(self):
        if self.map_msg is None:
            if self.verbose_skips:
                self.get_logger().warn("skip: waiting for /map")
            return
        if self.local_costmap_msg is None:
            if self.verbose_skips:
                self.get_logger().warn("skip: waiting for /local_costmap/costmap")
            return
        if self.global_costmap_msg is None:
            if self.verbose_skips:
                self.get_logger().warn("skip: waiting for /global_costmap/costmap")
            return

        pose = self.robot_pose_in_render()
        if pose is None:
            if self.verbose_skips:
                self.get_logger().warn("skip: TF render_frame -> robot_frame unavailable")
            return

        robot_x, robot_y, robot_yaw = pose
        half = self.crop_m / 2.0
        min_x = robot_x - half
        max_x = robot_x + half
        min_y = robot_y - half
        max_y = robot_y + half

        map_frame = self.map_msg.header.frame_id or "map"
        local_frame = self.local_costmap_msg.header.frame_id or "odom"
        global_frame = self.global_costmap_msg.header.frame_id or "map"

        tf_map = self.tf_render_to(map_frame)
        tf_local = self.tf_render_to(local_frame)
        tf_global = self.tf_render_to(global_frame)

        if map_frame != self.render_frame and tf_map is None:
            if self.verbose_skips:
                self.get_logger().warn(f"skip: TF {self.render_frame}->{map_frame} unavailable")
            return
        if local_frame != self.render_frame and tf_local is None:
            if self.verbose_skips:
                self.get_logger().warn(f"skip: TF {self.render_frame}->{local_frame} unavailable")
            return
        if global_frame != self.render_frame and tf_global is None:
            if self.verbose_skips:
                self.get_logger().warn(f"skip: TF {self.render_frame}->{global_frame} unavailable")
            return

        static_u8 = render_crop_vectorized(self.map_msg, tf_map, min_x, max_x, min_y, max_y, self.out_px, mode="static")
        local_u8 = render_crop_vectorized(self.local_costmap_msg, tf_local, min_x, max_x, min_y, max_y, self.out_px, mode="costmap")
        glob_u8 = render_crop_vectorized(self.global_costmap_msg, tf_global, min_x, max_x, min_y, max_y, self.out_px, mode="costmap")

        static_img = Image.fromarray(static_u8, mode="L").convert("RGB")
        local_img = Image.fromarray(local_u8, mode="L").convert("RGB")
        global_img = Image.fromarray(glob_u8, mode="L").convert("RGB")

        ts = time.strftime("%Y%m%d_%H%M%S")
        meters_per_px = self.crop_m / self.out_px

        meta = {
            "tick": self.tick,
            "timestamp": ts,
            "render_frame": self.render_frame,
            "robot_frame": self.robot_frame,
            "robot_pose_render": {"x": robot_x, "y": robot_y, "yaw_rad": robot_yaw},
            "crop": {
                "size_m": self.crop_m,
                "out_px": self.out_px,
                "meters_per_px": meters_per_px,
                "bounds_render": {"min_x": min_x, "max_x": max_x, "min_y": min_y, "max_y": max_y},
            },
            "topics": {
                "map": self.map_topic,
                "local_costmap": self.local_costmap_topic,
                "global_costmap": self.global_costmap_topic,
            },
            "grid_frames": {
                "map_frame": map_frame,
                "local_costmap_frame": local_frame,
                "global_costmap_frame": global_frame,
            },
            "pixel_to_map_equations": {
                "x_map": "min_x + (u + 0.5) * meters_per_px",
                "y_map": "max_y - (v + 0.5) * meters_per_px",
                "note": "Selection coords are on local_costmap_crop.png (u right, v down).",
            },
        }

        latest_dir = os.path.join(self.out_dir, "latest")
        self.write_outputs(latest_dir, static_img, local_img, global_img, meta)
        self.maybe_write_full_global(latest_dir)

        if self.save_history:
            hist_dir = os.path.join(self.out_dir, f"tick_{ts}_{self.tick:05d}")
            self.write_outputs(hist_dir, static_img, local_img, global_img, meta)
            self.write_full_global_outputs(hist_dir, self.global_costmap_msg)

        now_wall = time.time()
        dt = None if self.last_save_wall is None else (now_wall - self.last_save_wall)
        self.last_save_wall = now_wall

        if dt is None:
            self.get_logger().info(f"Saved latest tick {self.tick} -> {latest_dir}")
        else:
            self.get_logger().info(f"Saved latest tick {self.tick} -> {latest_dir} (Δt={dt:.2f}s)")
        self.tick += 1


def main():
    ap = argparse.ArgumentParser()
    default_out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "notes",
        "current",
        "exports",
    )
    ap.add_argument("--out-dir", default=default_out_dir)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--global-full-hz", type=float, default=1.0)
    ap.add_argument("--crop-m", type=float, default=10.0)
    ap.add_argument("--out-px", type=int, default=200)
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--local-costmap-topic", default="/local_costmap/costmap")
    ap.add_argument("--global-costmap-topic", default="/global_costmap/costmap")
    ap.add_argument("--render-frame", default="map")
    ap.add_argument("--robot-frame", default="base_link")
    ap.add_argument("--use-sim-time", action="store_true")
    ap.add_argument("--save-history", action="store_true")
    ap.add_argument("--verbose-skips", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = CropExporter(
        map_topic=args.map_topic,
        local_costmap_topic=args.local_costmap_topic,
        global_costmap_topic=args.global_costmap_topic,
        render_frame=args.render_frame,
        robot_frame=args.robot_frame,
        out_dir=args.out_dir,
        crop_m=args.crop_m,
        out_px=args.out_px,
        hz=args.hz,
        global_full_hz=args.global_full_hz,
        use_sim_time=bool(args.use_sim_time),
        save_history=bool(args.save_history),
        verbose_skips=bool(args.verbose_skips),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
