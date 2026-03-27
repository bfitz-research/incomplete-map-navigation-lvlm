#!/usr/bin/env python3
import argparse
import os
import time
from datetime import datetime
from typing import Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterNotDeclaredException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def atomic_write_bytes(path: str, data: bytes):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def atomic_write_text(path: str, text: str):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(text)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def encode_png_bytes(img_bgr) -> bytes:
    ok, buf = cv2.imencode(".png", img_bgr)
    if not ok:
        raise RuntimeError("cv2.imencode('.png', ...) failed")
    return buf.tobytes()


class RGBDualExporter(Node):
    def __init__(
        self,
        left_topic: str,
        right_topic: str,
        out_dir: str,
        hz: float,
        use_sim_time: bool,
        verbose_skips: bool,
    ):
        super().__init__("rgb_dual_exporter")

        try:
            self.declare_parameter("use_sim_time", use_sim_time)
        except ParameterAlreadyDeclaredException:
            pass
        try:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        except ParameterNotDeclaredException:
            self.declare_parameter("use_sim_time", use_sim_time)
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])

        self.left_topic = left_topic
        self.right_topic = right_topic
        self.out_dir = os.path.expanduser(out_dir)
        self.hz = float(hz)
        self.verbose_skips = bool(verbose_skips)

        os.makedirs(self.out_dir, exist_ok=True)
        os.makedirs(os.path.join(self.out_dir, "latest"), exist_ok=True)

        self.bridge = CvBridge()

        self.left_msg: Optional[Image] = None
        self.right_msg: Optional[Image] = None

        self.last_save_wall = None
        self.tick = 0

        self.create_subscription(Image, self.left_topic, self.on_left, qos_profile_sensor_data)
        self.create_subscription(Image, self.right_topic, self.on_right, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / self.hz, self.on_timer)

        self.get_logger().info(f"use_sim_time={use_sim_time}")
        self.get_logger().info(f"left_topic={self.left_topic}")
        self.get_logger().info(f"right_topic={self.right_topic}")
        self.get_logger().info(f"hz={self.hz}")
        self.get_logger().info(f"out_dir={self.out_dir} (writes exports/latest/)")

    def on_left(self, msg: Image):
        self.left_msg = msg

    def on_right(self, msg: Image):
        self.right_msg = msg

    def image_msg_to_bgr(self, msg: Image):
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def write_one(self, msg: Image, img_bgr, png_path: str, topic_name: str):
        png_bytes = encode_png_bytes(img_bgr)
        atomic_write_bytes(png_path, png_bytes)

        stamp = msg.header.stamp
        txt = (
            f"topic={topic_name}\n"
            f"frame_id={msg.header.frame_id}\n"
            f"stamp={stamp.sec}.{stamp.nanosec}\n"
            f"encoding={msg.encoding}\n"
            f"height={msg.height}\n"
            f"width={msg.width}\n"
            f"saved_wall={datetime.now().isoformat()}\n"
        )
        atomic_write_text(png_path + ".txt", txt)

    def on_timer(self):
        if self.left_msg is None:
            if self.verbose_skips:
                self.get_logger().warn(f"skip: waiting for {self.left_topic}")
            return
        if self.right_msg is None:
            if self.verbose_skips:
                self.get_logger().warn(f"skip: waiting for {self.right_topic}")
            return

        try:
            left_bgr = self.image_msg_to_bgr(self.left_msg)
            right_bgr = self.image_msg_to_bgr(self.right_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert RGB messages: {e}")
            return

        latest_dir = os.path.join(self.out_dir, "latest")
        left_out = os.path.join(latest_dir, "rgb_left.png")
        right_out = os.path.join(latest_dir, "rgb_right.png")

        try:
            self.write_one(self.left_msg, left_bgr, left_out, self.left_topic)
            self.write_one(self.right_msg, right_bgr, right_out, self.right_topic)
        except Exception as e:
            self.get_logger().error(f"Failed to write RGB outputs: {e}")
            return

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
    ap.add_argument("--left-topic", default="/zed_left/rgb")
    ap.add_argument("--right-topic", default="/zed_right/rgb")
    default_out_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "notes",
        "current",
        "exports",
    )
    ap.add_argument("--out-dir", default=default_out_dir)
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--use-sim-time", action="store_true")
    ap.add_argument("--verbose-skips", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = RGBDualExporter(
        left_topic=args.left_topic,
        right_topic=args.right_topic,
        out_dir=args.out_dir,
        hz=args.hz,
        use_sim_time=bool(args.use_sim_time),
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
