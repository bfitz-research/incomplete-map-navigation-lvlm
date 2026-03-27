#!/usr/bin/env python3
import argparse
import csv
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class ExecutedPathPublisher(Node):
    def __init__(self, csv_path: str, topic: str):
        super().__init__("executed_path_publisher")
        self.pub = self.create_publisher(Path, topic, 1)

        csv_path = os.path.expanduser(csv_path)
        if not os.path.exists(csv_path):
            raise FileNotFoundError(csv_path)

        poses = []
        with open(csv_path, "r") as f:
            r = csv.DictReader(f)
            for row in r:
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = float(row["x"])
                ps.pose.position.y = float(row["y"])
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                poses.append(ps)

        msg = Path()
        msg.header.frame_id = "map"
        msg.poses = poses
        self.msg = msg

        self.get_logger().info(f"Loaded {len(poses)} points from {csv_path}")
        self.get_logger().info(f"Publishing on {topic}")

        # Publish repeatedly so RViz always sees it
        self.create_timer(0.5, self.timer_cb)

    def timer_cb(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="Trajectory CSV from log_executed_traj_tf.py")
    ap.add_argument("--topic", default="/executed_path")
    args = ap.parse_args()

    rclpy.init()
    node = ExecutedPathPublisher(args.inp, args.topic)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
