#!/usr/bin/env python3
import argparse
import json
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class OraclePathPublisher(Node):
    def __init__(self, path_json: str, topic: str):
        super().__init__("oracle_path_publisher")
        self.pub = self.create_publisher(Path, topic, 1)

        with open(path_json, "r") as f:
            data = json.load(f)

        msg = Path()
        msg.header.frame_id = "map"

        poses = []
        for p in data["path"]["poses"]:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(p["x"])
            ps.pose.position.y = float(p["y"])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            poses.append(ps)

        msg.poses = poses
        self.path_msg = msg

        # Publish repeatedly so RViz always catches it (like a latched topic)
        self.create_timer(0.5, self.timer_cb)
        self.get_logger().info(f"Loaded {len(poses)} poses from {path_json}")
        self.get_logger().info(f"Publishing on topic: {topic}")

    def timer_cb(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="oracle_path.json")
    ap.add_argument("--topic", default="/oracle_path")
    args = ap.parse_args()

    rclpy.init()
    node = OraclePathPublisher(os.path.expanduser(args.inp), args.topic)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
