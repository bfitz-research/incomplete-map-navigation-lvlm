#!/usr/bin/env python3
import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(x, y, z, w) -> float:
    # yaw (Z) from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajLogger(Node):
    def __init__(self, out_csv: str, hz: float, parent_frame: str, child_frame: str):
        super().__init__("executed_traj_logger")

        self.parent_frame = parent_frame
        self.child_frame = child_frame

        os.makedirs(os.path.dirname(out_csv), exist_ok=True)
        self.f = open(out_csv, "w", buffering=1)  # line-buffered
        self.f.write("t_sec,x,y,yaw_rad\n")

        self.get_logger().info(f"Logging TF {parent_frame} -> {child_frame} at {hz} Hz")
        self.get_logger().info(f"Writing to: {out_csv}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        period = 1.0 / hz
        self.timer = self.create_timer(period, self.tick)

        self.t0 = time.time()
        self.samples = 0
        self.misses = 0

    def tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time(),  # latest available
            )
        except TransformException:
            self.misses += 1
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        t_sec = time.time() - self.t0
        self.f.write(f"{t_sec:.6f},{tx:.6f},{ty:.6f},{yaw:.6f}\n")
        self.samples += 1

        # Occasionally print a heartbeat
        if self.samples % 200 == 0:
            self.get_logger().info(f"samples={self.samples} misses={self.misses}")

    def close(self):
        try:
            self.get_logger().info(f"Closing logger. samples={self.samples} misses={self.misses}")
            self.f.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="", help="Output CSV path. If empty, writes to ~/table_nav2/notes/runs/<timestamp>/traj.csv")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--parent", default="map")
    ap.add_argument("--child", default="base_link")
    args = ap.parse_args()

    if args.out:
        out_csv = os.path.expanduser(args.out)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_csv = os.path.expanduser(f"~/table_nav2/notes/runs/{ts}/traj.csv")

    rclpy.init()
    node = TrajLogger(out_csv, args.hz, args.parent, args.child)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
