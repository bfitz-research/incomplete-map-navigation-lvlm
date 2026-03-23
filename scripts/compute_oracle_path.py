#!/usr/bin/env python3
import argparse
import json
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

def make_pose_stamped(frame_id: str, x: float, y: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    ps.pose.orientation.x = 0.0
    ps.pose.orientation.y = 0.0
    ps.pose.orientation.z = 0.0
    ps.pose.orientation.w = 1.0
    return ps

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal-x", type=float, required=True)
    ap.add_argument("--goal-y", type=float, required=True)
    ap.add_argument("--start-x", type=float, default=0.0)
    ap.add_argument("--start-y", type=float, default=0.0)
    ap.add_argument("--planner-id", type=str, default="")
    ap.add_argument("--out", type=str, required=True)
    ap.add_argument("--use-start", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("compute_oracle_path")

    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server /compute_path_to_pose not available.")
        rclpy.shutdown()
        raise SystemExit(1)

    goal = ComputePathToPose.Goal()
    goal.goal = make_pose_stamped("map", args.goal_x, args.goal_y)
    goal.start = make_pose_stamped("map", args.start_x, args.start_y)
    goal.planner_id = args.planner_id
    goal.use_start = bool(args.use_start)

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        node.get_logger().error("ComputePathToPose goal rejected.")
        rclpy.shutdown()
        raise SystemExit(2)

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    out = {
        "start": {"x": args.start_x, "y": args.start_y},
        "goal": {"x": args.goal_x, "y": args.goal_y},
        "planner_id": args.planner_id,
        "use_start": bool(args.use_start),
        "error_code": int(result.error_code),
        "error_msg": result.error_msg,
        "planning_time": {"sec": int(result.planning_time.sec), "nanosec": int(result.planning_time.nanosec)},
        "path": {
            "frame_id": result.path.header.frame_id,
            "poses": [{"x": float(p.pose.position.x), "y": float(p.pose.position.y)} for p in result.path.poses],
        },
    }

    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)

    node.get_logger().info(f"Saved: {args.out}")
    node.get_logger().info(f"error_code={out['error_code']} msg='{out['error_msg']}' poses={len(out['path']['poses'])}")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
