#!/usr/bin/env python3
import json
import math
import os
import time
from functools import partial
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient
from rclpy.exceptions import ParameterAlreadyDeclaredException, ParameterNotDeclaredException

from std_msgs.msg import Int32MultiArray, String
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException

from nav2_msgs.action import NavigateToPose


def yaw_from_quat(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def occgrid_value_at(grid: OccupancyGrid, x_f: float, y_f: float) -> Optional[int]:
    info = grid.info
    res = float(info.resolution)
    ox = float(info.origin.position.x)
    oy = float(info.origin.position.y)

    ix = int(math.floor((x_f - ox) / res))
    iy = int(math.floor((y_f - oy) / res))

    if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
        return None

    idx = ix + iy * info.width
    return int(grid.data[idx])


class RegionGoalPickerNode(Node):
    def __init__(self):
        super().__init__("region_goal_picker_node")

        use_sim_time_default = False
        try:
            self.declare_parameter("use_sim_time", use_sim_time_default)
        except ParameterAlreadyDeclaredException:
            pass

        use_sim_time = (
            bool(self.get_parameter("use_sim_time").value)
            if self.has_parameter("use_sim_time")
            else use_sim_time_default
        )
        try:
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])
        except ParameterNotDeclaredException:
            self.declare_parameter("use_sim_time", use_sim_time)
            self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)])

        def safe_declare(name: str, default):
            try:
                self.declare_parameter(name, default)
            except ParameterAlreadyDeclaredException:
                pass

        safe_declare("meta_path", "~/table_nav2/notes/current/exports/latest/meta.json")
        safe_declare("markers_path", "~/table_nav2/notes/current/state/markers.json")
        safe_declare("write_active_goal", True)

        safe_declare("local_costmap_topic", "/local_costmap/costmap")
        safe_declare("region_request_topic", "/region_request")
        safe_declare("region_result_topic", "/region_result")
        safe_declare("action_name", "/navigate_to_pose")

        safe_declare("max_drift_m", 0.5)
        safe_declare("max_drift_deg", 20.0)

        safe_declare("tf_wait_s", 10.0)
        safe_declare("feedback", True)

        self.meta_path = os.path.expanduser(str(self.get_parameter("meta_path").value))
        self.markers_path = os.path.expanduser(str(self.get_parameter("markers_path").value))
        self.write_active_goal = bool(self.get_parameter("write_active_goal").value)

        self.local_costmap_topic = str(self.get_parameter("local_costmap_topic").value)
        self.req_topic = str(self.get_parameter("region_request_topic").value)
        self.res_topic = str(self.get_parameter("region_result_topic").value)
        self.action_name = str(self.get_parameter("action_name").value)

        self.max_drift_m = float(self.get_parameter("max_drift_m").value)
        self.max_drift_rad = math.radians(float(self.get_parameter("max_drift_deg").value))
        self.tf_wait_s = float(self.get_parameter("tf_wait_s").value)
        self.print_feedback = bool(self.get_parameter("feedback").value)

        os.makedirs(os.path.dirname(self.markers_path), exist_ok=True)

        qos_costmap = QoSProfile(depth=1)
        qos_costmap.reliability = ReliabilityPolicy.RELIABLE
        qos_costmap.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.local_costmap: Optional[OccupancyGrid] = None
        self.create_subscription(OccupancyGrid, self.local_costmap_topic, self._on_costmap, qos_costmap)

        self.result_pub = self.create_publisher(String, self.res_topic, 10)
        self.create_subscription(Int32MultiArray, self.req_topic, self._on_request, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self._goal_seq = 0
        self._latest_goal_seq = -1
        self._active_goal_handle = None

        self.get_logger().info(f"use_sim_time={use_sim_time}")
        self.get_logger().info(f"meta_path={self.meta_path}")
        self.get_logger().info(f"markers_path={self.markers_path}")
        self.get_logger().info(f"write_active_goal={self.write_active_goal}")
        self.get_logger().info(f"local_costmap_topic={self.local_costmap_topic}")
        self.get_logger().info(f"region_request_topic={self.req_topic}")
        self.get_logger().info(f"region_result_topic={self.res_topic}")
        self.get_logger().info(f"action_name={self.action_name}")
        self.get_logger().info(
            f"staleness_guard: max_drift_m={self.max_drift_m} "
            f"max_drift_deg={math.degrees(self.max_drift_rad):.1f}"
        )
        self.get_logger().info(
            f"tf_wait_s={self.tf_wait_s} feedback={self.print_feedback}"
        )

    def _default_markers(self) -> dict:
        return {
            "ultimate_goal": None,
            "active_goal": None,
            "pois": [],
        }

    def _load_markers(self) -> dict:
        try:
            with open(self.markers_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return self._default_markers()
        except Exception:
            return self._default_markers()

        data.setdefault("ultimate_goal", None)
        data.setdefault("active_goal", None)
        data.setdefault("pois", [])
        if not isinstance(data.get("pois"), list):
            data["pois"] = []
        return data

    def _write_json_atomic(self, path: str, obj: dict):
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=2)
            f.write("\n")
        os.replace(tmp, path)

    def _update_active_goal_marker(self, x_map: float, y_map: float):
        markers = self._load_markers()
        self.get_logger().info(f"[markers] before write: {markers}")

        markers["active_goal"] = {"x": float(x_map), "y": float(y_map)}
        self.get_logger().info(
            f"[markers] writing active_goal to {self.markers_path}: "
            f"{markers['active_goal']}"
        )

        self._write_json_atomic(self.markers_path, markers)

        with open(self.markers_path, "r", encoding="utf-8") as f:
            verify = json.load(f)

        self.get_logger().info(f"[markers] after write: {verify}")

    def _clear_active_goal_marker(self):
        markers = self._load_markers()
        self.get_logger().info(f"[markers] before clear: {markers}")

        markers["active_goal"] = None
        self._write_json_atomic(self.markers_path, markers)

        with open(self.markers_path, "r", encoding="utf-8") as f:
            verify = json.load(f)

        self.get_logger().info(f"[markers] after clear: {verify}")

    def _on_costmap(self, msg: OccupancyGrid):
        self.local_costmap = msg

    def _publish_result(self, obj: dict):
        msg = String()
        msg.data = json.dumps(obj, indent=2)
        self.result_pub.publish(msg)

    def _load_meta(self) -> Optional[dict]:
        try:
            with open(self.meta_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            self._publish_result(
                {
                    "ok": False,
                    "error": f"Failed to read meta.json: {e}",
                    "meta_path": self.meta_path,
                }
            )
            return None

    def _wait_for_tf(self, target: str, source: str, timeout_s: float) -> bool:
        deadline = time.time() + timeout_s
        while rclpy.ok() and time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                return True
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _robot_pose_map(self) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        return x, y, yaw

    def _meta_fresh(self, meta: dict) -> bool:
        mp = meta.get("robot_pose_render", None)
        if not mp:
            return True

        if not self._wait_for_tf("map", "base_link", self.tf_wait_s):
            self._publish_result(
                {
                    "ok": False,
                    "error": "TF map<-base_link not ready (timeout)",
                    "tf_wait_s": self.tf_wait_s,
                }
            )
            return False

        curr = self._robot_pose_map()
        if curr is None:
            self._publish_result({"ok": False, "error": "TF map<-base_link not available after wait"})
            return False

        cx, cy, cyaw = curr
        mx = float(mp["x"])
        my = float(mp["y"])
        myaw = float(mp["yaw_rad"])

        dxy = math.hypot(cx - mx, cy - my)
        dyaw = abs(wrap_pi(cyaw - myaw))

        if dxy > self.max_drift_m or dyaw > self.max_drift_rad:
            self._publish_result(
                {
                    "ok": False,
                    "error": "STALE_META",
                    "dxy_m": dxy,
                    "dyaw_deg": math.degrees(dyaw),
                    "meta_pose": {"x": mx, "y": my, "yaw_deg": math.degrees(myaw)},
                    "curr_pose": {"x": cx, "y": cy, "yaw_deg": math.degrees(cyaw)},
                }
            )
            return False

        return True

    def _pixel_to_map(self, meta: dict, u: int, v: int) -> Tuple[float, float]:
        b = meta["crop"]["bounds_render"]
        min_x = float(b["min_x"])
        max_y = float(b["max_y"])
        mpp = float(meta["crop"]["meters_per_px"])
        x = min_x + (u + 0.5) * mpp
        y = max_y - (v + 0.5) * mpp
        return x, y

    def _map_to_costmap_frame(
        self, x_map: float, y_map: float, costmap_frame: str
    ) -> Optional[Tuple[float, float]]:
        if costmap_frame == "map":
            return (x_map, y_map)
        if not self._wait_for_tf(costmap_frame, "map", self.tf_wait_s):
            return None
        try:
            tf = self.tf_buffer.lookup_transform(costmap_frame, "map", rclpy.time.Time())
        except TransformException:
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        c = math.cos(yaw)
        s = math.sin(yaw)
        x_c = tx + c * x_map - s * y_map
        y_c = ty + s * x_map + c * y_map
        return (x_c, y_c)

    def _pick_best(self, meta: dict, x1: int, y1: int, x2: int, y2: int, free_thresh: int, stride: int):
        cm = self.local_costmap
        if cm is None:
            return None, {"ok": False, "error": "No local_costmap received yet"}

        cm_frame = cm.header.frame_id or "odom"
        out_px = int(meta["crop"]["out_px"])

        u1, u2 = sorted((x1, x2))
        v1, v2 = sorted((y1, y2))
        u1 = max(0, min(out_px - 1, u1))
        u2 = max(0, min(out_px - 1, u2))
        v1 = max(0, min(out_px - 1, v1))
        v2 = max(0, min(out_px - 1, v2))

        # Hard-cap the allowed cost so gray / inflated pixels cannot be selected
        # just because the LVLM asked for a loose threshold.
        HARD_FREE_THRESH = 15
        effective_thresh = min(int(free_thresh), HARD_FREE_THRESH)

        best = None  # (cost, u, v, x_map, y_map)

        for v in range(v1, v2 + 1, stride):
            for u in range(u1, u2 + 1, stride):
                x_map, y_map = self._pixel_to_map(meta, u, v)
                xy_c = self._map_to_costmap_frame(x_map, y_map, cm_frame)
                if xy_c is None:
                    continue

                x_c, y_c = xy_c
                val = occgrid_value_at(cm, x_c, y_c)
                if val is None or val < 0:
                    continue
                if val > 253:
                    continue
                if val > effective_thresh:
                    continue

                if (best is None) or (val < best[0]):
                    best = (val, u, v, x_map, y_map)

        if best is None:
            return None, {
                "ok": False,
                "error": "NO_FEASIBLE_PIXEL",
                "region": [u1, v1, u2, v2],
                "requested_free_thresh": free_thresh,
                "effective_free_thresh": effective_thresh,
                "stride": stride,
            }

        cost, u, v, x_map, y_map = best
        return best, {
            "ok": True,
            "picked": {
                "u": u,
                "v": v,
                "cost": cost,
                "x_map": x_map,
                "y_map": y_map,
            },
            "region": [u1, v1, u2, v2],
            "requested_free_thresh": free_thresh,
            "effective_free_thresh": effective_thresh,
            "stride": stride,
        }

    def _feedback_cb(self, fb_msg):
        if not self.print_feedback:
            return
        fb = fb_msg.feedback
        if hasattr(fb, "distance_remaining"):
            self.get_logger().info(f"[feedback] distance_remaining={fb.distance_remaining:.3f}")

    def _cancel_previous_goal(self):
        if self._active_goal_handle is None:
            return
        try:
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._on_cancel_done)
            self.get_logger().info("[nav2] cancel requested for previous goal")
        except Exception as e:
            self.get_logger().warn(f"[nav2] failed to request cancel on previous goal: {e}")

    def _on_cancel_done(self, future):
        try:
            _ = future.result()
            self.get_logger().info("[nav2] previous goal cancel response received")
        except Exception as e:
            self.get_logger().warn(f"[nav2] cancel callback exception: {e}")

    def _send_nav_goal_async(self, x_map: float, y_map: float, goal_seq: int):
        if not self.action_client.server_is_ready():
            self._publish_result(
                {
                    "ok": False,
                    "error": "Action server not available",
                    "action_name": self.action_name,
                }
            )
            return False

        if self._active_goal_handle is not None:
            self._cancel_previous_goal()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(x_map)
        goal_msg.pose.pose.position.y = float(y_map)
        goal_msg.pose.pose.orientation.w = 1.0

        send_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb if self.print_feedback else None,
        )
        send_future.add_done_callback(partial(self._on_goal_response, goal_seq, x_map, y_map))
        return True

    def _on_goal_response(self, goal_seq: int, x_map: float, y_map: float, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self._publish_result(
                {
                    "ok": False,
                    "phase": "goal_response",
                    "error": f"Exception waiting for goal response: {e}",
                    "goal_seq": goal_seq,
                    "goal": {"x": x_map, "y": y_map},
                }
            )
            return

        if not goal_handle.accepted:
            self._publish_result(
                {
                    "ok": False,
                    "phase": "goal_response",
                    "error": "Goal rejected by Nav2",
                    "goal_seq": goal_seq,
                    "goal": {"x": x_map, "y": y_map},
                }
            )
            return

        self._active_goal_handle = goal_handle
        self._publish_result(
            {
                "ok": True,
                "phase": "goal_accepted",
                "goal_seq": goal_seq,
                "goal": {"x": x_map, "y": y_map},
            }
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._on_nav_result, goal_seq, x_map, y_map, goal_handle))

    def _on_nav_result(self, goal_seq: int, x_map: float, y_map: float, goal_handle, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as e:
            self._publish_result(
                {
                    "ok": False,
                    "phase": "nav2_result",
                    "error": f"Exception waiting for Nav2 result: {e}",
                    "goal_seq": goal_seq,
                    "goal": {"x": x_map, "y": y_map},
                }
            )
            return

        stale = goal_seq != self._latest_goal_seq

        if stale:
            self.get_logger().info(
                f"[nav2] stale result for goal_seq={goal_seq}, "
                f"latest_goal_seq={self._latest_goal_seq}; leaving active_goal unchanged"
            )
        else:
            if self._active_goal_handle == goal_handle:
                self._active_goal_handle = None

            try:
                self._clear_active_goal_marker()
                self.get_logger().info("[markers] cleared active_goal after Nav2 result")
            except Exception as e:
                self.get_logger().warn(f"[markers] failed to clear active_goal after Nav2 result: {e}")

        self._publish_result(
            {
                "ok": True,
                "phase": "nav2_result",
                "goal_seq": goal_seq,
                "stale_result": stale,
                "nav2_status": status,
                "goal": {"x": x_map, "y": y_map},
                "markers_path": self.markers_path,
                "active_goal_written": bool(self.write_active_goal),
            }
        )

    def _on_request(self, msg: Int32MultiArray):
        arr = list(msg.data)
        if len(arr) < 6:
            self._publish_result(
                {
                    "ok": False,
                    "error": "Bad request. Expected [x1,y1,x2,y2,free_thresh,stride].",
                    "got": arr,
                }
            )
            return

        x1, y1, x2, y2, free_thresh, stride = arr[:6]
        free_thresh = int(free_thresh)
        stride = max(1, int(stride))

        meta = self._load_meta()
        if meta is None:
            return

        if not self._meta_fresh(meta):
            return

        _, info = self._pick_best(meta, x1, y1, x2, y2, free_thresh, stride)
        if not info.get("ok", False):
            self._publish_result(info)
            return

        picked = info["picked"]
        x_map = float(picked["x_map"])
        y_map = float(picked["y_map"])

        self.get_logger().info(
            f"Picked u={picked['u']} v={picked['v']} cost={picked['cost']} "
            f"-> goal (x={x_map:.3f}, y={y_map:.3f})"
        )

        if self.write_active_goal:
            try:
                self._update_active_goal_marker(x_map, y_map)
                self.get_logger().info("[markers] active_goal written before Nav2 send")
            except Exception as e:
                self.get_logger().error(f"Failed to update markers.json before send_goal: {e}")
                self._publish_result(
                    {
                        "ok": False,
                        "error": f"ACTIVE_GOAL_WRITE_FAILED: {e}",
                        "markers_path": self.markers_path,
                        "goal": {"x": x_map, "y": y_map},
                    }
                )
                return

        self._goal_seq += 1
        goal_seq = self._goal_seq
        self._latest_goal_seq = goal_seq

        self._publish_result(
            {
                "ok": True,
                "phase": "picked_and_sent",
                "goal_seq": goal_seq,
                "goal": {
                    "x": x_map,
                    "y": y_map,
                    "u": int(picked["u"]),
                    "v": int(picked["v"]),
                    "cost": int(picked["cost"]),
                },
                "markers_path": self.markers_path,
                "active_goal_written": bool(self.write_active_goal),
            }
        )

        self._send_nav_goal_async(x_map, y_map, goal_seq)


def main():
    rclpy.init()
    node = RegionGoalPickerNode()
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
