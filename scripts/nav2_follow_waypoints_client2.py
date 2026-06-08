#!/usr/bin/env python3
import json
import math
from pathlib import Path
from typing import Any, Dict, List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_to_quaternion(yaw_deg: float) -> Dict[str, float]:
    """Convert planar yaw in degrees to quaternion dict."""
    yaw_rad = math.radians(yaw_deg)
    half = yaw_rad * 0.5
    return {"z": math.sin(half), "w": math.cos(half)}

def normalize_waypoint(item: Dict[str, Any]) -> Dict[str, float]:
    x = float(item["x"])
    y = float(item["y"])
    z = float(item.get("z", 0.0))

    if "yaw" in item:
        q = yaw_to_quaternion(float(item["yaw"]))
        qz = q["z"]
        qw = q["w"]
    elif "orientation" in item:
        ori = item["orientation"]
        qz = float(ori.get("z", 0.0))
        qw = float(ori.get("w", 1.0))
    else:
        qz = float(item.get("qz", 0.0))
        qw = float(item.get("qw", 1.0))

    return {"x": x, "y": y, "z": z, "qz": qz, "qw": qw}


def load_waypoints_from_json(path: str) -> List[Dict[str, float]]:
    content = json.loads(Path(path).read_text(encoding="utf-8"))

    if isinstance(content, dict):
        if "poses" in content:
            raw = content["poses"]
        else:
            raise ValueError("JSON object must contain a 'poses' field.")
    elif isinstance(content, list):
        raw = content
    else:
        raise ValueError("JSON root must be an object or list.")

    if not raw:
        raise ValueError("No waypoints found in JSON file.")

    return [normalize_waypoint(item) for item in raw]


class FollowWaypointsClient(Node):
    def __init__(self) -> None:
        super().__init__("follow_waypoints_client")

        self.declare_parameter("route_file", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("number_of_loops", 0)
        self.declare_parameter("goal_index", 0)
        self.declare_parameter("wait_for_server_timeout_sec", 15.0)

        # โฟลเดอร์เดียวกับไฟล์ .py นี้
        script_dir = Path(__file__).resolve().parent
        default_route_file = script_dir / "waypoints_Slope_to_rtt2.json"

        route_file = self.get_parameter("route_file").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.number_of_loops = int(
            self.get_parameter("number_of_loops").get_parameter_value().integer_value
        )
        self.goal_index = int(
            self.get_parameter("goal_index").get_parameter_value().integer_value
        )
        self.wait_for_server_timeout_sec = float(
            self.get_parameter("wait_for_server_timeout_sec").get_parameter_value().double_value
        )

        # ถ้าไม่ได้ส่ง route_file มา ให้ใช้ waypoints.json ที่อยู่ข้างไฟล์นี้
        if not route_file:
            route_file = str(default_route_file)

        if Path(route_file).exists():
            self.get_logger().info(f"Loading waypoints from: {route_file}")
            self.waypoints = load_waypoints_from_json(route_file)
        else:
            self.get_logger().warn(
                f"Route file not found: {route_file}, using built-in sample waypoints."
            )
            self.waypoints = [
                {"x": 2.0, "y": 0.46, "z": 0.0, "qz": 0.0, "qw": 1.0},
                {"x": 4.0, "y": 0.46, "z": 0.0, "qz": 0.0, "qw": 1.0},
                {"x": 5.0, "y": 0.58, "z": 0.0, "qz": 0.0, "qw": 1.0},
            ]

        self.action_client = ActionClient(self, FollowWaypoints, "/follow_waypoints")
        self._goal_done = False
        self._exit_code = 0

    def build_pose(self, wp: Dict[str, float]) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = wp["x"]
        pose.pose.position.y = wp["y"]
        pose.pose.position.z = wp["z"]
        pose.pose.orientation.z = wp["qz"]
        pose.pose.orientation.w = wp["qw"]
        return pose

    def send_goal(self) -> None:
        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec):
            self.get_logger().error(
                f"/follow_waypoints action server not available after "
                f"{self.wait_for_server_timeout_sec:.1f}s"
            )
            self._goal_done = True
            self._exit_code = 1
            return

        poses = [self.build_pose(wp) for wp in self.waypoints]

        goal_msg = FollowWaypoints.Goal()
        if hasattr(goal_msg, "number_of_loops"):
            goal_msg.number_of_loops = self.number_of_loops
        if hasattr(goal_msg, "goal_index"):
            goal_msg.goal_index = self.goal_index
        goal_msg.poses = poses

        self.get_logger().info(f"Sending {len(poses)} waypoints to /follow_waypoints")
        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg: Any) -> None:
        feedback = feedback_msg.feedback
        current = getattr(feedback, "current_waypoint", None)
        if current is not None:
            self.get_logger().info(f"Current waypoint index: {current}")

    def goal_response_cb(self, future: Any) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("FollowWaypoints goal rejected.")
            self._goal_done = True
            self._exit_code = 1
            return

        self.get_logger().info("FollowWaypoints goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future: Any) -> None:
        wrapped = future.result()
        status = wrapped.status
        result = wrapped.result

        missed = getattr(result, "missed_waypoints", [])
        if status == 4:
            self.get_logger().error("FollowWaypoints aborted.")
            self._exit_code = 1
        elif status == 5:
            self.get_logger().warn("FollowWaypoints canceled.")
            self._exit_code = 1
        else:
            self._exit_code = 0

        if missed:
            rendered = []
            for item in missed:
                if isinstance(item, int):
                    rendered.append(str(item))
                else:
                    idx = getattr(item, "index", None)
                    code = getattr(item, "error_code", None)
                    msg = getattr(item, "error_msg", "")
                    rendered.append(f"index={idx}, code={code}, msg={msg}")
            self.get_logger().warn("Missed waypoints: " + "; ".join(rendered))

        error_code = getattr(result, "error_code", None)
        error_msg = getattr(result, "error_msg", None)
        if error_code not in (None, 0):
            self.get_logger().error(f"Result error_code={error_code}, error_msg={error_msg}")
            self._exit_code = 1

        if self._exit_code == 0 and not missed:
            self.get_logger().info("All waypoints completed successfully.")
        self._goal_done = True


def main() -> int:
    rclpy.init()
    node = FollowWaypointsClient()
    try:
        node.send_goal()
        while rclpy.ok() and not node._goal_done:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node._exit_code
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())