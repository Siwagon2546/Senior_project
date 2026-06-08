#!/usr/bin/env python3
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


Waypoint = Tuple[float, float, float]


class SequentialNavigator(Node):
    def __init__(self) -> None:
        super().__init__('sequential_nav_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pause_sec', 0.0)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('waypoints', [])

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.pause_sec = float(self.get_parameter('pause_sec').value)
        self.stop_on_failure = bool(self.get_parameter('stop_on_failure').value)

        raw_waypoints = list(self.get_parameter('waypoints').value)
        self.waypoints: List[Waypoint] = self.parse_waypoints(raw_waypoints)

        if not self.waypoints:
            self.get_logger().error('No valid waypoints provided.')
            raise RuntimeError('No valid waypoints provided.')

        self.current_index = 0
        self.pause_timer = None

        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        self.client.wait_for_server()
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoint(s).')

        self.send_next_goal()

    def parse_waypoints(self, raw_waypoints: List[str]) -> List[Waypoint]:
        parsed: List[Waypoint] = []

        for i, item in enumerate(raw_waypoints):
            if not isinstance(item, str):
                raise ValueError(
                    f'Waypoint index {i} must be string like "x,y,yaw_deg", got: {type(item)}'
                )

            parts = [p.strip() for p in item.split(',')]
            if len(parts) != 3:
                raise ValueError(
                    f'Waypoint index {i} invalid format: "{item}". '
                    f'Expected "x,y,yaw_deg"'
                )

            x = float(parts[0])
            y = float(parts[1])
            yaw_deg = float(parts[2])
            parsed.append((x, y, yaw_deg))

        return parsed

    def yaw_deg_to_quaternion(self, yaw_deg: float):
        yaw_rad = math.radians(yaw_deg)
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        return qx, qy, qz, qw

    def build_goal(self, waypoint: Waypoint) -> NavigateToPose.Goal:
        x, y, yaw_deg = waypoint

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_deg_to_quaternion(yaw_deg)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal.pose = pose
        return goal

    def send_next_goal(self) -> None:
        if self.current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed.')
            rclpy.shutdown()
            return

        x, y, yaw_deg = self.waypoints[self.current_index]
        self.get_logger().info(
            f'Sending waypoint {self.current_index + 1}/{len(self.waypoints)}: '
            f'x={x:.3f}, y={y:.3f}, yaw_deg={yaw_deg:.1f}'
        )

        goal_msg = self.build_goal(self.waypoints[self.current_index])
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('Goal handle is None')
            rclpy.shutdown()
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'Waypoint {self.current_index + 1} rejected')
            if self.stop_on_failure:
                rclpy.shutdown()
            else:
                self.current_index += 1
                self.send_next_goal()
            return

        self.get_logger().info(f'Waypoint {self.current_index + 1} accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future) -> None:
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self.current_index + 1} succeeded')
            self.current_index += 1

            if self.pause_sec > 0.0:
                self.get_logger().info(f'Pause {self.pause_sec:.1f} sec before next waypoint')
                self.pause_timer = self.create_timer(self.pause_sec, self.resume_after_pause)
            else:
                self.send_next_goal()
        else:
            self.get_logger().error(
                f'Waypoint {self.current_index + 1} failed with status={status}'
            )
            if self.stop_on_failure:
                rclpy.shutdown()
            else:
                self.current_index += 1
                self.send_next_goal()

    def resume_after_pause(self) -> None:
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.pause_timer = None
        self.send_next_goal()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SequentialNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()