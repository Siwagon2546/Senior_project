#!/usr/bin/env python3
import math
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(nav, x, y, yaw=0.0, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    nav = BasicNavigator()

    nav.waitUntilNav2Active()

    poses = [
        make_pose(nav, 2.0, 0.46, 0.0),
        make_pose(nav, 4.0, 0.46, 0.0),
        make_pose(nav, 5.0, 0.58, 0.0),
    ]

    nav.goThroughPoses(poses)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        if feedback:
            print(f"distance_remaining = {feedback.distance_remaining:.2f} m")

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("NavigateThroughPoses succeeded")
    elif result == TaskResult.CANCELED:
        print("NavigateThroughPoses canceled")
    else:
        print("NavigateThroughPoses failed")

    rclpy.shutdown()


if __name__ == '__main__':
    main()