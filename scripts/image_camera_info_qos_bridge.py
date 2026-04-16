#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class ImageCameraInfoQosBridge(Node):
    def __init__(self):
        super().__init__('image_camera_info_qos_bridge')

        in_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_pub = self.create_publisher(
            Image, '/apriltag_bridge/image', out_qos
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/apriltag_bridge/camera_info', out_qos
        )

        self.image_sub = self.create_subscription(
            Image, '/camera0/color/image_raw', self.image_cb, in_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera0/color/camera_info', self.camera_info_cb, in_qos
        )

    def image_cb(self, msg):
        self.image_pub.publish(msg)

    def camera_info_cb(self, msg):
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCameraInfoQosBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()