#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo


class QosRelay(Node):
    def __init__(self):
        super().__init__('apriltag_qos_relay')

        image_in = self.declare_parameter(
            'image_in', '/camera0/color/image_raw').get_parameter_value().string_value
        camera_info_in = self.declare_parameter(
            'camera_info_in', '/camera0/color/camera_info').get_parameter_value().string_value
        image_out = self.declare_parameter(
            'image_out', '/apriltag_bridge/image').get_parameter_value().string_value
        camera_info_out = self.declare_parameter(
            'camera_info_out', '/apriltag_bridge/camera_info').get_parameter_value().string_value

        # subscribe เข้าจาก RealSense/Nvblox ฝั่ง sensor data
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # publish ออกให้ apriltag แบบ reliable
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.image_pub = self.create_publisher(Image, image_out, pub_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_out, pub_qos)

        self.image_sub = self.create_subscription(
            Image, image_in, self.image_cb, sub_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_in, self.camera_info_cb, sub_qos)

        self.get_logger().info(f'Image relay: {image_in} -> {image_out}')
        self.get_logger().info(f'CameraInfo relay: {camera_info_in} -> {camera_info_out}')

    def image_cb(self, msg: Image):
        self.image_pub.publish(msg)

    def camera_info_cb(self, msg: CameraInfo):
        self.camera_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = QosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()