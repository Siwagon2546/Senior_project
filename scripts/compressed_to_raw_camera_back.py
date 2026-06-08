#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class CompressedToRawCameraBack(Node):
    def __init__(self):
        super().__init__('compressed_to_raw_camera_back')

        self.declare_parameter('compressed_topic', '/camera_back/image_raw/compressed')
        self.declare_parameter('raw_topic', '/camera_back/image_raw')
        self.declare_parameter('encoding', 'rgb8')

        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.raw_topic = self.get_parameter('raw_topic').value
        self.encoding = self.get_parameter('encoding').value

        if self.encoding not in ['rgb8', 'bgr8']:
            raise RuntimeError('encoding must be rgb8 or bgr8')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.raw_pub = self.create_publisher(Image, self.raw_topic, qos)

        self.sub = self.create_subscription(
            CompressedImage,
            self.compressed_topic,
            self.callback,
            qos
        )

        self.get_logger().info(
            f'Decompress relay started: {self.compressed_topic} -> {self.raw_topic}, '
            f'encoding={self.encoding}'
        )

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame_bgr is None:
            self.get_logger().warn('Failed to decode compressed image')
            return

        if self.encoding == 'rgb8':
            frame = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        else:
            frame = frame_bgr

        frame = np.ascontiguousarray(frame)

        h, w = frame.shape[:2]

        out = Image()
        out.header = msg.header
        out.height = h
        out.width = w
        out.encoding = self.encoding
        out.is_bigendian = 0
        out.step = w * 3
        out.data = frame.tobytes()

        self.raw_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedToRawCameraBack()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()