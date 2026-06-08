#!/usr/bin/env python3

import socket
import struct
import time
import argparse

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


class CameraBackTcpPublisher(Node):
    def __init__(self):
        super().__init__('camera_back_tcp_publisher')

        self.declare_parameter('host', '192.168.137.2')
        self.declare_parameter('port', 9101)
        self.declare_parameter('frame_id', 'camera_back_optical_frame')
        self.declare_parameter('fps_limit', 15.0)

        self.declare_parameter('image_topic', '/camera_back/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_back/camera_info')

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.fps_limit = float(self.get_parameter('fps_limit').value)

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.bridge = CvBridge()

        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)

        self.sock = None
        self.last_pub_time = 0.0

        self.get_logger().info(f'Camera BACK TCP Publisher started')
        self.get_logger().info(f'Connecting to {self.host}:{self.port}')
        self.get_logger().info(f'Publishing image: {self.image_topic}')
        self.get_logger().info(f'Publishing camera_info: {self.camera_info_topic}')

        self.timer = self.create_timer(0.001, self.loop)

    def connect_socket(self):
        try:
            if self.sock:
                self.sock.close()

            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.host, self.port))
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            self.get_logger().info(f'Connected to camera back stream {self.host}:{self.port}')
            return True

        except Exception as e:
            self.get_logger().warn(f'Cannot connect to {self.host}:{self.port}: {e}')
            self.sock = None
            time.sleep(2.0)
            return False

    def make_camera_info(self, width, height, stamp):
        """
        หมายเหตุ:
        ค่านี้เป็น camera_info แบบ placeholder
        ถ้าต้องการ pose ของ AprilTag ให้แม่น ควรใส่ calibration จริง
        """
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.width = width
        msg.height = height

        # ประมาณค่า focal length เบื้องต้น
        fx = float(width)
        fy = float(width)
        cx = width / 2.0
        cy = height / 2.0

        msg.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]

        msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = 'plumb_bob'

        return msg

    def loop(self):
        if self.sock is None:
            self.connect_socket()
            return

        try:
            now_time = time.time()
            min_dt = 1.0 / max(self.fps_limit, 1.0)

            header = recv_exact(self.sock, 4)
            if header is None:
                self.get_logger().warn('Connection lost. Reconnecting...')
                self.sock.close()
                self.sock = None
                return

            frame_size = struct.unpack('>I', header)[0]

            if frame_size <= 0 or frame_size > 10_000_000:
                self.get_logger().warn(f'Invalid frame size: {frame_size}')
                self.sock.close()
                self.sock = None
                return

            jpeg_data = recv_exact(self.sock, frame_size)
            if jpeg_data is None:
                self.get_logger().warn('Connection lost while reading frame. Reconnecting...')
                self.sock.close()
                self.sock = None
                return

            if now_time - self.last_pub_time < min_dt:
                return

            np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                self.get_logger().warn('Failed to decode JPEG frame')
                return

            stamp = self.get_clock().now().to_msg()

            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id

            height, width = frame.shape[:2]
            camera_info_msg = self.make_camera_info(width, height, stamp)

            self.image_pub.publish(image_msg)
            self.camera_info_pub.publish(camera_info_msg)

            self.last_pub_time = now_time

        except Exception as e:
            self.get_logger().warn(f'Camera stream error: {e}')
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBackTcpPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    try:
        if node.sock:
            node.sock.close()
    except Exception:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()