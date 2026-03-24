#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
import serial
import struct
import math

class DriveBridgeNode(Node):
    def __init__(self):
        super().__init__('esp32_drive_node')
        self.port_name = '/dev/esp32_drive'
        self.baud_rate = 115200
        self.serial_conn = None
        self.buffer = bytearray()
        
        # Odom State
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.last_update = self.get_clock().now()

        # Pub/Sub
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        self.connect_serial()
        self.create_timer(0.01, self.read_loop)      # 100Hz
        self.create_timer(1.0, self.watchdog_check)

    def connect_serial(self):
        try:
            if self.serial_conn: self.serial_conn.close()
            self.serial_conn = serial.Serial(self.port_name, self.baud_rate, timeout=0)
            self.get_logger().info(f"✅ Drive Serial Connected: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ Drive Connect Failed: {e}")
            self.serial_conn = None

    def watchdog_check(self):
        if (self.get_clock().now() - self.last_update).nanoseconds / 1e9 > 2.0:
            self.get_logger().warn("⚠️ Drive data timeout. Reconnecting...")
            self.connect_serial()

    def cmd_cb(self, msg):
        if not self.serial_conn or not self.serial_conn.is_open: return
        v, w = float(msg.linear.x), float(msg.angular.z)
        payload = struct.pack('<ff', v, w)
        checksum = sum(payload) & 0xFF
        packet = b'\xaa\x55' + payload + struct.pack('B', checksum)
        try:
            self.serial_conn.write(packet)
        except: pass

    def read_loop(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        try:
            if self.serial_conn.in_waiting > 0:
                self.buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))
        except: self.serial_conn = None; return

        while len(self.buffer) >= 19:
            idx = self.buffer.find(b'\xaa\x55')
            if idx == -1: self.buffer.clear(); break
            if idx > 0: del self.buffer[:idx]
            if len(self.buffer) < 19: break

            payload = self.buffer[2:18]
            if (sum(payload) & 0xFF) == self.buffer[18]:
                v_m, w_m, _, _ = struct.unpack('<ffii', payload)
                
                # Odometry Integration
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now
                
                self.x += v_m * math.cos(self.th) * dt
                self.y += v_m * math.sin(self.th) * dt
                self.th += w_m * dt
                
                # Publish Odometry
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = now.to_msg(), 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
                odom.pose.pose.orientation = self.yaw_to_quat(self.th)
                odom.twist.twist.linear.x, odom.twist.twist.angular.z = v_m, w_m
                self.odom_pub.publish(odom)
                self.last_update = now
                del self.buffer[:19]
            else:
                del self.buffer[:2]

    def yaw_to_quat(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))

def main():
    rclpy.init()
    rclpy.spin(DriveBridgeNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()