#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import serial
import struct
import operator
from functools import reduce

class SensorSerialNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')
        self.port_name = '/dev/esp32_sensor'
        self.baud_rate = 921600
        self.serial_conn = None
        self.buffer = bytearray()
        self.last_update = self.get_clock().now()
        
        # Struct definitions
        self.imu_struct = struct.Struct('<10f') # ax,ay,az,gx,gy,gz,qx,qy,qz,qw

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Connection Setup
        self.connect_serial()
        
        # Timers
        self.create_timer(0.002, self.read_loop)      # Read data (500Hz)
        self.create_timer(1.0, self.watchdog_check)  # Connection Watchdog

    def connect_serial(self):
        try:
            if self.serial_conn: self.serial_conn.close()
            self.serial_conn = serial.Serial(self.port_name, self.baud_rate, timeout=0, dtr=False, rts=False)
            self.get_logger().info(f"✅ IMU Serial Connected: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ IMU Connect Failed: {e}")
            self.serial_conn = None

    def watchdog_check(self):
        if (self.get_clock().now() - self.last_update).nanoseconds / 1e9 > 2.0:
            self.get_logger().warn("⚠️ IMU data timeout. Reconnecting...")
            self.connect_serial()

    def read_loop(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        
        try:
            if self.serial_conn.in_waiting > 0:
                self.buffer.extend(self.serial_conn.read(self.serial_conn.in_waiting))
        except Exception:
            self.serial_conn = None
            return

        while len(self.buffer) >= 12:
            idx = self.buffer.find(b'\xAA\xBB')
            if idx == -1: self.buffer.clear(); break
            if idx > 0: del self.buffer[:idx]
            if len(self.buffer) < 44: break # Check full packet size

            if self.buffer[2] == 1: # IMU Type
                packet = self.buffer[:44]
                if reduce(operator.xor, packet[2:43]) == packet[43]:
                    f = self.imu_struct.unpack_from(packet, 3)
                    msg = Imu()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'imu_link'
                    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = f[0:3]
                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = f[3:6]
                    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = f[6:10]
                    self.imu_pub.publish(msg)
                    self.last_update = self.get_clock().now()
                del self.buffer[:44]
            else:
                del self.buffer[:2]

def main():
    rclpy.init()
    rclpy.spin(SensorSerialNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()