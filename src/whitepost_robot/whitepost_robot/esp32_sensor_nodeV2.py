#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
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
        
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.imu_struct = struct.Struct('<10f')

        self.connect_serial()
        self.create_timer(0.002, self.read_loop)
        self.create_timer(2.0, self.watchdog_check)

    def connect_serial(self):
        try:
            if self.serial_conn: self.serial_conn.close()
            self.serial_conn = serial.Serial()
            self.serial_conn.port = self.port_name
            self.serial_conn.baudrate = self.baud_rate
            self.serial_conn.timeout = 0
            self.serial_conn.dtr = False
            self.serial_conn.rts = False
            self.serial_conn.open()
            self.last_update = self.get_clock().now() 
            self.get_logger().info(f"✅ IMU Serial Connected: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ IMU Connect Failed: {e}")
            self.serial_conn = None

    def watchdog_check(self):
        diff = (self.get_clock().now() - self.last_update).nanoseconds / 1e9
        if diff > 3.0:
            self.get_logger().warn(f"⚠️ IMU Timeout ({diff:.1f}s). Reconnecting...")
            self.connect_serial()

    def read_loop(self):
        if not self.serial_conn or not self.serial_conn.is_open: return
        try:
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                self.buffer.extend(self.serial_conn.read(waiting))
        except: self.serial_conn = None; return

        while len(self.buffer) >= 44:
            idx = self.buffer.find(b'\xAA\xBB')
            if idx == -1: self.buffer.clear(); break
            if idx > 0: del self.buffer[:idx]
            if len(self.buffer) < 44: break

            if self.buffer[2] == 1:
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
            else:
                del self.buffer[:2]

# 🌟 ฟังก์ชัน Main ที่หายไป
def main(args=None):
    rclpy.init(args=args)
    node = SensorSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()