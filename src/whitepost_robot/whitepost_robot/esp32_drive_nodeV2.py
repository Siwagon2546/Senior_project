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
        
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.last_update = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        self.connect_serial()
        self.create_timer(0.01, self.read_loop)      # 100Hz
        self.create_timer(3.0, self.watchdog_check)  # ขยับเป็น 3 วินาที

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
            self.get_logger().info(f"✅ Drive Serial Connected: {self.port_name}")
        except Exception as e:
            self.get_logger().error(f"❌ Drive Connect Failed: {e}")
            self.serial_conn = None

    def watchdog_check(self):
        diff = (self.get_clock().now() - self.last_update).nanoseconds / 1e9
        if diff > 5.0: # ปรับให้ใจเย็นขึ้นเป็น 5 วินาที
            self.get_logger().warn(f"⚠️ Drive Timeout ({diff:.1f}s). No valid packets received. Reconnecting...")
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
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                raw_in = self.serial_conn.read(waiting)
                self.buffer.extend(raw_in)
                # 🛠️ DEBUG: ถ้าอยากเห็นว่ามีอะไรเข้ามาบ้าง ให้เอาคอมเมนต์บรรทัดข้างล่างออก
                # self.get_logger().info(f"Raw bytes in: {raw_in.hex()}")
        except Exception as e:
            self.get_logger().error(f"Read error: {e}")
            self.serial_conn = None
            return

        while len(self.buffer) >= 19:
            idx = self.buffer.find(b'\xaa\x55')
            if idx == -1: 
                # ถ้าไม่เจอ Header เลยแต่มีข้อมูลค้างอยู่เยอะ ให้ล้างทิ้งบ้าง
                if len(self.buffer) > 100: self.buffer.clear()
                break
            if idx > 0: del self.buffer[:idx]
            if len(self.buffer) < 19: break

            payload = self.buffer[2:18]
            received_cs = self.buffer[18]
            calc_cs = sum(payload) & 0xFF

            if calc_cs == received_cs:
                v_m, w_m, _, _ = struct.unpack('<ffii', payload)
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now
                
                self.x += v_m * math.cos(self.th) * dt
                self.y += v_m * math.sin(self.th) * dt
                self.th += w_m * dt
                
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id = now.to_msg(), 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
                odom.pose.pose.orientation = self.yaw_to_quat(self.th)
                odom.twist.twist.linear.x, odom.twist.twist.angular.z = v_m, w_m
                
                self.odom_pub.publish(odom)
                self.last_update = now # อัปเดตเมื่อได้ข้อมูลที่ถูกต้องเท่านั้น
                del self.buffer[:19]
            else:
                self.get_logger().warn(f"Checksum mismatch: calc {calc_cs} != recv {received_cs}")
                del self.buffer[:2] # เตะ Header ทิ้งแล้วหาใหม่

    def yaw_to_quat(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2), w=math.cos(yaw/2))

def main(args=None):
    rclpy.init(args=args)
    node = DriveBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()