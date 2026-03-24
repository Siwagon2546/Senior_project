import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import serial
import struct
import math
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # คอนฟิก Serial Port (ปรับ Baudrate ให้ตรงกับ ESP32)
        self.ser = serial.Serial('/dev/esp32_drive', 115200, timeout=0.02)
        
        # Publishers & Subscribers
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # เราสามารถ Publish ข้อมูล Encoder ดิบๆ ออกไปให้ Node อื่นคำนวณ Odom ได้
        # หรือถ้าจะคำนวณ Odom ในนี้เลยก็ได้ (ผมจะส่งเป็น Array ของ Ticks ไปก่อนเพื่อความยืดหยุ่น)
        # self.tick_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        
        # Timer สำหรับอ่านข้อมูลจาก ESP32 (รันที่ประมาณ 50-100Hz เพื่อไม่ให้พลาดข้อมูล)
        self.create_timer(0.008, self.receive_feedback) 
        self.get_logger().info("ESP32 UART Bridge (Localization Ready) Started")

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        
        # เปลี่ยน Publisher เป็น Odometry
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # (Optional) ถ้าต้องการให้ Node นี้ส่ง TF เลย
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def cmd_callback(self, msg):
        # เตรียม Packet: Header(AA 55) + v(float) + w(float) + checksum(uint8)
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        payload = struct.pack('ff', v, w)
        checksum = sum(payload) % 256
        
        packet = b'\xaa\x55' + payload + struct.pack('B', checksum)
        self.ser.write(packet)

    def receive_feedback(self):
        # อ่านข้อมูลจนกว่า Buffer จะหมด
        while self.ser.in_waiting >= 19: # ขนาด FeedbackPacket ใหม่คือ 19 bytes
            # ค้นหา Header AA 55
            header = self.ser.read(1)
            if header == b'\xaa':
                if self.ser.read(1) == b'\x55':
                    raw_data = self.ser.read(17) # อ่าน Payload (16 bytes) + Checksum (1 byte)
                    if len(raw_data) < 17:
                        continue
                    
                    payload = raw_data[:16]
                    received_checksum = raw_data[16]
                    
                    # ตรวจสอบ Checksum
                    calculated_checksum = sum(payload) % 256
                    
                    if calculated_checksum == received_checksum:
                        v_m, w_m, left_ticks, right_ticks = struct.unpack('ffii', payload)
                        
                        # --- ส่วนการคำนวณ Odometry ---
                        current_time = self.get_clock().now()
                        dt = (current_time - self.last_time).nanoseconds / 1e9
                        self.last_time = current_time
                        
                        # Integration (หาตำแหน่งปัจจุบัน)
                        # สูตร: delta_x = v * cos(theta) * dt
                        self.x += v_m * math.cos(self.th) * dt
                        self.y += v_m * math.sin(self.th) * dt
                        self.th += w_m * dt
                        
                        # สร้าง Quaternion จาก Yaw (th)
                        q = self.euler_to_quaternion(0, 0, self.th)
                        
                        # สร้างและส่ง Message Odometry
                        odom = Odometry()
                        odom.header.stamp = current_time.to_msg()
                        odom.header.frame_id = 'odom'
                        odom.child_frame_id = 'base_link'
                        
                        # Pose (ตำแหน่ง)
                        odom.pose.pose.position.x = self.x
                        odom.pose.pose.position.y = self.y
                        odom.pose.pose.orientation = q
                        
                        # Twist (ความเร็ว)
                        odom.twist.twist.linear.x = v_m
                        odom.twist.twist.angular.z = w_m
                        
                        self.odom_pub.publish(odom)
    def euler_to_quaternion(self, roll, pitch, yaw):
        # ฟังก์ชันช่วยแปลงมุมเป็น Quaternion
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()