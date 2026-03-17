import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
import serial
import struct
import math
import time

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # คอนฟิก Serial Port (ปรับ Baudrate ให้ตรงกับ ESP32)
        self.ser = serial.Serial('/dev/esp32_drive', 115200, timeout=0.02)
        
        # Publishers & Subscribers
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # เราสามารถ Publish ข้อมูล Encoder ดิบๆ ออกไปให้ Node อื่นคำนวณ Odom ได้
        # หรือถ้าจะคำนวณ Odom ในนี้เลยก็ได้ (ผมจะส่งเป็น Array ของ Ticks ไปก่อนเพื่อความยืดหยุ่น)
        self.tick_pub = self.create_publisher(Int32MultiArray, '/wheel_ticks', 10)
        
        # Timer สำหรับอ่านข้อมูลจาก ESP32 (รันที่ประมาณ 50-100Hz เพื่อไม่ให้พลาดข้อมูล)
        self.create_timer(0.008, self.receive_feedback) 
        self.get_logger().info("ESP32 UART Bridge (Localization Ready) Started")

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
                        # แกะข้อมูล (float 2 ตัว, int32 2 ตัว)
                        # 'ffii' หมายถึง float, float, int, int (ตามลำดับ C struct)
                        v_m, w_m, left_ticks, right_ticks = struct.unpack('ffii', payload)
                        
                        # สร้าง Message เพื่อ Publish Ticks
                        tick_msg = Int32MultiArray()
                        tick_msg.data = [left_ticks, right_ticks]
                        self.tick_pub.publish(tick_msg)
                        
                    else:
                        self.get_logger().warn("Checksum Error Received!")

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