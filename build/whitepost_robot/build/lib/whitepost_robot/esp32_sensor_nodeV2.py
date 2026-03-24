#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu  # 🌟 เพิ่มการนำเข้า Message มาตรฐาน
import serial
import struct
import operator
from functools import reduce

class SensorSerialNode(Node):
    def __init__(self):
        super().__init__('esp32_sensor_node')

        self.port_name = '/dev/esp32_sensor'
        self.baud_rate = 921600
        
        try:
            self.serial_conn = serial.Serial()
            self.serial_conn.port = self.port_name
            self.serial_conn.baudrate = self.baud_rate
            self.serial_conn.dtr = False 
            self.serial_conn.rts = False 
            self.serial_conn.timeout = 0
            self.serial_conn.open()
        
            self.get_logger().info(f"✅ [Sensor] Binary Connected to {self.port_name} (Optimized for 200Hz)")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed: {e}")
            raise SystemExit

        # 🌟 เปลี่ยน Publisher เป็นมาตรฐาน Imu สำหรับ Isaac ROS
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # เก็บไว้เผื่อคุณยังต้องใช้ Debug หรือ Node อื่นเรียกใช้
        self.airpress_pub = self.create_publisher(Vector3, '/air/airpress', 10)

        self.buffer = bytearray()
        self.imu_struct = struct.Struct('<10f')
        self.bmp_struct = struct.Struct('<2f')

        # รันที่ 500Hz เพื่อเคลียร์ Buffer ให้ทันข้อมูล 200Hz จาก ESP32
        self.timer = self.create_timer(0.002, self.read_binary_data)

    def read_binary_data(self):
        waiting = self.serial_conn.in_waiting
        if waiting > 0:
            self.buffer.extend(self.serial_conn.read(waiting))

        while len(self.buffer) >= 12:
            idx = self.buffer.find(b'\xAA\xBB')
            
            if idx == -1:
                self.buffer.clear()
                break
            elif idx > 0:
                del self.buffer[:idx] 
                if len(self.buffer) < 12:
                    break

            msg_type = self.buffer[2]

            # --- IMU (44 Bytes) ---
            if msg_type == 1:
                if len(self.buffer) < 44:
                    break
                
                packet = self.buffer[:44]
                calc_cs = reduce(operator.xor, packet[2:43])
                
                if calc_cs == packet[43]:
                    floats = self.imu_struct.unpack_from(packet, 3)
                    
                    # 🌟 สร้าง Message ชนิด Imu
                    imu_msg = Imu()
                    
                    # 1. ใส่ Header (สำคัญมากสำหรับ VSLAM)
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link' # ต้องตรงกับใน URDF/TF
                    
                    # 2. ใส่ Linear Acceleration (m/s^2)
                    imu_msg.linear_acceleration.x = floats[0]
                    imu_msg.linear_acceleration.y = floats[1]
                    imu_msg.linear_acceleration.z = floats[2]
                    
                    # 3. ใส่ Angular Velocity (rad/s)
                    imu_msg.angular_velocity.x = floats[3]
                    imu_msg.angular_velocity.y = floats[4]
                    imu_msg.angular_velocity.z = floats[5]
                    
                    # 4. ใส่ Orientation (Quaternion)
                    imu_msg.orientation.x = floats[6]
                    imu_msg.orientation.y = floats[7]
                    imu_msg.orientation.z = floats[8]
                    imu_msg.orientation.w = floats[9]
                    
                    # 5. ส่งข้อมูลออกไป
                    self.imu_pub.publish(imu_msg)
                    
                else:
                    self.get_logger().warn(f"❌ IMU Checksum Fail!")

                del self.buffer[:44]

            # --- BMP (12 Bytes) ---
            elif msg_type == 2:
                if len(self.buffer) < 12:
                    break
                
                packet = self.buffer[:12]
                calc_cs = reduce(operator.xor, packet[2:11])
                
                if calc_cs == packet[11]:
                    floats = self.bmp_struct.unpack_from(packet, 3)
                    self.airpress_pub.publish(Vector3(x=floats[0], y=floats[1], z=0.0))
                else:
                    self.get_logger().warn("❌ BMP Checksum Fail!")

                del self.buffer[:12]
            else:
                del self.buffer[:2]

    def destroy_node(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

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