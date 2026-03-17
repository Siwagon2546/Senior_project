#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, Quaternion
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
            # 🌟 แก้ไขตรงนี้: ปรับวิธีเปิด Serial เพื่อป้องกันไม่ให้ ESP32 รีเซ็ตตัวเอง
            self.serial_conn = serial.Serial()
            self.serial_conn.port = self.port_name
            self.serial_conn.baudrate = self.baud_rate
            self.serial_conn.dtr = False # ห้ามส่งสัญญาณ Reset
            self.serial_conn.rts = False # ห้ามส่งสัญญาณ Reset
            self.serial_conn.timeout = 0
            self.serial_conn.open()
        
            self.get_logger().info(f"✅ [Sensor] Binary Connected to {self.port_name} (Optimized for 200Hz)")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed: {e}")
            raise SystemExit

        self.accel_pub = self.create_publisher(Vector3, '/bno/accel', 10)
        self.gyro_pub = self.create_publisher(Vector3, '/bno/gyro', 10)
        self.quat_pub = self.create_publisher(Quaternion, '/bno/quaternion', 10)
        self.airpress_pub = self.create_publisher(Vector3, '/air/airpress', 10)

        self.buffer = bytearray()
        
        # 🌟 OPTIMIZE 1: Pre-compile struct เพื่อความเร็วสูงสุด
        self.imu_struct = struct.Struct('<10f')
        self.bmp_struct = struct.Struct('<2f')

        # 🌟 OPTIMIZE 2: ให้ Timer วิ่งเร็วกว่าข้อมูลเข้า (วิ่งที่ 500Hz = 0.002s) 
        # เพื่อเคลียร์ข้อมูลให้ทัน 200Hz ของ ESP32
        self.timer = self.create_timer(0.002, self.read_binary_data)

    def read_binary_data(self):
        waiting = self.serial_conn.in_waiting
        if waiting > 0:
            # ใช้ extend เพื่อต่อท้าย Buffer อย่างรวดเร็ว
            self.buffer.extend(self.serial_conn.read(waiting))

        while len(self.buffer) >= 12:
            idx = self.buffer.find(b'\xAA\xBB')
            
            if idx == -1:
                self.buffer.clear()
                break
            elif idx > 0:
                # 🌟 OPTIMIZE 3: ใช้ del ลบ in-place ดีกว่าการหั่น Array
                del self.buffer[:idx] 
                if len(self.buffer) < 12:
                    break

            msg_type = self.buffer[2]

            # --- IMU (44 Bytes) ---
            if msg_type == 1:
                if len(self.buffer) < 44:
                    break
                
                packet = self.buffer[:44]
                
                # 🌟 OPTIMIZE 4: คำนวณ Checksum ด้วย C-level ของ Python (เร็วติดจรวด)
                calc_cs = reduce(operator.xor, packet[2:43])
                
                if calc_cs == packet[43]:
                    # Unpack แบบรวดเร็ว
                    floats = self.imu_struct.unpack_from(packet, 3)
                    
                    self.accel_pub.publish(Vector3(x=floats[0], y=floats[1], z=floats[2]))
                    self.gyro_pub.publish(Vector3(x=floats[3], y=floats[4], z=floats[5]))
                    self.quat_pub.publish(Quaternion(x=floats[6], y=floats[7], z=floats[8], w=floats[9]))
                    
                    # 🚫 เอา print/log ตรงนี้ออกเด็ดขาด!
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
                del self.buffer[:2] # ชนิดข้อมูลไม่ตรง เตะ Header ทิ้ง

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