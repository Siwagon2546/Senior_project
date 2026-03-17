#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import serial
import struct
import operator
from functools import reduce

class DriveSerialNode(Node):
    def __init__(self):
        super().__init__('esp32_drive_node')

        self.port_name = '/dev/esp32_drive'
        self.baud_rate = 921600
        
        try:
            self.serial_conn = serial.Serial(self.port_name, self.baud_rate, timeout=0)
            self.get_logger().info(f"✅ [Drive] Binary Connected to {self.port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"❌ Failed: {e}")
            raise SystemExit

        self.status_pub = self.create_publisher(Vector3, '/drive_status', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.buffer = bytearray()
        
        # Struct definitions (Little Endian)
        self.odom_struct = struct.Struct('<3f') # 12 bytes
        self.cmd_struct = struct.Struct('<2f')  # 8 bytes

        self.timer = self.create_timer(0.01, self.read_binary_data) # 100Hz Poll Rate

    def cmd_vel_callback(self, msg):
        # สร้างแพ็กเกจ (ความยาวรวม 12 Bytes)
        payload = bytearray([0xAA, 0xBB, 0x11])
        payload.extend(self.cmd_struct.pack(msg.linear.x, msg.angular.z))
        
        # คำนวณ Checksum ตั้งแต่ Type ไปจนสุด Float (index 2 ถึง 10)
        cs = reduce(operator.xor, payload[2:11])
        payload.append(cs)
        
        # ส่งลง Serial
        self.serial_conn.write(payload)
        self.get_logger().info(f"Sent CMD: V={msg.linear.x:.2f}, W={msg.angular.z:.2f}")

    def read_binary_data(self):
        waiting = self.serial_conn.in_waiting
        if waiting > 0:
            self.buffer.extend(self.serial_conn.read(waiting))

        # 🌟 แก้เป็น 16 Bytes แล้ว!
        while len(self.buffer) >= 16: 
            idx = self.buffer.find(b'\xAA\xBB')
            
            if idx == -1:
                self.buffer.clear()
                break
            elif idx > 0:
                del self.buffer[:idx]
                if len(self.buffer) < 16:
                    break

            msg_type = self.buffer[2]

            # --- Odom (16 Bytes) ---
            if msg_type == 0x12:
                packet = self.buffer[:16]
                
                # Checksum: XOR จาก Type ไปจนสุดระยะทาง (index 2 ถึง 14)
                calc_cs = reduce(operator.xor, packet[2:15])
                
                if calc_cs == packet[15]:
                    floats = self.odom_struct.unpack_from(packet, 3)
                    
                    msg = Vector3()
                    msg.x = floats[0] # v
                    msg.y = floats[1] # omega
                    msg.z = floats[2] # dist
                    self.status_pub.publish(msg)
                    # self.get_logger().info(f"✅ Published: V={msg.x:.2f}, W={msg.y:.2f}, Dist={msg.z:.2f}")
                
                else:
                    self.get_logger().warn(f"❌ Odom Checksum Fail! Calc:{calc_cs} Recv:{packet[15]}")

                del self.buffer[:16]
            
            # --- ข้ามข้อความ Handshake (ถ้ามี) ---
            elif msg_type == ord('W'): 
                del self.buffer[:1]
            else:
                del self.buffer[:2]

    def destroy_node(self):
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DriveSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()