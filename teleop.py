import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

class TeleopGUI(Node):
    def __init__(self):
        super().__init__('cmd_vel_gui_tester')
        
        # สร้าง Publisher ส่งค่าไปยัง Topic /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()

        # --- สร้างหน้าต่าง UI ด้วย Tkinter ---
        self.root = tk.Tk()
        self.root.title("Robot /cmd_vel Tester")
        self.root.geometry("350x250")

        # ป้ายข้อความ
        tk.Label(self.root, text="ปรับค่าความเร็วหุ่นยนต์", font=("Arial", 14)).pack(pady=10)

        # Slider สำหรับความเร็วเดินหน้า-ถอยหลัง (Linear X)
        self.linear_scale = tk.Scale(self.root, from_=1.0, to=-1.0, resolution=0.05, 
                                     orient=tk.HORIZONTAL, label="เดินหน้า / ถอยหลัง (Linear X : m/s)", 
                                     length=250)
        self.linear_scale.pack()

        # Slider สำหรับความเร็วหมุนซ้าย-ขวา (Angular Z)
        self.angular_scale = tk.Scale(self.root, from_=2.0, to=-2.0, resolution=0.1, 
                                      orient=tk.HORIZONTAL, label="หมุนซ้าย / ขวา (Angular Z : rad/s)", 
                                      length=250)
        self.angular_scale.pack()

        # ปุ่มหยุดฉุกเฉิน (รีเซ็ตค่าเป็น 0)
        self.stop_btn = tk.Button(self.root, text="🛑 STOP (รีเซ็ตเป็น 0)", 
                                  command=self.stop_robot, bg="red", fg="white", font=("Arial", 12, "bold"))
        self.stop_btn.pack(pady=15)

        # สร้าง Timer ให้ Publish ค่าความเร็วทุกๆ 0.1 วินาที (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def stop_robot(self):
        # เซ็ต Slider กลับมาที่ 0
        self.linear_scale.set(0.0)
        self.angular_scale.set(0.0)
        self.publish_velocity() # ส่งคำสั่งหยุดทันที

    def publish_velocity(self):
        # ดึงค่าจาก Slider มาใส่ใน Message Twist
        self.twist_msg.linear.x = float(self.linear_scale.get())
        self.twist_msg.angular.z = float(self.angular_scale.get())
        
        # Publish คำสั่ง
        self.publisher_.publish(self.twist_msg)

    def run(self):
        # ฟังก์ชันสำหรับให้ ROS 2 และ Tkinter ทำงานพร้อมกันได้
        def spin_ros():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.after(10, spin_ros) # เรียกตัวเองทุก 10ms

        spin_ros()
        self.root.mainloop() # เริ่มการทำงานของ UI

def main(args=None):
    rclpy.init(args=args)
    gui_node = TeleopGUI()
    
    try:
        gui_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()