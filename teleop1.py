import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_msg = Twist()

        # ตั้งค่าความเร็วสูงสุดและอัตราการเร่ง
        self.max_linear = 0.2   # m/s
        self.max_angular = 1.0  # rad/s
        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # --- สร้าง UI ---
        self.root = tk.Tk()
        self.root.title("ROS 2 Keyboard Control")
        self.root.geometry("400x300")

        # ส่วนแสดงผลความเร็วบนหน้าจอ
        self.label = tk.Label(self.root, text="ใช้ W A S D หรือ Arrow Keys ในการควบคุม", font=("Arial", 12))
        self.label.pack(pady=20)
        
        self.status_label = tk.Label(self.root, text="Linear: 0.0 | Angular: 0.0", font=("Courier", 14, "bold"))
        self.status_label.pack(pady=10)

        tk.Label(self.root, text="Space: STOP", fg="red").pack()

        # --- Bind Keys ---
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)

        # Timer สำหรับส่งค่าไปที่ Robot (10Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)

    def on_key_press(self, event):
        key = event.keysym.lower()
        
        # ควบคุม Linear X
        if key in ['w', 'up']:
            self.linear_vel = self.max_linear
        elif key in ['s', 'down']:
            self.linear_vel = -self.max_linear
            
        # ควบคุม Angular Z
        if key in ['a', 'left']:
            self.angular_vel = self.max_angular
        elif key in ['d', 'right']:
            self.angular_vel = -self.max_angular
            
        # ปุ่มหยุด (Space)
        if key == 'space':
            self.stop_robot()

    def on_key_release(self, event):
        key = event.keysym.lower()
        # เมื่อปล่อยปุ่ม ให้หยุดการเคลื่อนที่ในแกนนั้นๆ
        if key in ['w', 's', 'up', 'down']:
            self.linear_vel = 0.0
        if key in ['a', 'd', 'left', 'right']:
            self.angular_vel = 0.0

    def stop_robot(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0

    def publish_velocity(self):
        self.twist_msg.linear.x = float(self.linear_vel)
        self.twist_msg.angular.z = float(self.angular_vel)
        self.publisher_.publish(self.twist_msg)
        
        # อัปเดตข้อความบน UI
        self.status_label.config(
            text=f"Linear: {self.linear_vel:.2f} | Angular: {self.angular_vel:.2f}"
        )

    def run(self):
        def spin_ros():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.after(10, spin_ros)
        
        spin_ros()
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()