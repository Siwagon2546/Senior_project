#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Bool
# from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    # yaw (Z axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ImuRotateTestNode(Node):
    def __init__(self):
        super().__init__('imu_rotate_test_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_topic', '/rotate_target_deg')
        self.declare_parameter('cancel_topic', '/rotate_cancel')

        self.declare_parameter('control_rate_hz', 50.0)

        # PID
        self.declare_parameter('kp', 3.2)
        self.declare_parameter('ki', 0.02)
        self.declare_parameter('kd', 0.5)

        # limits
        self.declare_parameter('max_angular_speed', 1.2)   # rad/s
        self.declare_parameter('min_angular_speed', 0.12)  # rad/s กันมอเตอร์ไม่ขยับ
        self.declare_parameter('max_angular_accel', 3.0)   # rad/s^2

        # finish conditions
        self.declare_parameter('yaw_tolerance_deg', 1.5)
        self.declare_parameter('settle_time_sec', 0.25)
        self.declare_parameter('stop_gyro_threshold', 0.03)  # rad/s
        self.declare_parameter('imu_timeout_sec', 0.3)

        # source selection
        self.declare_parameter('use_orientation_if_available', True)

        # debug
        self.declare_parameter('publish_debug', True)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_topic = self.get_parameter('target_topic').value
        self.cancel_topic = self.get_parameter('cancel_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.max_w = float(self.get_parameter('max_angular_speed').value)
        self.min_w = float(self.get_parameter('min_angular_speed').value)
        self.max_accel = float(self.get_parameter('max_angular_accel').value)

        self.yaw_tolerance = math.radians(float(self.get_parameter('yaw_tolerance_deg').value))
        self.settle_time_sec = float(self.get_parameter('settle_time_sec').value)
        self.stop_gyro_threshold = float(self.get_parameter('stop_gyro_threshold').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)

        self.use_orientation_if_available = bool(
            self.get_parameter('use_orientation_if_available').value
        )
        self.publish_debug = bool(self.get_parameter('publish_debug').value)

        # ---------------- ROS interfaces ----------------
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            imu_qos
        )
        self.target_sub = self.create_subscription(Float64, self.target_topic, self.target_callback, 10)
        self.cancel_sub = self.create_subscription(Bool, self.cancel_topic, self.cancel_callback, 10)

        if self.publish_debug:
            self.err_pub = self.create_publisher(Float64, '/rotate_test/error_deg', 10)
            self.cur_pub = self.create_publisher(Float64, '/rotate_test/current_yaw_deg', 10)
            self.tgt_pub = self.create_publisher(Float64, '/rotate_test/target_yaw_deg', 10)
            self.cmd_dbg_pub = self.create_publisher(Float64, '/rotate_test/cmd_wz', 10)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        # ---------------- State ----------------
        self.current_yaw = 0.0              # continuous yaw
        self.last_raw_yaw = None            # wrapped yaw from quaternion
        self.last_gyro_z = 0.0
        self.last_imu_time = None
        self.last_imu_local_sec = None
        self.yaw_ready = False

        self.active = False
        self.target_yaw = 0.0

        self.integral = 0.0
        self.last_error = None
        self.last_cmd = 0.0
        self.last_control_time = None
        self.settle_start_time = None

        self.get_logger().info('IMU rotate test node started')
        self.get_logger().info(f'IMU topic    : {self.imu_topic}')
        self.get_logger().info(f'cmd_vel topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'target topic : {self.target_topic}')

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def imu_callback(self, msg: Imu) -> None:
        now = self.now_sec()

        if self.last_imu_local_sec is None:
            dt = 0.0
        else:
            dt = now - self.last_imu_local_sec
            dt = clamp(dt, 0.0, 0.1)

        self.last_imu_local_sec = now
        self.last_imu_time = now
        self.last_gyro_z = float(msg.angular_velocity.z)

        orientation_valid = True
        if len(msg.orientation_covariance) > 0 and msg.orientation_covariance[0] == -1.0:
            orientation_valid = False

        if self.use_orientation_if_available and orientation_valid:
            raw_yaw = quat_to_yaw(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )

            if not self.yaw_ready:
                self.current_yaw = raw_yaw
                self.last_raw_yaw = raw_yaw
                self.yaw_ready = True
            else:
                delta = wrap_to_pi(raw_yaw - self.last_raw_yaw)
                self.current_yaw += delta
                self.last_raw_yaw = raw_yaw
        else:
            # fallback: integrate gyro z
            if not self.yaw_ready:
                self.current_yaw = 0.0
                self.yaw_ready = True

            if dt > 0.0:
                self.current_yaw += self.last_gyro_z * dt

    def target_callback(self, msg: Float64) -> None:
        if not self.yaw_ready:
            self.get_logger().warn('IMU yaw not ready yet, ignore target command')
            return

        relative_deg = float(msg.data)
        relative_rad = math.radians(relative_deg)

        self.target_yaw = self.current_yaw + relative_rad
        self.active = True

        self.integral = 0.0
        self.last_error = None
        self.settle_start_time = None

        self.get_logger().info(
            f'New target: rotate {relative_deg:.2f} deg '
            f'(current={math.degrees(self.current_yaw):.2f} deg, '
            f'target={math.degrees(self.target_yaw):.2f} deg)'
        )

    def cancel_callback(self, msg: Bool) -> None:
        if msg.data:
            self.stop_robot('Rotation cancelled')

    def publish_cmd(self, wz: float) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(wz)
        self.cmd_pub.publish(twist)

    def stop_robot(self, reason: str = '') -> None:
        self.active = False
        self.integral = 0.0
        self.last_error = None
        self.last_cmd = 0.0
        self.settle_start_time = None
        self.publish_cmd(0.0)
        if reason:
            self.get_logger().info(reason)

    def control_loop(self) -> None:
        now = self.now_sec()

        if self.last_control_time is None:
            self.last_control_time = now
            return

        dt = now - self.last_control_time
        self.last_control_time = now
        if dt <= 0.0:
            return

        if not self.yaw_ready:
            return

        if self.last_imu_time is None or (now - self.last_imu_time) > self.imu_timeout_sec:
            if self.active:
                self.stop_robot('IMU timeout, stop robot')
            return

        if not self.active:
            return

        error = self.target_yaw - self.current_yaw

        # stop condition: อยู่ใน tolerance และ gyro ใกล้หยุดจริง
        if abs(error) < self.yaw_tolerance and abs(self.last_gyro_z) < self.stop_gyro_threshold:
            if self.settle_start_time is None:
                self.settle_start_time = now
            elif (now - self.settle_start_time) >= self.settle_time_sec:
                final_err_deg = math.degrees(error)
                self.stop_robot(f'Rotation done, final error={final_err_deg:.3f} deg')
                return
        else:
            self.settle_start_time = None

        # PID
        self.integral += error * dt
        self.integral = clamp(self.integral, -1.0, 1.0)  # anti-windup แบบง่าย

        derivative = 0.0 if self.last_error is None else (error - self.last_error) / dt
        raw_cmd = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # minimum output compensation
        if abs(error) > self.yaw_tolerance and abs(raw_cmd) < self.min_w:
            raw_cmd = math.copysign(self.min_w, error)

        cmd = clamp(raw_cmd, -self.max_w, self.max_w)

        # acceleration limiting
        max_step = self.max_accel * dt
        cmd = clamp(cmd, self.last_cmd - max_step, self.last_cmd + max_step)

        self.publish_cmd(cmd)

        self.last_cmd = cmd
        self.last_error = error

        if self.publish_debug:
            m1 = Float64()
            m1.data = math.degrees(error)
            self.err_pub.publish(m1)

            m2 = Float64()
            m2.data = math.degrees(self.current_yaw)
            self.cur_pub.publish(m2)

            m3 = Float64()
            m3.data = math.degrees(self.target_yaw)
            self.tgt_pub.publish(m3)

            m4 = Float64()
            m4.data = cmd
            self.cmd_dbg_pub.publish(m4)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRotateTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot('Shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()