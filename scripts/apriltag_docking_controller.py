#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray


class MultiAprilTagDockingController(Node):
    def __init__(self):
        super().__init__('multi_apriltag_docking_controller')

        # =========================
        # Parameters
        # =========================
        self.declare_parameter('target_tag_ids', [19, 16])
        self.declare_parameter('target_distance', 0.40)
        self.declare_parameter('distance_tolerance', 0.05)
        self.declare_parameter('center_tolerance', 0.04)

        self.declare_parameter('kp_linear', 0.20)
        self.declare_parameter('kp_angular', 0.70)

        self.declare_parameter('max_linear_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.20)

        self.declare_parameter('lost_tag_timeout', 0.5)

        # จำตำแหน่งล่าสุดของ tag กรณี detection ติด ๆ ดับ ๆ
        self.declare_parameter('tag_memory_timeout', 2.0)
        self.declare_parameter('use_last_pose_when_lost', True)
        self.declare_parameter('lost_linear_scale', 0.0)
        self.declare_parameter('lost_angular_scale', 0.6)

        self.declare_parameter('tag_topic', '/camera_back/tag_detections')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # กล้องหลัง: True = ถอยเข้า Dock
        self.declare_parameter('reverse_mode', False)

        # ถ้าหมุนเข้าหา tag ผิดทาง ให้เปลี่ยนเป็น True
        self.declare_parameter('reverse_angular', False)

        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('debug_log', True)

        # หลัง Dock tag เสร็จ ให้หยุดกี่วินาทีก่อนทำ step ถัดไป
        self.declare_parameter('pause_after_dock_sec', 1.0)

        # ทิศทางหมุนหา tag ถัดไป: left หรือ right
        self.declare_parameter('search_turn_direction', 'left')

        # ความเร็วตอนหมุนหา tag ถัดไป
        self.declare_parameter('search_angular_speed', 0.18)

        # จำกัดเวลาหมุนหา tag แต่ละตัว ป้องกันหมุนไม่หยุด
        self.declare_parameter('search_timeout_sec', 20.0)

        self.target_tag_ids = list(self.get_parameter('target_tag_ids').value)

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.distance_tolerance = float(self.get_parameter('distance_tolerance').value)
        self.center_tolerance = float(self.get_parameter('center_tolerance').value)

        self.kp_linear = float(self.get_parameter('kp_linear').value)
        self.kp_angular = float(self.get_parameter('kp_angular').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.lost_tag_timeout = float(self.get_parameter('lost_tag_timeout').value)

        self.tag_memory_timeout = float(self.get_parameter('tag_memory_timeout').value)
        self.use_last_pose_when_lost = bool(self.get_parameter('use_last_pose_when_lost').value)
        self.lost_linear_scale = float(self.get_parameter('lost_linear_scale').value)
        self.lost_angular_scale = float(self.get_parameter('lost_angular_scale').value)

        self.tag_topic = self.get_parameter('tag_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.reverse_mode = bool(self.get_parameter('reverse_mode').value)
        self.reverse_angular = bool(self.get_parameter('reverse_angular').value)

        self.x_offset = float(self.get_parameter('x_offset').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        self.pause_after_dock_sec = float(self.get_parameter('pause_after_dock_sec').value)
        self.search_turn_direction = str(self.get_parameter('search_turn_direction').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.search_timeout_sec = float(self.get_parameter('search_timeout_sec').value)

        if len(self.target_tag_ids) == 0:
            raise RuntimeError('target_tag_ids is empty')

        if self.search_turn_direction not in ['left', 'right']:
            raise RuntimeError('search_turn_direction must be "left" or "right"')

        # =========================
        # State
        # =========================
        self.current_index = 0
        self.current_target_id = int(self.target_tag_ids[self.current_index])

        # สำคัญ:
        # tag แรกไม่ต้องหมุนหา ให้หยุดรอจนเห็นเอง
        self.state = 'WAIT_FIRST_TAG'
        self.state_start_time = self.get_clock().now()

        self.last_seen_time = None
        self.latest_x = None
        self.latest_z = None
        self.latest_tag_id = None

        self.visible_tags = {}

        # =========================
        # ROS Interface
        # =========================
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.tag_topic,
            self.tag_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            'Multi AprilTag Docking Controller started | '
            f'tag_topic={self.tag_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, '
            f'target_tag_ids={self.target_tag_ids}, '
            f'current_target_id={self.current_target_id}, '
            f'target_distance={self.target_distance:.3f}, '
            f'reverse_mode={self.reverse_mode}, '
            f'reverse_angular={self.reverse_angular}, '
            f'initial_state={self.state}, '
            f'search_turn_direction={self.search_turn_direction}, '
            f'tag_memory_timeout={self.tag_memory_timeout}'
        )

    # =========================
    # Utility
    # =========================
    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def now_sec_from(self, start_time):
        now = self.get_clock().now()
        return (now - start_time).nanoseconds / 1e9

    def set_state(self, new_state):
        self.state = new_state
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'STATE -> {self.state} | '
            f'current_target_id={self.current_target_id}, '
            f'index={self.current_index + 1}/{len(self.target_tag_ids)}'
        )

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def publish_cmd(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_pub.publish(cmd)

    def get_detection_id(self, detection):
        if not hasattr(detection, 'id'):
            return None

        tag_id = detection.id

        if isinstance(tag_id, list):
            if len(tag_id) == 0:
                return None
            return int(tag_id[0])

        try:
            return int(tag_id)
        except Exception:
            return None

    def get_position(self, detection):
        try:
            return detection.pose.pose.pose.position
        except Exception:
            pass

        try:
            return detection.pose.pose.position
        except Exception:
            pass

        try:
            return detection.pose.position
        except Exception:
            pass

        return None

    # =========================
    # AprilTag Callback
    # =========================
    def tag_callback(self, msg):
        self.visible_tags = {}

        for detection in msg.detections:
            tag_id = self.get_detection_id(detection)
            position = self.get_position(detection)

            if tag_id is None or position is None:
                continue

            self.visible_tags[tag_id] = {
                'x': float(position.x),
                'z': float(position.z),
                'stamp': self.get_clock().now()
            }

        if self.current_target_id not in self.visible_tags:
            return

        tag_data = self.visible_tags[self.current_target_id]

        self.latest_tag_id = self.current_target_id
        self.latest_x = tag_data['x']
        self.latest_z = tag_data['z']
        self.last_seen_time = tag_data['stamp']

        if self.state in ['WAIT_FIRST_TAG', 'SEARCH_TAG']:
            self.get_logger().info(
                f'Found target tag {self.current_target_id} | '
                f'x={self.latest_x:.3f}, z={self.latest_z:.3f}'
            )
            self.set_state('DOCKING')

    # =========================
    # Control Logic
    # =========================
    def control_loop(self):
        if self.state == 'WAIT_FIRST_TAG':
            self.handle_wait_first_tag()
            return

        if self.state == 'SEARCH_TAG':
            self.handle_search_tag()
            return

        if self.state == 'DOCKING':
            self.handle_docking()
            return

        if self.state == 'PAUSE_AFTER_DOCK':
            self.handle_pause_after_dock()
            return

        if self.state == 'COMPLETE':
            self.stop_robot()
            return

        self.stop_robot()

    def handle_wait_first_tag(self):
        # tag แรกไม่หมุนหา หยุดรอเฉย ๆ
        self.stop_robot()

        if self.debug_log:
            self.get_logger().info(
                f'waiting first tag_id={self.current_target_id} without search rotation'
            )

    def handle_search_tag(self):
        elapsed = self.now_sec_from(self.state_start_time)

        if elapsed > self.search_timeout_sec:
            self.get_logger().warn(
                f'Search timeout for tag {self.current_target_id}. Stop robot.'
            )
            self.stop_robot()
            return

        direction_sign = 1.0 if self.search_turn_direction == 'left' else -1.0
        angular_z = direction_sign * abs(self.search_angular_speed)

        self.publish_cmd(0.0, angular_z)

        if self.debug_log:
            self.get_logger().info(
                f'searching tag_id={self.current_target_id} | '
                f'turn={self.search_turn_direction}, wz={angular_z:.3f}'
            )

    def handle_docking(self):
        if self.last_seen_time is None:
            self.stop_robot()

            if self.current_index == 0:
                self.set_state('WAIT_FIRST_TAG')
            else:
                self.set_state('SEARCH_TAG')
            return

        dt = self.now_sec_from(self.last_seen_time)

        tag_is_fresh = dt <= self.lost_tag_timeout
        tag_is_in_memory = dt <= self.tag_memory_timeout

        if not tag_is_in_memory:
            if self.debug_log:
                self.get_logger().warn(
                    f'Tag {self.current_target_id} lost for {dt:.2f}s.'
                )

            self.latest_x = None
            self.latest_z = None
            self.latest_tag_id = None
            self.last_seen_time = None

            self.stop_robot()

            if self.current_index == 0:
                self.set_state('WAIT_FIRST_TAG')
            else:
                self.set_state('SEARCH_TAG')

            return

        using_memory_pose = (not tag_is_fresh) and self.use_last_pose_when_lost

        x_error = self.latest_x - self.x_offset
        z_error = self.latest_z - self.target_distance

        # Dock complete เฉพาะข้อมูล live สดเท่านั้น
        if tag_is_fresh:
            if abs(z_error) < self.distance_tolerance and abs(x_error) < self.center_tolerance:
                self.get_logger().info(
                    f'Docking complete for tag {self.current_target_id} | '
                    f'x={self.latest_x:.3f}, z={self.latest_z:.3f}'
                )

                self.stop_robot()
                self.set_state('PAUSE_AFTER_DOCK')
                return

        # Angular control
        angular_sign = 1.0 if self.reverse_angular else -1.0
        angular_cmd = angular_sign * self.kp_angular * x_error

        angular_cmd = self.clamp(
            angular_cmd,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if using_memory_pose:
            angular_cmd *= self.lost_angular_scale

        # Linear control
        if abs(x_error) < 0.12:
            linear_cmd = self.kp_linear * z_error

            linear_cmd = self.clamp(
                linear_cmd,
                -self.max_linear_speed,
                self.max_linear_speed
            )

            if self.reverse_mode:
                # กล้องหลัง: ถอยหลังเข้า tag
                if z_error > self.distance_tolerance:
                    linear_cmd = -abs(linear_cmd)
                else:
                    linear_cmd = 0.0
            else:
                # กล้องหน้า: เดินหน้าเข้า tag
                if z_error > self.distance_tolerance:
                    linear_cmd = abs(linear_cmd)
                elif z_error < -self.distance_tolerance:
                    linear_cmd = -min(abs(linear_cmd), 0.05)
                else:
                    linear_cmd = 0.0
        else:
            linear_cmd = 0.0

        # ถ้าใช้ตำแหน่งจำล่าสุด ไม่ให้เดิน/ถอย ใช้แค่หมุนหาแนวเดิม
        if using_memory_pose:
            linear_cmd *= self.lost_linear_scale

        self.publish_cmd(linear_cmd, angular_cmd)

        if self.debug_log:
            pose_status = 'MEMORY' if using_memory_pose else 'LIVE'
            self.get_logger().info(
                f'docking tag={self.current_target_id} [{pose_status}] | '
                f'dt={dt:.2f}s, '
                f'x={self.latest_x:.3f}, z={self.latest_z:.3f}, '
                f'x_error={x_error:.3f}, z_error={z_error:.3f}, '
                f'vx={linear_cmd:.3f}, wz={angular_cmd:.3f}'
            )

    def handle_pause_after_dock(self):
        self.stop_robot()

        elapsed = self.now_sec_from(self.state_start_time)

        if elapsed < self.pause_after_dock_sec:
            return

        self.current_index += 1

        if self.current_index >= len(self.target_tag_ids):
            self.get_logger().info('All docking targets completed.')
            self.set_state('COMPLETE')
            return

        self.current_target_id = int(self.target_tag_ids[self.current_index])

        self.latest_x = None
        self.latest_z = None
        self.latest_tag_id = None
        self.last_seen_time = None

        self.get_logger().info(
            f'Move to next target tag_id={self.current_target_id}. '
            f'Start searching by turning {self.search_turn_direction}.'
        )

        # tag ที่สองเป็นต้นไป ให้หมุนหา
        self.set_state('SEARCH_TAG')


def main(args=None):
    rclpy.init(args=args)
    node = MultiAprilTagDockingController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            if rclpy.ok():
                node.stop_robot()
        except Exception as e:
            try:
                node.get_logger().warn(f'Failed to stop robot during shutdown: {e}')
            except Exception:
                pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()