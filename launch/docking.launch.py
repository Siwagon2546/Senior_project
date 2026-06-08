from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    apriltag_back_node = ComposableNode(
        name='apriltag_back',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.33,
            'max_tags': 64,
            'tag_family': 'tag36h11',
            # ถ้า CUDA/VPI ยัง stride error ให้เปลี่ยนไปใช้ apriltag_ros CPU
            'backends': 'CUDA',
        }],
        remappings=[
            ('image', '/camera_back/image_raw'),
            ('camera_info', '/camera_back/camera_info'),
            ('tag_detections', '/camera_back/tag_detections'),
            ('tf', '/tf'),
        ],
    )

    apriltag_container = ComposableNodeContainer(
        name='apriltag_back_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_back_node],
        output='screen',
    )

    docking_controller = Node(
        package='robot_bringup',
        executable='apriltag_docking_controller.py',
        name='apriltag_back_docking_controller',
        output='screen',
        parameters=[{
            'target_tag_ids': [19, 16],

            # ระยะ Dock แยกแต่ละ Tag ตามลำดับ target_tag_ids
            # Tag 19 หยุดที่ 0.40 m
            # Tag 16 หยุดที่ 0.25 m
            'target_distances': [2.00, 0.80],

            # fallback ถ้าจำนวน target_distances ไม่ตรงกับ target_tag_ids
            'target_distance': 2.00,

            'distance_tolerance': 0.15,
            'center_tolerance': 0.24,

            'kp_linear': 0.25,
            'kp_angular': 0.50,

            'max_linear_speed': 0.20,
            'max_angular_speed': 0.50,

            'lost_tag_timeout': 2.0,
            'tag_memory_timeout': 2.0,
            'use_last_pose_when_lost': True,
            'lost_linear_scale': 0.1,
            'lost_angular_scale': 0.1,

            'tag_topic': '/camera_back/tag_detections',
            'cmd_vel_topic': '/cmd_vel',

            'reverse_mode': True,
            'reverse_angular': False,

            'x_offset': 0.0,
            'debug_log': True,

            'pause_after_dock_sec': 1.0,

            # Tag แรกไม่หมุนหาอยู่แล้ว
            # หลัง Dock Tag แรกเสร็จ ถ้า True จะหมุนหา Tag ถัดไป
            'rotate_when_searching': True,
            'search_turn_direction': 'right',
            'search_angular_speed': 0.25,
            'search_timeout_sec': 30.0,

            # ถ้า tag หลุดระหว่าง docking เกิน memory timeout:
            # False = หยุดรอ ไม่หมุนมั่ว
            # True  = กลับไปหมุนหา
            'rotate_after_lost': False,
        }]
    )

    return LaunchDescription([
        apriltag_container,
        docking_controller,
    ])