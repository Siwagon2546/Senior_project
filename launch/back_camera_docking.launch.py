from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    back_camera_qos_relay = Node(
        package='robot_bringup',
        executable='back_camera_qos_relay.py',
        name='back_camera_qos_relay',
        output='screen',
        parameters=[{
            'input_image_topic': '/camera_back/image_raw',
            'input_camera_info_topic': '/camera_back/camera_info',
            'output_image_topic': '/apriltag_back_bridge/image',
            'output_camera_info_topic': '/apriltag_back_bridge/camera_info',
        }]
    )

    apriltag_back_node = ComposableNode(
        name='apriltag_back',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.14,
            'max_tags': 64,
            'tag_family': 'tag36h11',
            'backends': 'CPU',
            # เริ่มจาก VPI/CPU ก่อน ถ้า CUDA ยัง pitch error
            # ถ้าอยากลอง CUDA ค่อยเปลี่ยนกลับเป็น 'CUDA'
            # 'backends': 'CUDA',
        }],
        remappings=[
            ('image', '/apriltag_back_bridge/image'),
            ('camera_info', '/apriltag_back_bridge/camera_info'),
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
            'target_tag_id': 0,
            'target_distance': 0.45,
            'distance_tolerance': 0.05,
            'center_tolerance': 0.04,

            'kp_linear': 0.20,
            'kp_angular': 0.7,

            'max_linear_speed': 0.05,
            'max_angular_speed': 0.20,

            'lost_tag_timeout': 0.5,
            'tag_topic': '/camera_back/tag_detections',
            'reverse_mode': True,
        }]
    )

    return LaunchDescription([
        back_camera_qos_relay,
        apriltag_container,
        docking_controller,
    ])