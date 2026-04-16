from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    qos_relay = ExecuteProcess(
        cmd=[
            'python3',
            '/workspaces/isaac_ros-dev/src/robot_bringup/scripts/qos_relay.py'
        ],
        output='screen'
    )

    apriltag_node = ComposableNode(
        name='apriltag',
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        parameters=[{
            'size': 0.14,        # แก้เป็นขนาด tag จริง หน่วยเมตร
            'max_tags': 64,
            'tag_family': 'tag36h11',
            'backends': 'CUDA',
        }],
        remappings=[
            ('image', '/apriltag_bridge/image'),
            ('camera_info', '/apriltag_bridge/camera_info'),
            ('tag_detections', '/tag_detections'),
            ('tf', '/tf'),
        ],
    )

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[apriltag_node],
        output='screen',
    )

    return LaunchDescription([
        qos_relay,
        container,
    ])