from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoint_file_arg = DeclareLaunchArgument(
        'waypoint_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_bringup'),
            'config',
            'sequential_waypoints.yaml'
        ])
    )

    return LaunchDescription([
        waypoint_file_arg,
        Node(
            package='robot_bringup',
            executable='sequential_nav_node.py',
            name='sequential_nav_node',
            output='screen',
            parameters=[LaunchConfiguration('waypoint_file')],
        )
    ])