import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    default_nav2_params = os.path.join(
        pkg_robot_bringup,
        'config',
        'nav2_carter_style_fix.yaml'
    )

    nav2_params_path = LaunchConfiguration('nav2_params_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    delay_nav2 = LaunchConfiguration('delay_nav2')
    publish_map_to_odom = LaunchConfiguration('publish_map_to_odom')

    declare_args = [
        DeclareLaunchArgument(
            'nav2_params_path',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameter file.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation clock if true.'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='True',
            description='Automatically startup the Nav2 stack.'
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Use composed Nav2 bringup if true.'
        ),
        DeclareLaunchArgument(
            'delay_nav2',
            default_value='0.0',
            description='Delay in seconds before starting Nav2.'
        ),
        DeclareLaunchArgument(
            'publish_map_to_odom',
            default_value='False',
            description='Publish a static identity transform from map to odom.'
        ),
    ]

    tf_camera_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera0_to_base_link_tf',
        arguments=[
            '--x', '-0.305',
            '--y', '0.0',
            '--z', '-0.46',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'camera0_link',
            '--child-frame-id', 'base_link',
        ],
        output='screen',
    )


    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': autostart,
            'use_composition': use_composition,
        }.items(),
    )

    nav2_actions = [
        LogInfo(msg='[Launch] Starting Nav2 stack'),
        nav2_stack,
    ]

    nav2_start = TimerAction(
        period=delay_nav2,
        actions=nav2_actions,
    )

    return LaunchDescription(
        declare_args + [
            LogInfo(msg='[Launch] Publishing static TF: camera0_link -> base_link'),
            LogInfo(msg='[Launch] Optional static TF: map -> odom (disabled by default)'),
            tf_camera_to_base,
            nav2_start,
        ]
    )
