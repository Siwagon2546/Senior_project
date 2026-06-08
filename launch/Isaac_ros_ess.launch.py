from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('isaac_ros_examples'),
                    'launch',
                    'isaac_ros_examples.launch.py'
                ])
            ),
            launch_arguments={
                'launch_fragments': 'realsense_stereo_rect,ess_disparity',
                'engine_file_path': PathJoinSubstitution([
                    EnvironmentVariable('ISAAC_ROS_WS'),
                    'isaac_ros_assets',
                    'models',
                    'dnn_stereo_disparity',
                    'dnn_stereo_disparity_v4.1.0_onnx',
                    'ess.engine'
                ]),
                'threshold': '0.0',
                'realsense_config_file': PathJoinSubstitution([
                    FindPackageShare('robot_bringup'),
                    'config',
                    'realsense.yaml'
                ]),
            }.items()
        )
    ])