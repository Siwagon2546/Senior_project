from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': 'camera0',
            'camera_namespace': '',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'true',
        }.items()
    )

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
            'size': 0.14,
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

    docking_controller = ExecuteProcess(
    cmd=[
        'python3',
        '/workspaces/isaac_ros-dev/src/robot_bringup/scripts/apriltag_docking_controller2.py'
    ],
    output='screen'
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
        realsense_launch,
        qos_relay,
        container,
        docking_controller,
    ])