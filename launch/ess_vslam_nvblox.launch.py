from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    container_name = LaunchConfiguration('container_name')
    engine_file_path = LaunchConfiguration('engine_file_path')
    threshold = LaunchConfiguration('threshold')
    run_realsense = LaunchConfiguration('run_realsense')
    num_cameras = LaunchConfiguration('num_cameras')
    camera_serial_numbers = LaunchConfiguration('camera_serial_numbers')

    declared_arguments = [
        DeclareLaunchArgument(
            'container_name',
            default_value='perception_container'
        ),
        DeclareLaunchArgument(
            'engine_file_path',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('ISAAC_ROS_WS'),
                'isaac_ros_assets',
                'models',
                'dnn_stereo_disparity',
                'dnn_stereo_disparity_v4.1.0_onnx',
                'ess.engine'
            ])
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='0.0'
        ),
        DeclareLaunchArgument(
            'run_realsense',
            default_value='True'
        ),
        DeclareLaunchArgument(
            'num_cameras',
            default_value='1'
        ),
        DeclareLaunchArgument(
            'camera_serial_numbers',
            default_value=''
        ),
    ]

    # 1) Launch Realsense driver first
    # ใช้ launch เดิมของ nvblox_examples_bringup เพื่อให้ topic names ตรงกับระบบมึง
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nvblox_examples_bringup'),
                'launch',
                'sensors',
                'realsense.launch.py'
            ])
        ),
        launch_arguments={
            'container_name': container_name,
            'camera_serial_numbers': camera_serial_numbers,
            'num_cameras': num_cameras,
        }.items(),
        condition=UnlessCondition(PythonExpression(["'", run_realsense, "' == 'False'"]))
    )

    image_format_left = ComposableNode(
        name='image_format_left',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/camera0/infra1/image_rect_raw'),
            ('image', '/ess_rgb/left/image'),
        ],
    )

    image_format_right = ComposableNode(
        name='image_format_right',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        parameters=[{
            'encoding_desired': 'rgb8'
        }],
        remappings=[
            ('image_raw', '/camera0/infra2/image_rect_raw'),
            ('image', '/ess_rgb/right/image'),
        ],
    )

    # 2) ESS pipeline
    left_resize_node = ComposableNode(
    name='left_resize',
    package='isaac_ros_image_proc',
    plugin='nvidia::isaac_ros::image_proc::ResizeNode',
    parameters=[{
        'output_width': 960,
        'output_height': 576,
    }],
    remappings=[
        ('image', '/ess_rgb/left/image'),
        ('camera_info', '/camera0/infra1/camera_info'),
        ('resize/image', '/ess/left/image_rect'),
        ('resize/camera_info', '/ess/left/camera_info'),
    ],
)

    right_resize_node = ComposableNode(
        name='right_resize',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'output_width': 960,
            'output_height': 576,
        }],
        remappings=[
            ('image', '/ess_rgb/right/image'),
            ('camera_info', '/camera0/infra2/camera_info'),
            ('resize/image', '/ess/right/image_rect'),
            ('resize/camera_info', '/ess/right/camera_info'),
        ],
    )

    ess_disparity_node = ComposableNode(
        name='ess_disparity',
        package='isaac_ros_ess',
        plugin='nvidia::isaac_ros::dnn_stereo_depth::ESSDisparityNode',
        parameters=[{
            'engine_file_path': engine_file_path,
            'threshold': threshold,
        }],
        remappings=[
            ('left/image_rect', '/ess/left/image_rect'),
            ('right/image_rect', '/ess/right/image_rect'),
            ('left/camera_info', '/ess/left/camera_info'),
            ('right/camera_info', '/ess/right/camera_info'),
            ('disparity', '/ess/disparity'),
        ],
    )

    disparity_to_depth_node = ComposableNode(
        name='disparity_to_depth',
        package='isaac_ros_stereo_image_proc',
        plugin='nvidia::isaac_ros::stereo_image_proc::DisparityToDepthNode',
        remappings=[
            ('disparity', '/ess/disparity'),
            ('depth', '/ess/depth/image'),
        ],
    )

    # 3) VSLAM (no splitter)
    vslam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,
            'enable_localization_n_mapping': True,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'rig_frame': 'base_link',
            'publish_odom_to_base_tf': True,
            'publish_map_to_odom_tf': True,
            'imu_frame': 'camera0_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'path_max_size': 200,
            'verbosity': 5,
            'enable_debug_mode': False,
            'debug_dump_path': '/tmp/cuvslam',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'camera0_link',
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam': True,
            'localizer_horizontal_radius': 2.0,
            'localizer_vertical_radius': 0.3,
            'localizer_horizontal_step': 0.2,
            'localizer_vertical_step': 0.25,
            'localizer_angular_step': 0.09,
            'slam_max_map_size': 500,
            'enable_request_hint': True,
            'enable_rectified_pose': True,
            'enable_image_denoising': False,
            'rectified_images': True,
            # 'load_map_folder_path': '/workspaces/isaac_ros-dev/maps/site_a_live/vslam_map6',#
            # 'localize_on_startup': True,
            'camera_optical_frames': [
                'camera0_infra1_optical_frame',
                'camera0_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/camera_info_0', '/camera0/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/camera0/infra2/camera_info'),
            ('visual_slam/image_0', '/camera0/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/camera0/infra2/image_rect_raw'),
            ('visual_slam/imu', '/camera0/imu'),
        ],
    )

    # 4) Nvblox (ESS depth -> Nvblox)
    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('nvblox_examples_bringup'),
                'config', 'nvblox', 'nvblox_base.yaml'
            ]),
            PathJoinSubstitution([
                FindPackageShare('nvblox_examples_bringup'),
                'config', 'nvblox', 'specializations', 'nvblox_realsense.yaml'
            ]),
            {'num_cameras': 1},
            {'use_lidar': False},
        ],
        remappings=[
            ('camera_0/depth/image', '/ess/depth/image'),
            ('camera_0/depth/camera_info', '/ess/left/camera_info'),
            ('camera_0/color/image', '/camera0/color/image_raw'),
            ('camera_0/color/camera_info', '/camera0/color/camera_info'),
        ],
    )

    # 5) Shared component container
    # ESS + VSLAM + Nvblox อยู่ container เดียวกัน
    perception_container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            image_format_left,
            image_format_right,
            left_resize_node,
            right_resize_node,
            ess_disparity_node,
            disparity_to_depth_node,
            vslam_node,
            nvblox_node,
        ],
        output='screen',
    )

    return LaunchDescription([
        *declared_arguments,
        realsense_launch,
        perception_container,
    ])