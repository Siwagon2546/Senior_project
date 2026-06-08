cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)

# Python nodes
find_package(rclpy REQUIRED)

# C++ composable node
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# =========================
# C++ Realtime Depth Filter Component
# =========================
add_library(realtime_depth_filter_node SHARED
  src/realtime_depth_filter_node.cpp
)

ament_target_dependencies(realtime_depth_filter_node
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_nodes(
  realtime_depth_filter_node
  "robot_bringup::RealtimeDepthFilterNode"
)

install(TARGETS
  realtime_depth_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================
# Install launch/config
# =========================
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# =========================
# Install Python scripts
# =========================
install(PROGRAMS
  scripts/image_camera_info_qos_bridge.py
  scripts/Nav2_through_pose.py
  scripts/sequential_nav_node.py
  scripts/apriltag_docking_controller.py
  scripts/apriltag_docking_controller.py
  scripts/compressed_to_raw_camera_back.py
  scripts/camera_back_tcp_publisher.py
  scripts/back_camera_qos_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()