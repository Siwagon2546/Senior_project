cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ติดตั้งโฟลเดอร์ launch และ config ไปยังโฟลเดอร์ที่ระบบหาเจอ
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  scripts/image_camera_info_qos_bridge.py
  scripts/Nav2_through_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()