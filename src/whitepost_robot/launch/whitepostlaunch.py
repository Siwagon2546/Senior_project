import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # เปลี่ยน 'your_package_name' เป็นชื่อ Package ของคุณนะครับ
    package_name = 'whitepost_robot' 

    drive_node = Node(
        package=package_name,
        executable='esp32_drive_node', # ชื่อไฟล์ executable หรือ entry_point
        name='esp32_drive_node',
        output='screen'
    )

    sensor_node = Node(
        package=package_name,
        executable='esp32_sensor_node', # ชื่อไฟล์ executable หรือ entry_point
        name='esp32_sensor_node',
        output='screen'
    )

    return LaunchDescription([
        drive_node,
        sensor_node
    ])