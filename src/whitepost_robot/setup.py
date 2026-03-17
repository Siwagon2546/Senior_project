import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'whitepost_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # บรรทัดนี้แหละที่ต้องการ os และ glob
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='whitepost',
    maintainer_email='whitepost@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # เช็คชื่อไฟล์ตรงนี้ให้ตรงกับที่คุณตั้งไว้นะครับ
            'esp32_drive_node = whitepost_robot.esp32_drive_node:main',
            'esp32_sensor_node = whitepost_robot.esp32_sensor_node:main',
        ],
    },
)