from setuptools import setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={
        package_name: [
            'templates/*.html',       
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', ['config/ekf.yaml',]),
        ('lib/' + package_name, ['robot_bringup/run_camera.sh']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'flask',
        'opencv-python',
        'numpy',
        'pyturbojpeg',
    ],
    zip_safe=True,
    maintainer='Thanadet Thanapremin',
    maintainer_email='n.thanadett@gmail.com',
    description='391 Project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'joystick_teleop = robot_bringup.joystick_teleop:main',
            'serial_bridge = robot_bringup.serial_bridge:main',
            'camera_stream = robot_bringup.camera_stream:main',
            'encode_bridge = robot_bringup.encode_bridge:main',
            'servo_joy = robot_bringup.servo_joy:main',
            'node2_bridge = robot_bringup.node2_bridge:main',
            'odom_imu_monitor = robot_bringup.odom_imu_monitor:main',
        ],
    },
    python_requires='>=3.12',
)
