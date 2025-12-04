from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
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
        ('share/' + package_name + '/config', ['config/params.yaml',]),
        ('share/' + package_name + '/scripts',
         ['robot_bringup/run_camera.sh']),
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
    license='Ha HA Ha',
    entry_points={
        'console_scripts': [
            'joystick_teleop = robot_bringup.joystick_teleop:main',
            'serial_bridge = robot_bringup.serial_bridge:main',
            'camera_stream = robot_bringup.camera_stream:main',
            'servo_joy = robot_bringup.servo_joy:main',
            'step_joy = robot_bringup.step_joy:main',
            'node2_bridge = robot_bringup.node2_bridge:main',
            'base_controller = robot_bringup.base_controller:main',
            'encode_bridge = robot_bringup.encode_bridge:main',
            'wheel_ticks_from_cmdvel= robot_bringup.wheel_ticks_from_cmdvel:main',
            'pid_check = robot_bringup.pid_check:main',
            'camera_publisher = robot_bringup.robot_vision.camera_publisher:main',
            'april_tag_reader = robot_bringup.robot_vision.april_tag_reader:main',
        ],
    },
    python_requires='>=3.10,<3.13',
)
