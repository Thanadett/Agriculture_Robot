# launch/pi_bringup.launch.py
import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Serial to ESP32
        DeclareLaunchArgument('port_serial', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('max_linear', default_value='0.25'),
        DeclareLaunchArgument('max_angular', default_value='1.5'),

        Node(
            package='robot_bringup', executable='serial_bridge', name='serial_bridge', output='screen',
            parameters=[{
                'port': LaunchConfiguration('port_serial'),
                'baud': LaunchConfiguration('baud'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
            }],
        ),

        # Camera Stream (Flask)
        DeclareLaunchArgument('video_device', default_value='0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='30'),
        DeclareLaunchArgument('web_port', default_value='5000'),

        Node(
            package='robot_bringup',
            executable=sys.executable,
            arguments=[
                '-m', 'robot_bringup.camera_stream',
                '--device', LaunchConfiguration('video_device'),
                '--width', LaunchConfiguration('width'),
                '--height', LaunchConfiguration('height'),
                '--fps', LaunchConfiguration('fps')
            ],
            name='camera_stream',
            output='screen',
            emulate_tty=True,
        )


    ])