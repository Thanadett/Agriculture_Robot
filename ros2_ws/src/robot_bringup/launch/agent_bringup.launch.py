from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_dev', default_value='/dev/esp32_node1'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('log_level', default_value='6'),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            emulate_tty=True,
            arguments=[
                'serial',
                '--dev', LaunchConfiguration('serial_dev'),
                '-b', LaunchConfiguration('baud'),
                # '-v', LaunchConfiguration('log_level'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
