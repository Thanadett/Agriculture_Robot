from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    transport = LaunchConfiguration('transport', default='serial')
    dev       = LaunchConfiguration('dev', default='/dev/esp32_node1')
    baud      = LaunchConfiguration('baud', default='115200')

    params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'params.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='serial'),
        DeclareLaunchArgument('dev', default_value='/dev/esp32_node1'),
        DeclareLaunchArgument('baud', default_value='115200'),

        # micro-ROS agent (ใช้ executable เดียว + subcommand)
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            arguments=['serial', '--dev', dev, '-b', baud],
        ),

        # base_controller (Python)
        Node(
            package='robot_bringup',
            executable='base_node_py',
            name='base_controller',
            output='screen',
            parameters=[params],
        ),
    ])
