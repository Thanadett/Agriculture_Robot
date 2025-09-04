from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    joy_topic = LaunchConfiguration('joy_topic')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),

        # Joy driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Our joystick teleop (joy -> /cmd_vel)
        Node(
            package='robot_bringup',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen',
            parameters=[{'joy_topic': joy_topic}],
        ),

        # Serial bridge to ESP32 (cmd_vel -> serial "VW ...")
        Node(
            package='robot_bringup',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'port': port, 'baud': int(baud.perform(None))}],
        ),
    ])
