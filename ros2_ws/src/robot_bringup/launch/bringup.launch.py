from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('deadzone', default_value='0.12'),
        DeclareLaunchArgument('max_linear', default_value='255.0'),
        DeclareLaunchArgument('max_angular', default_value='255.0'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps_num', default_value='1'),
        DeclareLaunchArgument('fps_den', default_value='30'),
    ]

    # LaunchConfigurations
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    joy_topic = LaunchConfiguration('joy_topic')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')
    video_device = LaunchConfiguration('video_device')
    width = LaunchConfiguration('width')
    height = LaunchConfiguration('height')
    fps_num = LaunchConfiguration('fps_num')
    fps_den = LaunchConfiguration('fps_den')

    # Nodes
    nodes = [
        # Joy driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': 50.0,
                'deadzone': LaunchConfiguration('deadzone')
            }],
        ),

        # Teleop
        Node(
            package='robot_bringup',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen',
            parameters=[{
                'joy_topic': joy_topic,
                'max_linear': max_linear,
                'max_angular': max_angular,
                'deadzone': 0.12,
                'expo_linear': 0.50,
                'expo_angular': 0.55,
                'ramp_rate_linear': 255.0,
                'ramp_rate_angular': 255.0,
                'joy_timeout_ms': 2000,
                'btn_turbo': 1,
                'btn_emergency_stop': 0,
                'axis_left_y': 1,
                'axis_right_x': 3
            }],
        ),

        # Serial bridge
        # Node(
        #     package='robot_bringup',
        #     executable='serial_bridge',
        #     name='serial_bridge',
        #     output='screen',
        #     parameters=[{'port': port, 'baud': baud}],
        # ),
    ]

    # รวม launch arguments และ nodes
    return LaunchDescription(declare_args + nodes)
