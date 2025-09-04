from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    joy_topic = LaunchConfiguration('joy_topic')
    max_linear = LaunchConfiguration('max_linear')
    max_angular = LaunchConfiguration('max_angular')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('deadzone', default_value='0.12'),

        # พรีเซ็ตตามฮาร์ดแวร์ของคุณ (ล้อ 5", 37 rpm, พื้นดิน)
        DeclareLaunchArgument('max_linear', default_value='1.0'),  # m/s
        DeclareLaunchArgument('max_angular', default_value='3.0'),  # rad/s

        # Joy driver + autorepeat กันคำสั่งวูบ
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


        # Joystick teleop (two-stick; RB linear instant; B e-stop)
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
                'expo_linear': 0.30,
                'expo_angular': 0.35,
                'ramp_rate': 3.0,
                'joy_timeout_ms': 2000,
                'btn_turbo': 5,
                'btn_emergency_stop': 1,
                'axis_left_y': 1,
                'axis_right_x': 3
            }],
        ),

        # Serial bridge to ESP32 (cmd_vel -> serial "VW ...")
        Node(
            package='robot_bringup',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{'port': port, 'baud': baud}],
        ),
    ])
