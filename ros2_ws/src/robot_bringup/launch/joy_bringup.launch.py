# launch/laptop_joy.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch args (ปรับได้ตอนเรียก launch)
    joy_topic = LaunchConfiguration('joy_topic')
    deadzone = LaunchConfiguration('deadzone')
    max_linear = LaunchConfiguration('max_linear')          # m/s
    max_angular = LaunchConfiguration('max_angular')         # rad/s
    ramp_rate_linear = LaunchConfiguration('ramp_rate_linear')    # m/s^2
    ramp_rate_angular = LaunchConfiguration('ramp_rate_angular')   # rad/s^2
    invert_left_y = LaunchConfiguration('invert_left_y')
    invert_right_x = LaunchConfiguration('invert_right_x')
    expo_linear = LaunchConfiguration('expo_linear')         # 0..1
    expo_angular = LaunchConfiguration('expo_angular')        # 0..1

    return LaunchDescription([
        # ---------- Launch arguments ----------
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('deadzone', default_value='0.12'),

        # ใช้หน่วยจริง (เริ่มให้ปลอดภัยก่อน ค่อยปรับเพิ่มได้)
        DeclareLaunchArgument('max_linear', default_value='1.0'),    # m/s
        DeclareLaunchArgument('max_angular', default_value='1.50'),   # rad/s
        DeclareLaunchArgument('ramp_rate_linear',
                              default_value='1.5'),   # m/s^2
        DeclareLaunchArgument('ramp_rate_angular',
                              default_value='6.00'),  # rad/s^2

        DeclareLaunchArgument('invert_left_y', default_value='1.0'),
        DeclareLaunchArgument('invert_right_x', default_value='1.0'),

        # โค้งคันเร่ง/พวงมาลัย (0=เส้นตรง, 1=โค้งจัด)
        DeclareLaunchArgument('expo_linear', default_value='0.40'),
        DeclareLaunchArgument('expo_angular', default_value='0.60'),

        # ---------- Joy driver ----------
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'autorepeat_rate': 50.0,
                'deadzone': deadzone
            }],
        ),

        # ---------- Servo Buttons ----------
        Node(
            package='robot_bringup',
            executable='servo_joy',
            name='servo_joy',
            output='screen',
            parameters=[{
                'joy_topic': joy_topic,
                'servo_cmd_topic': '/servo_cmd',
                'btn_a': 0,
                'btn_b': 1,
                'btn_x': 2,
                'btn_y': 3,
                'debounce_ms': 20
            }],
        ),

        # ---------- Stepper Buttons ----------
        Node(
            package='robot_bringup',
            executable='step_joy',
            name='step_joy',
            output='screen',
            parameters=[{
                'joy_topic': joy_topic,
                'stp_cmd_topic': '/step_cmd',
                'axis_index': 7,      # Xbox D-pad vertical = 7
                'threshold': 0.5,
                'debounce_ms': 20
            }],
        ),

        # ---------- Teleop (ส่ง /cmd_vel เป็นหน่วยจริง) ----------
        Node(
            package='robot_bringup',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen',
            parameters=[{
                'joy_topic': joy_topic,
                'max_linear': max_linear,                  # m/s
                'max_angular': max_angular,                # rad/s
                'deadzone': deadzone,
                'expo_linear': expo_linear,                # 0..1
                'expo_angular': expo_angular,              # 0..1
                'ramp_rate_linear': ramp_rate_linear,      # m/s^2
                'ramp_rate_angular': ramp_rate_angular,    # rad/s^2
                'joy_timeout_ms': 800,
                'btn_turbo': 5,               # RB
                'btn_emergency_stop': 4,      # LB
                'axis_left_y': 1,
                'axis_right_x': 3,
                'invert_left_y': invert_left_y,
                'invert_right_x': invert_right_x
            }],
        ),
    ])
