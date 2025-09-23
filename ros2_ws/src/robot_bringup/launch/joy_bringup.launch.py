# launch/laptop_joy.launch.py
import sys
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('joy_topic', default_value='/joy'),
        DeclareLaunchArgument('deadzone', default_value='0.12'),
        DeclareLaunchArgument('max_linear', default_value='255.0'),   
        DeclareLaunchArgument('max_angular', default_value='255.0'),  

        # Joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 50.0, 'deadzone': LaunchConfiguration('deadzone')}],
        ),

        # ---------- Servo Buttons (publish /servo_cmd) ----------
        Node(
            package='robot_bringup',      
            executable='servo_joy',
            name='servo_joy',
            output='screen',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'servo_cmd_topic': '/servo_cmd',
                'btn_a': 0,
                'btn_b': 1,
                'btn_x': 2,
                'btn_y': 3,
                'debounce_ms': 20
            }],
        ),
        # ---------- Stepper Buttons (publish /step_cmd) ----------
        Node(
            package='robot_bringup',
            executable='step_joy',
            name='step_joy',        
            output='screen',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'stp_cmd_topic': '/step_cmd',
                'axis_index': 7,      # Xbox D-pad vertical = 7
                'threshold': 0.5,     # deadzone threshold
                'debounce_ms': 20
            }],
        ),
        # Teleop node
        Node(
            package='robot_bringup',
            executable='joystick_teleop',
            name='joystick_teleop',
            output='screen',
            parameters=[{
                'joy_topic': LaunchConfiguration('joy_topic'),
                'max_linear': LaunchConfiguration('max_linear'),
                'max_angular': LaunchConfiguration('max_angular'),
                'deadzone': LaunchConfiguration('deadzone'),
                'expo_linear': 3.0,
                'expo_angular': 2.5,
                'ramp_rate_linear': 250.0,
                'ramp_rate_angular': 250.0,
                'joy_timeout_ms': 800,
                'btn_turbo': 5,
                'btn_emergency_stop': 4,
                'axis_left_y': 1,
                'axis_right_x': 3
            }],
        ),

    ])
