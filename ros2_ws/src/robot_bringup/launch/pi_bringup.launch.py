# launch/pi_bringup.launch.py
import sys
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # agent = Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros_agent_serial',
    #     output='screen',
    #     arguments=[
    #         'serial',
    #         '--dev', '/dev/esp_node2',   # เปลี่ยนพอร์ตตามเครื่องคุณ
    #         '--baudrate', '115200',      # ถ้าใช้ค่าอื่น ปรับตรงนี้
    #         '--ros-domain-id', '69',     # ตั้ง Domain ID
    #         '-v6'                        # ระดับ verbosity ของ agent (ปรับได้)
    #     ],
    #     # ถ้าอยากให้ agent ล้มแล้วลุกใหม่
    #     # respawn=True, respawn_delay=2.0,
    # )

    # # encode_bridge.py (เวอร์ชันเดิมของคุณ) — ไม่มี autostart agent แล้ว
    # enc_bridge = Node(
    #     package='robot_bringup',
    #     executable='encode_bridge',
    #     name='encode_bridge',
    #     output='screen',
    #     parameters=[{
    #         'print_hz': 2.0,
    #         'decimals': 4,
    #         'deadband': 1e-4,
    #         'wheel_radius': 0.0635,   # m
    #         'units_pos': 'rad',       # 'rad' or 'deg'
    #         'units_dist': 'm',        # 'm' or 'mm'
    #         'enable_log': False,
    #     }],
    # )

    # #ให้ agent ขึ้นก่อนแล้วค่อยสตาร์ท encode_bridge ทีหลังสัก 2 วิ
    # enc_bridge_after_agent = TimerAction(
    #     period=2.0,
    #     actions=[enc_bridge]
    # )

    # return LaunchDescription([
    #     agent,
    #     enc_bridge_after_agent
    # ])

    return LaunchDescription([
        # Serial to ESP32
        # DeclareLaunchArgument('port_serial', default_value='/dev/ttyUSB0'),
        # DeclareLaunchArgument('baud', default_value='115200'),
        # DeclareLaunchArgument('max_linear', default_value='255.0'),
        # DeclareLaunchArgument('max_angular', default_value='255.0'),

        # Node(
        #     package='robot_bringup', executable='serial_bridge', name='serial_bridge', output='screen',
        #     parameters=[{
        #         'port': LaunchConfiguration('port_serial'),
        #         'baud': LaunchConfiguration('baud'),
        #         'max_linear': LaunchConfiguration('max_linear'),
        #         'max_angular': LaunchConfiguration('max_angular'),
        #     }],
        # ),

        # Node(
        #     package='robot_bringup', executable='encode_bridge', name='encode_bridge', output='screen',
        #     parameters=[{
        #         'print_hz': 2.0,
        #         'decimals': 4,
        #         'deadband': 1e-4,
        #         'wheel_radius': 0.0635,   # m
        #         'units_pos': 'rad',       # 'rad' or 'deg'
        #         'units_dist': 'm',        # 'm' or 'mm'
        #         'enable_log': False,      # << สวิตช์ เปิด/ปิด log
        #     }],
        # ),

        # Camera Stream (Flask)
        DeclareLaunchArgument('video_device', default_value='0'),
        DeclareLaunchArgument('width', default_value='800'),
        DeclareLaunchArgument('height', default_value='600'),
        DeclareLaunchArgument('fps', default_value='30'),            
        DeclareLaunchArgument('web_port', default_value='5000'),

        ExecuteProcess(
            cmd=[
                'python3', 
                'install/robot_bringup/lib/robot_bringup/camera_stream',
                '--device', LaunchConfiguration('video_device'),
                '--width', LaunchConfiguration('width'),
                '--height', LaunchConfiguration('height'),
                '--fps', LaunchConfiguration('fps'),
                '--port', LaunchConfiguration('web_port')
            ],
            output='screen'
        )
    ]        
)