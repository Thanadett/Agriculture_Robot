from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # ---- Args ----
    dev_arg = DeclareLaunchArgument(
        'agent_transport', default_value='serial',
        description='micro-ROS agent transport: serial or udp'
    )
    serial_dev_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial device for micro-ROS agent'
    )
    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='8888',
        description='UDP port for micro-ROS agent'
    )
    ekf_cfg_arg = DeclareLaunchArgument(
        'ekf_config', default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'ekf.yaml'
        ),
        description='Path to EKF YAML file'
    )
    run_monitor_arg = DeclareLaunchArgument(
        'run_monitor', default_value='true',
        description='Run odom/imu monitor node'
    )

    agent_transport = LaunchConfiguration('agent_transport')
    serial_port = LaunchConfiguration('serial_port')
    udp_port = LaunchConfiguration('udp_port')
    ekf_config = LaunchConfiguration('ekf_config')
    run_monitor = LaunchConfiguration('run_monitor')

    # ---- micro-ROS agent (เลือก serial/udp) ----
    agent_cmd = [
        'micro-ros-agent',
        agent_transport,
    ]
    # เงื่อนไข transport
    # serial: micro-ros-agent serial --dev /dev/ttyUSB0 -v6
    # udp:    micro-ros-agent udp4 --port 8888 -v6
    agent_cmd += ['--dev', serial_port, '-v6']
    agent_serial = ExecuteProcess(
        cmd=['micro-ros-agent', 'serial', '--dev', serial_port, '-v6'],
        output='screen',
        shell=False
    )
    agent_udp = ExecuteProcess(
        cmd=['micro-ros-agent', 'udp4', '--port', udp_port, '-v6'],
        output='screen',
        shell=False
    )

    # จะสตาร์ทอย่างใดอย่างหนึ่ง: ใช้ GroupAction/opaque ถ้าต้องการเงื่อนไขซับซ้อน
    # ที่ง่ายสุด: ให้ผู้ใช้เลือก launch ไฟล์อีกอัน หรือ override ด้วย micro_ros_agent.launch.py
    # ที่นี่เราสตาร์ทแบบ serial เป็นดีฟอลต์ และให้ไฟล์อีกอันสำหรับ UDP แยกต่างหาก

    # ---- EKF (robot_localization) ----
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[ekf_config]
    )

    # ---- Monitor node (Python) ----
    monitor = Node(
        package='robot_bringup',
        executable='odom_imu_monitor',
        name='odom_imu_monitor',
        output='screen',
        parameters=[{
            'odom_topic': '/wheel/odom',
            'imu_topic': '/imu/data',
            'warn_rate_odom': 30.0,
            'warn_rate_imu': 80.0
        }]
    )

    # ดีฟอลต์: รัน agent serial + EKF + monitor
    ld = LaunchDescription()
    ld.add_action(dev_arg)
    ld.add_action(serial_dev_arg)
    ld.add_action(udp_port_arg)
    ld.add_action(ekf_cfg_arg)
    ld.add_action(run_monitor_arg)

    ld.add_action(agent_serial)
    ld.add_action(ekf)
    ld.add_action(monitor)

    return ld
