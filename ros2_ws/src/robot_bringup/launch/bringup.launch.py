from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    dev_arg        = DeclareLaunchArgument('dev', default_value='/dev/agribot')
    baud_arg       = DeclareLaunchArgument('baud', default_value='115200')
    use_ekf_arg    = DeclareLaunchArgument('use_ekf', default_value='true')
    use_teleop_arg = DeclareLaunchArgument('use_teleop', default_value='false')

    dev        = LaunchConfiguration('dev')
    baud       = LaunchConfiguration('baud')
    use_ekf    = LaunchConfiguration('use_ekf')
    use_teleop = LaunchConfiguration('use_teleop')

    # micro-ROS agent (serial)
    agent = ExecuteProcess(
        cmd=['micro-ros-agent', 'serial', '--dev', dev, '-b', baud],
        output='screen')

    # Static TF: base_link -> imu_link (เริ่มที่ศูนย์ ถ้าเซนเซอร์ติดเอียงค่อยแก้ค่าที่นี่ทีหลัง)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # EKF (robot_localization) — รวม wheel/odom + imu/data (2D)
    pkg_share = get_package_share_directory('agribot_bringup')
    ekf_yaml  = os.path.join(pkg_share, 'config', 'ekf.yaml')
    ekf = Node(
        condition=IfCondition(use_ekf),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    # # Teleop (คีย์บอร์ด) — ออปชัน
    # teleop = Node(
    #     condition=IfCondition(use_teleop),
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop',
    #     output='screen',
    #     remappings=[('cmd_vel', 'cmd_vel')]
    # )

    ld = LaunchDescription()
    ld.add_action(dev_arg)
    ld.add_action(baud_arg)
    ld.add_action(use_ekf_arg)
    ld.add_action(use_teleop_arg)

    ld.add_action(agent)
    ld.add_action(static_tf)
    ld.add_action(ekf)
    # ld.add_action(teleop)
    return ld
