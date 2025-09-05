from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from robot_bringup.camera_nodes import get_camera_nodes


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

        # Camera HUD
        Node(
            package='robot_bringup',
            executable='camera_hud',
            name='camera_hud',
            output='screen',
            parameters=[{
                'image_topic': '/image_raw/compressed',
                'joy_topic':   '/joy',
                'cmd_topic':   '/cmd_vel',
                'show_fps': True,
                'show_latency': True,
                'show_center': True,
                'show_joy': True,
                'show_speed': True,
                'font_scale': 0.6,
                'thickness': 2,
                'alpha': 0.25,
                'hud_color_bgr': [0, 255, 255],
                'center_color_bgr': [0, 255, 0],
                'joy_color_bgr': [255, 0, 0],
                'speed_color_bgr': [0, 0, 255],
                'center_size_px': 25,
                'joy_scale': 0.9,
                'speed_bar_len': 150,
                'metrics_period_s': 1.0,
                'latency_cap_ms': 500,
            }],
        ),

        # Foxglove Bridge
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        ),
    ]

    # เพิ่ม nodes ของกล้อง
    nodes += get_camera_nodes(
        video_device=video_device,
        width=width,
        height=height,
        fps_num=fps_num,
        fps_den=fps_den,
        enable_compressed=True,
        qos_best_effort=True,
        add_foxglove=False
    )

    # รวม launch arguments และ nodes
    return LaunchDescription(declare_args + nodes)
