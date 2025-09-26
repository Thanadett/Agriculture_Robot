# launch/pi_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ----- Serial Bridge args -----
    port_serial   = LaunchConfiguration('port_serial')
    baud          = LaunchConfiguration('baud')
    max_linear    = LaunchConfiguration('max_linear')
    max_angular   = LaunchConfiguration('max_angular')

    # ----- Camera args -----
    video_device1 = LaunchConfiguration('video_device1')
    video_device2 = LaunchConfiguration('video_device2')
    width1         = LaunchConfiguration('width1')
    height1        = LaunchConfiguration('height1')
    width2         = LaunchConfiguration('width2')
    height2        = LaunchConfiguration('height2')
    fps           = LaunchConfiguration('fps')
    quality       = LaunchConfiguration('quality')
    flip          = LaunchConfiguration('flip')
    rotate        = LaunchConfiguration('rotate')
    show_fps      = LaunchConfiguration('show_fps')

    return LaunchDescription([

        # ================= Base Controller =================
        Node(
            package='robot_bringup',
            executable='base_controller',
            name='base_controller',
            output='screen',
            parameters=[{
                'ticks_topic': 'wheel_ticks',
                'odom_topic': 'odom',
                'wheel_radius_m': 0.0635,
                'track_width_m': 0.365,
                'ppr_out': 5940.0,
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'publish_tf': True
            }],
        ),

        # ---------- Serial Bridge for esp32 node2 ----------
        Node(
            package='robot_bringup',
            executable='node2_bridge',
            name='node2_bridge',
            output='screen',
            parameters=[{
                'verbose': False,
            }],
        ),

        # ================= Camera Stream (Flask) =================
        # Camera devices
        DeclareLaunchArgument('video_device1', default_value='2'),
        DeclareLaunchArgument('video_device2', default_value='0'),
        DeclareLaunchArgument('width1',   default_value='2592'),
        DeclareLaunchArgument('height1',  default_value='1944'),
        DeclareLaunchArgument('width2',   default_value='800'),
        DeclareLaunchArgument('height2',  default_value='480'),
        DeclareLaunchArgument('fps',     default_value='30'),
        DeclareLaunchArgument('quality', default_value='75'),
        DeclareLaunchArgument('flip',     default_value='0', description='Flip horizontally (0/1)'),
        DeclareLaunchArgument('rotate',   default_value='0', description='Rotate (0/90/180/270)'),
        DeclareLaunchArgument('show_fps', default_value='0', description='Show FPS overlay (0/1)'),

        Node(
            package='robot_bringup',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            arguments=[
                '--device1',  video_device1,
                '--device2',  video_device2,
                '--width1',    width1,
                '--height1',   height1,
                '--width2',    width2,
                '--height2',   height2,
                '--fps',      fps,
                '--quality',  quality,
                '--flip',     flip,
                '--rotate',   rotate,
                '--show-fps', show_fps,
            ],
        ),
    ])



        # # ================= Serial to ESP32 =================
        # DeclareLaunchArgument('port_serial', default_value='/dev/esp32_node1'),
        # DeclareLaunchArgument('baud', default_value='115200'),
        # DeclareLaunchArgument('max_linear', default_value='255.0'),
        # DeclareLaunchArgument('max_angular', default_value='255.0'),

        # Node(
        #     package='robot_bringup',
        #     executable='serial_bridge',
        #     name='serial_bridge',
        #     output='screen',
        #     parameters=[{
        #         'port':        port_serial,
        #         'baud':        baud,
        #         'max_linear':  max_linear,
        #         'max_angular': max_angular,
        #     }],
        # ),

        # Node(
        #     package='robot_bringup',
        #     executable='encode_bridge',
        #     name='encode_bridge',
        #     output='screen',
        #     parameters=[{
        #         'print_hz':    2.0,
        #         'decimals':    4,
        #         'deadband':    1e-4,
        #         'wheel_radius':0.0635,  # m
        #         'units_pos':   'rad',   # 'rad' or 'deg'
        #         'units_dist':  'm',     # 'm' or 'mm'
        #         'enable_log':  False,   # << สวิตช์ เปิด/ปิด log
        #     }],
        # ),