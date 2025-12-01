# launch/pi_bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ----- Camera args -----
    video_device1 = LaunchConfiguration('video_device1')
    video_device2 = LaunchConfiguration('video_device2')
    width1 = LaunchConfiguration('width1')
    height1 = LaunchConfiguration('height1')
    width2 = LaunchConfiguration('width2')
    height2 = LaunchConfiguration('height2')
    fps = LaunchConfiguration('fps')
    quality = LaunchConfiguration('quality')
    flip = LaunchConfiguration('flip')
    rotate = LaunchConfiguration('rotate')
    show_fps = LaunchConfiguration('show_fps')

    # ----- Guide args -----
    show_guides = LaunchConfiguration('show_guides')

    # vertical modes/positions/range/style
    c1_mode = LaunchConfiguration('c1_mode')
    c2_mode = LaunchConfiguration('c2_mode')

    c1_left = LaunchConfiguration('c1_left')
    c1_right = LaunchConfiguration('c1_right')
    c2_left = LaunchConfiguration('c2_left')
    c2_right = LaunchConfiguration('c2_right')

    c1_angle = LaunchConfiguration('c1_angle')
    c2_angle = LaunchConfiguration('c2_angle')

    c1_y_top = LaunchConfiguration('c1_y_top')
    c1_y_bottom = LaunchConfiguration('c1_y_bottom')
    c2_y_top = LaunchConfiguration('c2_y_top')
    c2_y_bottom = LaunchConfiguration('c2_y_bottom')

    c1_style = LaunchConfiguration('c1_style')
    c2_style = LaunchConfiguration('c2_style')

    c1_color = LaunchConfiguration('c1_color')
    c2_color = LaunchConfiguration('c2_color')

    c1_thickness = LaunchConfiguration('c1_thickness')
    c2_thickness = LaunchConfiguration('c2_thickness')

    c1_alpha = LaunchConfiguration('c1_alpha')
    c2_alpha = LaunchConfiguration('c2_alpha')

    c1_dash = LaunchConfiguration('c1_dash')
    c1_gap = LaunchConfiguration('c1_gap')
    c2_dash = LaunchConfiguration('c2_dash')
    c2_gap = LaunchConfiguration('c2_gap')

    # horizontal lines (top/bottom + x-span + style override)
    c1_top = LaunchConfiguration('c1_top')
    c1_bottom = LaunchConfiguration('c1_bottom')
    c2_top = LaunchConfiguration('c2_top')
    c2_bottom = LaunchConfiguration('c2_bottom')

    c1_x_left = LaunchConfiguration('c1_x_left')
    c1_x_right = LaunchConfiguration('c1_x_right')
    c2_x_left = LaunchConfiguration('c2_x_left')
    c2_x_right = LaunchConfiguration('c2_x_right')

    c1_h_angle = LaunchConfiguration('c1_h_angle')
    c2_h_angle = LaunchConfiguration('c2_h_angle')

    c1_h_style = LaunchConfiguration('c1_h_style')
    c2_h_style = LaunchConfiguration('c2_h_style')

    c1_h_color = LaunchConfiguration('c1_h_color')
    c2_h_color = LaunchConfiguration('c2_h_color')

    c1_h_thickness = LaunchConfiguration('c1_h_thickness')
    c2_h_thickness = LaunchConfiguration('c2_h_thickness')

    c1_h_alpha = LaunchConfiguration('c1_h_alpha')
    c2_h_alpha = LaunchConfiguration('c2_h_alpha')

    c1_h_dash = LaunchConfiguration('c1_h_dash')
    c1_h_gap = LaunchConfiguration('c1_h_gap')
    c2_h_dash = LaunchConfiguration('c2_h_dash')
    c2_h_gap = LaunchConfiguration('c2_h_gap')

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
                'publish_tf': True,
                'yaw_mode': 'kf',       # 'enc' | 'kf' | 'blend'
                'wz_source': 'imu',     # 'enc' | 'imu'
                'blend_alpha': 0.2,
            }],
        ),

        # ---------- Serial Bridge for esp32 node2 ----------
        Node(
            package='robot_bringup',
            executable='node2_bridge',
            name='node2_bridge',
            output='screen',
            parameters=[{'verbose': False}],
        ),

        # ================= Camera Stream (Flask) =================
        # Camera devices
        DeclareLaunchArgument('video_device1', default_value='0'),
        DeclareLaunchArgument('video_device2', default_value='2'),
        DeclareLaunchArgument('width1',   default_value='648'),
        DeclareLaunchArgument('height1',  default_value='432'),
        DeclareLaunchArgument('width2',   default_value='432'),
        DeclareLaunchArgument('height2',  default_value='288'),
        DeclareLaunchArgument('fps',      default_value='20'),
        DeclareLaunchArgument('quality',  default_value='60'),
        DeclareLaunchArgument('flip',     default_value='0',
                              description='Flip horizontally (0/1)'),
        DeclareLaunchArgument('rotate',   default_value='180',
                              description='Rotate (0/90/180/270)'),
        DeclareLaunchArgument('show_fps', default_value='0',
                              description='Show FPS overlay (0/1)'),

        # Guides (vertical + horizontal)
        DeclareLaunchArgument('show_guides', default_value='1',
                              description='Show guide lines (0/1)'),

        DeclareLaunchArgument('c1_mode', default_value='pixel',
                              description='cam1 guide mode: percent|pixel'),
        DeclareLaunchArgument('c2_mode', default_value='pixel',
                              description='cam2 guide mode: percent|pixel'),

        DeclareLaunchArgument('c1_left',  default_value='-5.0'),
        DeclareLaunchArgument('c1_right', default_value='324'),
        DeclareLaunchArgument('c2_left',  default_value='30'),
        DeclareLaunchArgument('c2_right', default_value='275'),

        DeclareLaunchArgument('c1_angle', default_value='0.0',
                              description='cam1 vertical angle deg'),
        DeclareLaunchArgument('c2_angle', default_value='0.0',
                              description='cam2 vertical angle deg'),

        DeclareLaunchArgument('c1_y_top',    default_value='0.0'),
        DeclareLaunchArgument('c1_y_bottom', default_value='1.0'),
        DeclareLaunchArgument('c2_y_top',    default_value='0.0'),
        DeclareLaunchArgument('c2_y_bottom', default_value='1.0'),

        DeclareLaunchArgument('c1_style', default_value='solid',
                              description='solid|dashed|dotted'),
        DeclareLaunchArgument('c2_style', default_value='solid',
                              description='solid|dashed|dotted'),

        DeclareLaunchArgument(
            'c1_color', default_value='0,255,255', description='B,G,R'),
        DeclareLaunchArgument(
            'c2_color', default_value='255,255,0', description='B,G,R'),

        DeclareLaunchArgument('c1_thickness', default_value='3'),
        DeclareLaunchArgument('c2_thickness', default_value='3'),

        DeclareLaunchArgument('c1_alpha', default_value='0.4'),
        DeclareLaunchArgument('c2_alpha', default_value='0.6'),

        DeclareLaunchArgument('c1_dash', default_value='18'),
        DeclareLaunchArgument('c1_gap',  default_value='12'),
        DeclareLaunchArgument('c2_dash', default_value='18'),
        DeclareLaunchArgument('c2_gap',  default_value='12'),

        # ---- Horizontal guide lines ----
        DeclareLaunchArgument('c1_top',    default_value='-5.0',
                              description='cam1 horizontal top line (percent|pixel by mode)'),
        DeclareLaunchArgument('c1_bottom', default_value='304.0',
                              description='cam1 horizontal bottom line (percent|pixel by mode)'),
        DeclareLaunchArgument('c2_top',    default_value='68.0',
                              description='cam2 horizontal top line (percent|pixel by mode)'),
        DeclareLaunchArgument('c2_bottom', default_value='-50.0',
                              description='cam2 horizontal bottom line (percent|pixel by mode)'),

        # ช่วง X สำหรับเส้นแนวนอน (ใช้ตาม c1_mode/c2_mode: percent|pixel)
        DeclareLaunchArgument('c1_x_left',  default_value='20.0',
                              description='span left (percent|pixel by mode)'),
        DeclareLaunchArgument('c1_x_right', default_value='628.0',
                              description='span right (percent|pixel by mode)'),
        DeclareLaunchArgument('c2_x_left',  default_value='0.0',
                              description='span left (percent|pixel by mode)'),
        DeclareLaunchArgument('c2_x_right', default_value='350.0',
                              description='span right (percent|pixel by mode)'),

        # มุมเอียงของเส้นแนวนอน
        DeclareLaunchArgument('c1_h_angle', default_value='0.0',
                              description='cam1 horizontal angle deg'),
        DeclareLaunchArgument('c2_h_angle', default_value='7.0',
                              description='cam2 horizontal angle deg'),

        # สไตล์/สี/ความหนา/ความโปร่งใส/ขีด-ช่องว่าง ของเส้นแนวนอน
        # เดิมบางตัวเป็นค่าว่าง → ใส่ default ให้ครบเพื่อกันล่ม
        DeclareLaunchArgument('c1_h_style', default_value='solid',
                              description='solid|dashed|dotted (override)'),
        DeclareLaunchArgument('c2_h_style', default_value='solid',
                              description='solid|dashed|dotted (override)'),

        DeclareLaunchArgument('c1_h_color', default_value='0,255,255',
                              description='B,G,R (override)'),
        DeclareLaunchArgument('c2_h_color', default_value='255,255,0',
                              description='B,G,R (override)'),

        DeclareLaunchArgument('c1_h_thickness', default_value='5'),
        DeclareLaunchArgument('c2_h_thickness', default_value='5'),

        DeclareLaunchArgument('c1_h_alpha', default_value='0.7'),
        DeclareLaunchArgument('c2_h_alpha', default_value='0.7'),

        DeclareLaunchArgument('c1_h_dash', default_value='18'),
        DeclareLaunchArgument('c1_h_gap',  default_value='12'),
        DeclareLaunchArgument('c2_h_dash', default_value='18'),
        DeclareLaunchArgument('c2_h_gap',  default_value='12'),

        Node(
            package='robot_bringup',
            executable='camera_stream',
            name='camera_stream',
            output='screen',
            arguments=[
                '--device1',  video_device1,
                '--device2',  video_device2,
                '--width1',   width1,
                '--height1',  height1,
                '--width2',   width2,
                '--height2',  height2,
                '--fps',      fps,
                '--quality',  quality,
                '--flip',     flip,
                '--rotate',   rotate,
                '--show-fps', show_fps,

                # ---- pass guide args ----
                '--show-guides', show_guides,

                '--c1-mode', c1_mode,
                '--c2-mode', c2_mode,

                '--c1-left',  c1_left,
                '--c1-right', c1_right,
                '--c2-left',  c2_left,
                '--c2-right', c2_right,

                '--c1-angle', c1_angle,
                '--c2-angle', c2_angle,

                '--c1-y-top',    c1_y_top,
                '--c1-y-bottom', c1_y_bottom,
                '--c2-y-top',    c2_y_top,
                '--c2-y-bottom', c2_y_bottom,

                '--c1-style', c1_style,
                '--c2-style', c2_style,

                '--c1-color', c1_color,
                '--c2-color', c2_color,

                '--c1-thickness', c1_thickness,
                '--c2-thickness', c2_thickness,

                '--c1-alpha', c1_alpha,
                '--c2-alpha', c2_alpha,

                '--c1-dash', c1_dash,
                '--c1-gap',  c1_gap,
                '--c2-dash', c2_dash,
                '--c2-gap',  c2_gap,

                # ---- Horizontal ----
                '--c1-top',    c1_top,
                '--c1-bottom', c1_bottom,
                '--c2-top',    c2_top,
                '--c2-bottom', c2_bottom,

                '--c1-x-left',  c1_x_left,
                '--c1-x-right', c1_x_right,
                '--c2-x-left',  c2_x_left,
                '--c2-x-right', c2_x_right,

                '--c1-h-angle', c1_h_angle,
                '--c2-h-angle', c2_h_angle,

                '--c1-h-style', c1_h_style,
                '--c2-h-style', c2_h_style,
                '--c1-h-color', c1_h_color,
                '--c2-h-color', c2_h_color,
                '--c1-h-thickness', c1_h_thickness,
                '--c2-h-thickness', c2_h_thickness,
                '--c1-h-alpha', c1_h_alpha,
                '--c2-h-alpha', c2_h_alpha,
                '--c1-h-dash', c1_h_dash,
                '--c1-h-gap',  c1_h_gap,
                '--c2-h-dash', c2_h_dash,
                '--c2-h-gap',  c2_h_gap,
            ],
        ),
    ])
