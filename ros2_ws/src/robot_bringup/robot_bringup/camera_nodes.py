from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def get_camera_nodes(
    video_device,
    width,
    height,
    fps_num,
    fps_den,
    enable_compressed=True,
    qos_best_effort=True,
    add_foxglove=False,
):
    """
    สร้าง list ของ ROS2 nodes สำหรับกล้อง
    รองรับ LaunchConfiguration ของ width/height/fps
    """

    image_size_expr = PythonExpression(
        ["[int(", width, "), int(", height, ")]"]
    )
    time_per_frame_expr = PythonExpression(
        ["[int(", fps_num, "), int(", fps_den, ")]"]
    )

    v4l2_params = {
        'video_device': video_device,
        'image_size': image_size_expr,
        'time_per_frame': time_per_frame_expr,
        'output_encoding': 'mjpeg',
    }

    if qos_best_effort:
        v4l2_params['qos_overrides'] = {
            '/image_raw': {
                'depth': 5,
                'reliability': 'best_effort',
                'history': 'keep_last',
            }
        }

    nodes = [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='usb_cam',
            output='screen',
            parameters=[v4l2_params],
        )
    ]

    if enable_compressed:
        nodes.append(
            Node(
                package='image_transport',
                executable='republish',
                name='img_republish',
                output='screen',
                arguments=['raw', 'compressed'],
                remappings=[('in', '/image_raw'),
                            ('out', '/image_raw/compressed')],
            )
        )

    if add_foxglove:
        nodes.append(
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
            )
        )

    return nodes
