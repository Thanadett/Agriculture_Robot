from launch_ros.actions import Node


def get_camera_nodes(
    video_device,
    width,
    height,
    fps_num,
    fps_den,
    enable_compressed=True,
    qos_best_effort=True,
):
    # พารามิเตอร์กล้อง (รองรับ LaunchConfiguration ที่ส่งเข้ามาได้)
    v4l2_params = {
        'video_device': video_device,
        'image_size': [width, height],
        'time_per_frame': [fps_num, fps_den],  # e.g. 1/30
        'output_encoding': 'mjpeg',            # ลดแบนด์วิธ/หน่วง
    }

    # ตั้ง QoS เป็น best_effort ช่วยลดอาการค้างเฟรมถ้าเครือข่ายช้าชั่วคราว
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

    # เพิ่ม compressed image สำหรับดูผ่านเน็ต/SSH
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

    # Foxglove Bridge (WebSocket :8765)
    nodes.append(
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
        )
    )

    return nodes
