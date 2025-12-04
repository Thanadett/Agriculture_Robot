#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#ros2 topic echo /camera/image_raw --no-arr


import sys
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    """Publish webcam frames as sensor_msgs/Image without cv_bridge."""

    def __init__(self):
        super().__init__('camera_publisher')

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        p = self.get_parameter
        self.device_id = int(p('device_id').value)
        self.width = int(p('width').value)
        self.height = int(p('height').value)
        self.fps = max(1, int(p('fps').value))
        self.image_topic = p('image_topic').value
        self.frame_id = p('frame_id').value

        # Publisher
        self.pub = self.create_publisher(Image, self.image_topic, 10)

        # Open camera
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera device {self.device_id}")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
        self.cap.set(cv2.CAP_PROP_FPS, float(self.fps))

        # Timer
        self.timer = self.create_timer(1.0 / float(self.fps), self._step)

        self.get_logger().info(
            f"camera_publisher started: dev={self.device_id}, "
            f"{self.width}x{self.height}@{self.fps}Hz -> {self.image_topic}"
        )

    def _step(self):
        """Grab one frame and publish as Image."""
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to read frame")
            return

        # Ensure correct size
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))

        # Ensure BGR8 uint8
        if frame.dtype != np.uint8:
            frame = frame.astype(np.uint8)

        # Build sensor_msgs/Image manually
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3  # 3 bytes per pixel (B, G, R)
        msg.data = frame.tobytes()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.pub.publish(msg)

    def close(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[camera_publisher] Fatal error: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
