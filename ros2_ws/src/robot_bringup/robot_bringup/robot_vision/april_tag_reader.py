#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AprilTag reader node.

This node:
  - Subscribes to a sensor_msgs/Image topic (e.g. /camera/image_raw)
  - Converts the image message to a numpy array (no cv_bridge)
  - Detects AprilTags using OpenCV ArUco (DICT_APRILTAG_36h11)
  - Writes detected tag IDs to a text or JSON file

Parameters:
  - image_topic (str):  Image topic to subscribe (default: "/camera/image_raw")
  - output_path (str):  Output file path (default: "april_tags.txt")
  - output_mode (str):  "txt" or "json" (default: "txt")
  - frame_skip (int):   Process every Nth frame (default: 1, i.e. every frame)
"""
import sys
import os
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pathlib import Path


class AprilTagReader(Node):
    """Subscribe image topic, detect AprilTags, log IDs to file."""

    def __init__(self):
        super().__init__('april_tag_reader')

        # ---- Parameters ----
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_name', 'april_tags.txt')  # file name only
        # default dir: ~/Agriculture_Robot/ros2_ws/src/robot_bringup/robot_bringup/robot_vision
        default_dir = Path.home() / "Agriculture_Robot" / "ros2_ws" / "src" \
            / "robot_bringup" / "robot_bringup" / "robot_vision"
        self.declare_parameter('output_dir', str(default_dir))
        self.declare_parameter('output_mode', 'txt')             # "txt" or "json"
        self.declare_parameter('frame_skip', 1)                  # process every Nth frame

        p = self.get_parameter
        self.image_topic = p('image_topic').value
        output_name = p('output_name').value
        output_dir = Path(p('output_dir').value)
        self.output_mode = p('output_mode').value.lower()
        self.frame_skip = max(1, int(p('frame_skip').value))

        if self.output_mode not in ('txt', 'json'):
            self.get_logger().warn(f"Invalid output_mode='{self.output_mode}', use 'txt'.")
            self.output_mode = 'txt'

        # ---- Resolve & create output directory ----
        output_dir.mkdir(parents=True, exist_ok=True)
        self.output_path = str(output_dir / output_name)

        try:
            self.file = open(self.output_path, 'a', encoding='utf-8')
        except Exception as e:
            self.get_logger().error(f"Cannot open output file '{self.output_path}': {e}")
            raise

        # ---- AprilTag detector ----
        try:
            from cv2 import aruco
        except Exception as e:
            self.get_logger().error(
                "cv2.aruco not available. Install opencv-contrib-python:\n"
                "  pip install opencv-contrib-python\n"
                f"Error: {e}"
            )
            raise

        self.aruco = aruco
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters()
        self.frame_count = 0

        self.sub = self.create_subscription(Image, self.image_topic, self._image_callback, 10)

        self.get_logger().info(
            f"AprilTagReader started\n"
            f"  image_topic = {self.image_topic}\n"
            f"  output_path = {self.output_path}\n"
            f"  output_mode = {self.output_mode}\n"
            f"  frame_skip  = {self.frame_skip}\n"
            f"  dict        = DICT_APRILTAG_36h11"
        )

    def _image_msg_to_numpy(self, msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('bgr8', 'rgb8'):
            img = data.reshape((msg.height, msg.width, 3))
            if enc == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif enc in ('mono8', '8uc1'):
            img = data.reshape((msg.height, msg.width))
        else:
            self.get_logger().warn_once(
                f"Unknown encoding '{msg.encoding}', assuming BGR8."
            )
            img = data.reshape((msg.height, msg.width, 3))
        return img

    def _image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return

        try:
            frame_bgr = self._image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Image->numpy failed: {e}")
            return

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY) if frame_bgr.ndim == 3 else frame_bgr
        corners, ids, _ = self.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        if ids is None or len(ids) == 0:
            return

        tag_ids = [int(x[0]) for x in ids]
        self.get_logger().info(f"Detected AprilTags: {tag_ids}")

        stamp = msg.header.stamp
        t_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        try:
            if self.output_mode == 'txt':
                line = f"{t_sec:.6f} : " + ", ".join(str(i) for i in tag_ids) + "\n"
                self.file.write(line)
            else:
                record = {"timestamp": t_sec, "ids": tag_ids, "num_tags": len(tag_ids)}
                self.file.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.file.flush()
        except Exception as e:
            self.get_logger().error(f"Write to file failed: {e}")

    def close(self):
        try:
            if getattr(self, 'file', None) is not None:
                self.file.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AprilTagReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[april_tag_reader] Fatal error: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
