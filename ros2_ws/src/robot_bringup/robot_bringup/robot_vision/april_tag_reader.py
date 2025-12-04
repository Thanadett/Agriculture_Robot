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
import json
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory
import os

class AprilTagReader(Node):
    """ROS 2 node that reads images and detects AprilTags."""

    def __init__(self):
        super().__init__('april_tag_reader')

        # --- Parameters ---
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('output_path', 'april_tags.txt')
        self.declare_parameter('output_mode', 'txt')   # "txt" or "json"
        self.declare_parameter('frame_skip', 1)        # process every Nth frame

        p = self.get_parameter
        self.image_topic = p('image_topic').value
        self.output_path = p('output_path').value
        self.output_mode = p('output_mode').value.lower()
        self.frame_skip = max(1, int(p('frame_skip').value))

        if self.output_mode not in ('txt', 'json'):
            self.get_logger().warn(
                f"Invalid output_mode='{self.output_mode}', falling back to 'txt'."
            )
            self.output_mode = 'txt'

        # --- Open output file in append mode ---
        # Each line = one processed frame (if any tags are detected)
        try:
            self.file = open(self.output_path, 'a', encoding='utf-8')
        except Exception as e:
            self.get_logger().error(f"Failed to open output file '{self.output_path}': {e}")
            raise

        # --- Prepare AprilTag detector (via ArUco) ---
        # Requires opencv-contrib-python with aruco module.
        try:
            from cv2 import aruco
        except Exception as e:
            self.get_logger().error(
                "cv2.aruco is not available. "
                "Please install opencv-contrib-python in the same environment, e.g.:\n"
                "  pip install opencv-contrib-python\n"
                f"Error: {e}"
            )
            raise

        self.aruco = aruco
        # Use AprilTag 36h11 dictionary (common choice)
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_APRILTAG_36h11)
        self.parameters = aruco.DetectorParameters()
        self.frame_count = 0

        # --- Subscriber ---
        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            10
        )

        self.get_logger().info(
            f"AprilTagReader started:\n"
            f"  image_topic = {self.image_topic}\n"
            f"  output_path = {self.output_path}\n"
            f"  output_mode = {self.output_mode}\n"
            f"  frame_skip  = {self.frame_skip}\n"
            f"  dict        = DICT_APRILTAG_36h11"
        )

    # ------------------------------------------------------------------
    # Image message -> numpy array (no cv_bridge)
    # ------------------------------------------------------------------
    def _image_msg_to_numpy(self, msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image (bgr8 or rgb8 or mono8) to numpy array."""
        # Determine number of channels from encoding
        enc = msg.encoding.lower()

        if enc in ('bgr8', 'rgb8'):
            channels = 3
        elif enc in ('mono8', '8uc1'):
            channels = 1
        else:
            # Fallback: assume 3 channels
            self.get_logger().warn_once(
                f"Unknown encoding '{msg.encoding}', assuming BGR8 with 3 channels."
            )
            channels = 3

        # Create numpy array from the raw data buffer
        img_array = np.frombuffer(msg.data, dtype=np.uint8)

        if channels == 1:
            img = img_array.reshape((msg.height, msg.width))
        else:
            img = img_array.reshape((msg.height, msg.width, channels))

        # If RGB, convert to BGR for OpenCV consistency
        if enc == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        return img

    # ------------------------------------------------------------------
    # Image callback: detect AprilTags and write result to file
    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        self.frame_count += 1

        # Skip frames if frame_skip > 1
        if self.frame_count % self.frame_skip != 0:
            return

        # Convert to numpy image
        try:
            frame_bgr = self._image_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert Image to numpy: {e}")
            return

        # Convert to grayscale
        if frame_bgr.ndim == 3:
            gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame_bgr

        # Detect AprilTags via aruco.detectMarkers
        corners, ids, _ = self.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters=self.parameters
        )

        if ids is None or len(ids) == 0:
            # No tags detected for this frame
            return

        # Flatten ids array to Python int list
        tag_ids = [int(x[0]) for x in ids]
        self.get_logger().info(f"Detected AprilTags: {tag_ids}")

        # Timestamp (float seconds)
        stamp = msg.header.stamp
        t_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        # Write to file depending on mode
        try:
            if self.output_mode == 'txt':
                # Example line:  1764698000.123456 : 10, 5
                line = f"{t_sec:.6f} : " + ", ".join(str(i) for i in tag_ids) + "\n"
                self.file.write(line)
            else:
                # JSON per line
                record = {
                    "timestamp": t_sec,
                    "ids": tag_ids,
                    "num_tags": len(tag_ids),
                }
                self.file.write(json.dumps(record, ensure_ascii=False) + "\n")

            self.file.flush()

        except Exception as e:
            self.get_logger().error(f"Failed to write AprilTag IDs to file: {e}")

    # ------------------------------------------------------------------
    def close(self):
        """Clean up file handle."""
        try:
            if hasattr(self, 'file') and self.file is not None:
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
