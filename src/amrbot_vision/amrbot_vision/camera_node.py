#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 320)
        self.declare_parameter('frame_height', 240)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('jpeg_quality', 80)  # 0-100

        self.camera_index = self.get_parameter('camera_index').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.fps = self.get_parameter('fps').value
        self.jpg_quality = self.get_parameter('jpeg_quality').value

        self.publisher = self.create_publisher(CompressedImage, 'camera_image/compressed', 10)
        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpg_quality]
        _, jpg_buf = cv2.imencode('.jpg', frame, encode_param)

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpg_buf.tobytes()

        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()