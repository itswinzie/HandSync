#!/usr/bin/env python3
# ============================================================
# MasterPi Camera Publisher
# Publish camera USB MasterPi ke ROS2 topic /masterpi/camera
# Run dalam Docker container MasterPi
# ============================================================

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('masterpi_camera')
        self.publisher_ = self.create_publisher(
            CompressedImage,
            'masterpi/camera/compressed',
            10
        )
        # Timer 30fps
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)

        # Buka camera
        self.cap = None
        for idx in [0, 1, 2]:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.cap = cap
                self.get_logger().info(f'Camera found: index {idx}')
                break

        if not self.cap:
            self.get_logger().error('Tiada camera!')
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.get_logger().info('Camera Publisher ready! Topic: /masterpi/camera/compressed')

    def publish_frame(self):
        if not self.cap:
            return
        ok, frame = self.cap.read()
        if not ok:
            return

        # Compress ke JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buffer.tobytes()
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()
    node = CameraPublisher()
    print('=' * 50)
    print('  MasterPi Camera Publisher')
    print('  Topic: /masterpi/camera/compressed')
    print('  Ctrl+C untuk berhenti')
    print('=' * 50)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
