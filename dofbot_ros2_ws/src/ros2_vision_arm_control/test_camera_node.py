#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from utils import TOPIC_CAMERA_RGB  # Ensure this topic name is defined in utils

class ImagePublisherNode(Node):
    def __init__(self):
        super().__init__('image_publisher_node')
        self.publisher = self.create_publisher(Image, TOPIC_CAMERA_RGB, 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_image)  # Publish every second
        image_path = "/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/output0113_mov-0057.jpg"
        self.image = cv2.imread(image_path)  # Change this to your actual image path

        if self.image is None:
            self.get_logger().error("Failed to load image")

    def publish_image(self):
        if self.image is not None:
            ros_image = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(ros_image)
            self.get_logger().info("Published image successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
