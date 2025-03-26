#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

from utils import TOPIC_CAMERA_DEPTH, TOPIC_CAMERA_RGB

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.rgb_publisher = self.create_publisher(Image, TOPIC_CAMERA_RGB, 10)
        self.depth_publisher = self.create_publisher(Image, TOPIC_CAMERA_DEPTH, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert RealSense frames to ROS Image messages
        color_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(color_frame.get_data()), encoding='bgr8'
        )
        depth_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(depth_frame.get_data()), encoding='16UC1'
        )

        # Publish the images
        self.rgb_publisher.publish(color_image)
        self.depth_publisher.publish(depth_image)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()