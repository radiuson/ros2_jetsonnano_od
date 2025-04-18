#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
from std_srvs.srv import Trigger

from ros2_vision_arm_control.msg import VisionDetection, BoundingBox

from utils import (TOPIC_CAMERA_DEPTH,
                   TOPIC_CAMERA_RGB,
                   FRAME_RATE,
                   TOPIC_CAMERA_INFO,
                   TOPIC_CAMERA_MSG,
                   CAMERA_WIDTH,
                   CAMERA_HEIGHT,
                   TRIGGER_CAMERA_INFO,
                   )

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.camera_publisher = self.create_publisher(VisionDetection, TOPIC_CAMERA_MSG, 2)

        # 分别发布rgb图像和depth图像
        # self.rgb_publisher = self.create_publisher(Image, TOPIC_CAMERA_RGB, 2)
        # self.depth_publisher = self.create_publisher(Image, TOPIC_CAMERA_DEPTH, 2)

        self.timer = self.create_timer(round(1 / FRAME_RATE, 2), self.timer_vision_detection_callback)

        self.intrinics_publisher = self.create_publisher(CameraInfo, TOPIC_CAMERA_INFO, 2)

        self.camera_info_trigger = self.create_service(Trigger, TRIGGER_CAMERA_INFO, self.generate_camera_info)

        self.bridge = CvBridge()
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        
        config = rs.config()
        config.enable_stream(rs.stream.color, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, CAMERA_WIDTH, CAMERA_HEIGHT, rs.format.z16, 30)
        self.pipeline.start(config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        

    def generate_camera_info(self,request,response):
        # 获取深度相机的内参
        depth_intrinsics = self.pipeline.get_active_profile().get_stream(rs.stream.depth).as_video_stream_profile().intrinsics

        # 创建 CameraInfo 消息
        camera_info_msg = CameraInfo()
        camera_info_msg.width = CAMERA_WIDTH
        camera_info_msg.height = CAMERA_HEIGHT
        camera_info_msg.distortion_model = depth_intrinsics.model.name
        
        # 设置相机内参（包括焦距 fx, fy 和主点位置 ppx, ppy）
        camera_info_msg.k = [
            float(depth_intrinsics.fx), 0.0, float(depth_intrinsics.ppx),
            0.0, float(depth_intrinsics.fy), float(depth_intrinsics.ppy),
            0.0, 0.0, 1.0
        ]

        # 畸变系数 D，深度相机通常有 5 个畸变系数
        camera_info_msg.d = [
            depth_intrinsics.coeffs[0], depth_intrinsics.coeffs[1],
            depth_intrinsics.coeffs[2], depth_intrinsics.coeffs[3],
            depth_intrinsics.coeffs[4]
        ]
        
        # 旋转矩阵 R（通常是单位矩阵）
        camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # 投影矩阵 P
        camera_info_msg.p = [
            depth_intrinsics.fx, 0.0, depth_intrinsics.ppx, 0.0,
            0.0, depth_intrinsics.fy, depth_intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # 发布相机内参
        self.intrinics_publisher.publish(camera_info_msg)
        self.get_logger().info("Camera info Generated and Published")

        # 设置 Trigger 响应
        response.success = True
        response.message = 'Camera info generated and published'

        return response

    def timer_vision_detection_callback(self):

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()

        if not aligned_color_frame or not aligned_depth_frame:
            return

        # Convert RealSense frames to ROS Image messages
        color_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(aligned_color_frame.get_data()), encoding='bgr8'
        )
        depth_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(aligned_depth_frame.get_data()), encoding='16UC1'
        )
        print(f"depth shape:{np.asanyarray(aligned_depth_frame.get_data()).shape},color shape:{np.asanyarray(aligned_color_frame.get_data()).shape}")
        detection_msg = VisionDetection()
        detection_msg.rgb_image = color_image
        detection_msg.depth_image = depth_image

        boxes = []
        detection_msg.boxes = boxes

        self.camera_publisher.publish(detection_msg)
        self.get_logger().info("Published image")
        
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        aligned_color_frame = aligned_frames.get_color_frame()


        if not aligned_color_frame or not aligned_depth_frame:
            return

        # Convert RealSense frames to ROS Image messages
        color_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(aligned_color_frame.get_data()), encoding='bgr8'
        )
        depth_image = self.bridge.cv2_to_imgmsg(
            np.asanyarray(aligned_depth_frame.get_data()), encoding='16UC1'
        )

        # Publish the images
        self.rgb_publisher.publish(color_image)
        self.depth_publisher.publish(depth_image)
        self.get_logger().info("Published image and intrinics successfully.")

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