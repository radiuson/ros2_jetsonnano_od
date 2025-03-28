import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge
from utils import TOPIC_YOLO_DEPTH, TOPIC_YOLO_DETECTION, TOPIC_ROBOT_STATUS

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(String, TOPIC_ROBOT_STATUS, self.arm_state_callback, 10)
        self.create_subscription(Image, TOPIC_YOLO_DETECTION, self.yolo_callback, 10)
        self.create_subscription(Image, TOPIC_YOLO_DEPTH, self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Variables
        self.arm_state = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.depth_image = None
        self.yolo_results = None
        self.hand_eye_matrix = None  # Hand-eye calibration matrix

    def arm_state_callback(self, msg):
        # Process arm state
        self.arm_state = msg.data
        self.get_logger().info(f"Received arm state: {self.arm_state}")

    def yolo_callback(self, msg):
        # Process YOLO results
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.yolo_results = self.process_yolo_results(cv_image)

    def depth_callback(self, msg):
        # Process depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        # Process camera intrinsic parameters
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def process_yolo_results(self, image):
        # Placeholder for YOLO result processing
        # Extract bounding boxes or object positions
        return []

    def perform_hand_eye_calibration(self):
        # Perform hand-eye calibration
        # This should compute the transformation matrix between the camera and the robot arm
        self.hand_eye_matrix = np.eye(4)  # Placeholder for actual calibration

    def locate_object(self):
        if self.yolo_results is None or self.depth_image is None or self.camera_matrix is None:
            self.get_logger().warn("Missing data for object localization")
            return None

        # Example: Assume YOLO results provide pixel coordinates of the object
        for obj in self.yolo_results:
            pixel_x, pixel_y = obj['center']
            depth = self.depth_image[pixel_y, pixel_x]

            # Convert pixel coordinates to camera coordinates
            camera_coords = self.pixel_to_camera_coords(pixel_x, pixel_y, depth)

            # Convert camera coordinates to world coordinates
            world_coords = self.camera_to_world_coords(camera_coords)
            self.get_logger().info(f"Object located at: {world_coords}")
            return world_coords

    def pixel_to_camera_coords(self, pixel_x, pixel_y, depth):
        # Convert pixel coordinates to camera coordinates using intrinsic parameters
        x = (pixel_x - self.camera_matrix[0, 2]) * depth / self.camera_matrix[0, 0]
        y = (pixel_y - self.camera_matrix[1, 2]) * depth / self.camera_matrix[1, 1]
        z = depth
        return np.array([x, y, z])

    def camera_to_world_coords(self, camera_coords):
        # Convert camera coordinates to world coordinates using hand-eye calibration
        if self.hand_eye_matrix is None:
            self.get_logger().warn("Hand-eye calibration not performed")
            return None
        camera_coords_homogeneous = np.append(camera_coords, 1)  # Convert to homogeneous coordinates
        world_coords_homogeneous = np.dot(self.hand_eye_matrix, camera_coords_homogeneous)
        return world_coords_homogeneous[:3]

def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()
    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()