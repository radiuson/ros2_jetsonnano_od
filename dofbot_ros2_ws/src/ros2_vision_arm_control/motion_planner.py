import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
# Removed unused import
from std_msgs.msg import String, Float32MultiArray
import numpy as np
# Removed unused import
from cv_bridge import CvBridge
import json
from utils import (TOPIC_YOLO_DEPTH,
                   TOPIC_YOLO_DETECTION,
                   TOPIC_ROBOT_STATUS,
                   TOPIC_ROBOT_TRANSFORM,
                   MOUNT_TO_CAMERA_OFFSET,
                   TOPIC_CAMERA_INFO,
                   )

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(String, TOPIC_ROBOT_STATUS, self.arm_state_callback, 10)
        self.create_subscription(Float32MultiArray, TOPIC_ROBOT_TRANSFORM,self.camera_mount_transform_callback, 10)
        self.create_subscription(String, TOPIC_YOLO_DETECTION, self.yolo_callback, 10)
        self.create_subscription(Image, TOPIC_YOLO_DEPTH, self.depth_callback, 10)
        self.create_subscription(CameraInfo, TOPIC_CAMERA_INFO, self.camera_info_callback, 10)

        # Variables
        self.camera_transform = None
        self.arm_state = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.depth_image = None
        self.yolo_results = None
        self.hand_eye_matrix = None  # Hand-eye calibration matrix
        self.get_logger().info(f"Node Initialized: {self.get_name()}")
        
    def arm_state_callback(self, msg):
        # Process arm state
        self.arm_state = msg.data
        self.get_logger().info(f"Received arm state: {self.arm_state}")

    def camera_mount_transform_callback(self, msg):
        # Convert the transform string back to a numpy array
        self.mount_transform = np.array(msg.data).reshape(4,4)
        # Perform hand-eye calibration
        self.camera_transform = self.perform_hand_eye_calibration(mount_to_camera=MOUNT_TO_CAMERA_OFFSET)
        self.camera_coords = self.camera_transform[:3,3]
        self.camera_rotation = self.camera_transform[:3,:3]
        self.get_logger().info(f"Camera coords: {self.camera_coords}")
        

    def yolo_callback(self, msg):
        # Process YOLO results
        self.yolo_results = self.decode_yolo_results(msg.data)
        print(self.yolo_results)


    def depth_callback(self, msg):
        # Process depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        # Process camera intrinsic parameters
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def decode_yolo_results(self, data):
        # Placeholder for YOLO result processing
        try:
            # Parse the JSON string into a Python list of dictionaries
            yolo_data = json.loads(data)
            results = []
            for obj in yolo_data:
                # Extract relevant information
                class_name = obj.get("class")
                confidence = obj.get("confidence")
                bbox = obj.get("bbox")
                if class_name and confidence and bbox:
                    # Calculate the center of the bounding box
                    x_min, y_min, x_max, y_max = bbox
                    center_x = int((x_min + x_max) / 2)
                    center_y = int((y_min + y_max) / 2)
                    results.append({
                        "class": class_name,
                        "confidence": confidence,
                        "center": (center_x, center_y),
                        "bbox": bbox
                    })
            return results
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode YOLO data: {e}")
            return []

    def perform_hand_eye_calibration(self,mount_to_camera):
        # Perform hand-eye calibration
        # This should compute the transformation matrix between the camera and the robot arm
        return np.dot(self.mount_transform, mount_to_camera)

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
        self.get_logger().warning("Hand-eye calibration not performed")
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
    # rclpy.init(args=None)
    # motion_planner = MotionPlanner()
