import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
# Removed unused import
from std_msgs.msg import String, Float32MultiArray
import numpy as np
# Removed unused import
from cv_bridge import CvBridge
import json
import pyrealsense2 as rs
from std_srvs.srv import Trigger
import time
from utils import (TOPIC_YOLO_DEPTH,
                   TOPIC_YOLO_DETECTION,
                   TOPIC_ARM_CONTROL,
                   TOPIC_ROBOT_STATUS,
                   TOPIC_ROBOT_TRANSFORM,
                   MOUNT_TO_CAMERA_OFFSET,
                   TOPIC_CAMERA_INFO,
                   DISTORTION_MAPPING,
                   TRIGGER_CAMERA_INFO,
                   TRIGGER_YOLO_DEPTH,
                   DEPTH_IMAGE_SCALE,
                   ARM_COMPEN,
                   )

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.bridge = CvBridge()
        self.depth_intrinsics = None
        self.depth_image = None
        self.compen = ARM_COMPEN
        self.arm_state = 'IDLE'
        self.create_subscription(CameraInfo, TOPIC_CAMERA_INFO, self.camera_info_callback, 10)
        
        # Clients
        self.trigger_camera_info_client = self.create_client(Trigger, TRIGGER_CAMERA_INFO)
        self.depth_image_client = self.create_client(Trigger,TRIGGER_YOLO_DEPTH)
        # Get camera info
        while not self.trigger_camera_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service trigger camera info not ready')
        
        self.get_logger().info('Service trigger camera info ready')


        self.send_camera_info_request()
        # Subscribers
        self.create_subscription(String, TOPIC_ROBOT_STATUS, self.arm_state_callback, 2)
        self.create_subscription(Float32MultiArray, TOPIC_ROBOT_TRANSFORM,self.camera_mount_transform_callback, 2)
        self.create_subscription(String, TOPIC_YOLO_DETECTION, self.yolo_callback, 2)
        self.create_subscription(Image, TOPIC_YOLO_DEPTH, self.depth_callback, 2)
        self.command_publisher = self.create_publisher(
                String,
                TOPIC_ARM_CONTROL,
                10
            )
        init_position = [-0.05,0,0.20]
        init_x,init_y,init_z = init_position
        self.arm_control_msg = String()
        self.arm_control_msg.data = f"{init_x},{init_y},{init_z},open"
        self.command_publisher.publish(self.arm_control_msg)
        # Variables
        self.get_logger().info(f"Node Initialized: {self.get_name()}")

    def send_camera_info_request(self):
        # 创建一个空的 Trigger 请求
        req = Trigger.Request()
        
        # 异步发送请求
        future = self.trigger_camera_info_client.call_async(req)
        
        # 注册回调
        future.add_done_callback(self.trigger_callback)

    def trigger_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('请求成功：' + response.message)
            else:
                self.get_logger().info('请求失败：' + response.message)
        except Exception as e:
            self.get_logger().error('请求过程中出错：%r' % (e,))

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
        # self.get_logger().info(f"Rotation: {self.camera_rotation}")

        # self.get_logger().info(f"Camera coords: {self.camera_coords}")
        
    def yolo_callback(self, msg):
        # Process YOLO results
        self.yolo_results = self.decode_yolo_results(msg.data)
        target_class = 'tomato'
        try:
            tomatoes = self.yolo_results[target_class]
            next_tomato = tomatoes[-1]
            if self.depth_intrinsics is not None and self.depth_image is not None:
                tomato_coor_cam = self.pixel_to_camera_coords(next_tomato['center'][0],next_tomato['center'][1])
                self.get_logger().info(f"In camera_coor is {tomato_coor_cam}")
                # 如果是由绿色机械臂发布
                tomato_coor_world = self.camera_rotation @ tomato_coor_cam + self.camera_coords 
                self.get_logger().info(f"Tomato coor is {tomato_coor_world}")
                if self.compen:
                    tomato_coor_world = self.compensator(tomato_coor_world)
                self.grab_tomato(init_position=[-0.05,0,0.20],tomato_position=tomato_coor_world,drop_position=[0,0.05,0.25])
                # ros2 topic pub /arm_control std_msgs/msg/String "data: '-0.2820361, 0.04377077,0.25786347, open'"
            else:
                self.get_logger().info(f"Waiting for depth intrinsics and image")
        except (KeyError,ValueError) as e:
            if isinstance(e, KeyError):
                    self.get_logger().info(f"No {target_class} detected (KeyError)")
            elif isinstance(e, ValueError):
                self.get_logger().info(f"No valid depth for {target_class} ")
    def grab_tomato(self,init_position,tomato_position,drop_position):
        # 格式化目标坐标
        target_x = tomato_position[0]
        target_y = tomato_position[1]
        target_z = tomato_position[2]
        
        drop_x = drop_position[0]
        drop_y = drop_position[1]
        drop_z = drop_position[2]

        init_x = init_position[0]
        init_y = init_position[1]
        init_z = init_position[2]

        self.wait_for_idle()

        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, open"
        self.command_publisher.publish(self.arm_control_msg)
        self.wait_for_idle()

        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, close"
        self.command_publisher.publish(self.arm_control_msg)
        self.wait_for_idle()

        self.arm_control_msg.data = f"{drop_x},{drop_y},{drop_z}, close"
        self.command_publisher.publish(self.arm_control_msg)
        self.wait_for_idle()

        self.arm_control_msg.data = f"{drop_x},{drop_y},{drop_z}, open"
        self.command_publisher.publish(self.arm_control_msg)
        self.wait_for_idle()

        self.arm_control_msg.data = f"{init_x},{init_y},{init_z}, open"
        self.command_publisher.publish(self.arm_control_msg)
        self.wait_for_idle()

    def wait_for_idle(self):
        time.sleep(0.1)

        while self.arm_state == 'MOVING':
            self.get_logger().info(f"Waiting for idle")
            time.sleep(0.1)
    
        self.get_logger().info("Arm is IDLE, ready to move.")


    def depth_callback(self, msg, scale=DEPTH_IMAGE_SCALE):
        # Process depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') / scale
    
    def compensator(self,P_world):
        z = P_world[2]
        z = z + 0.03 * (np.abs(P_world[1])+np.abs(P_world[0])) + (0.45 - z)*0.02
        P_world[2] = z
        return np.array(P_world)
    
    def camera_info_callback(self, msg):
        # Process camera intrinsic parameters
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        # 从 CameraInfo 消息中提取相机内参
        fx = msg.k[0]  # 焦距 fx
        fy = msg.k[4]  # 焦距 fy
        ppx = msg.k[2]  # 主点位置 ppx
        ppy = msg.k[5]  # 主点位置 ppy
        w = msg.width
        h = msg.height
        
        # 创建对应的深度相机内参对象
        self.depth_intrinsics = rs.pyrealsense2.intrinsics()
        self.depth_intrinsics.height = h
        self.depth_intrinsics.width = w 
        self.depth_intrinsics.fx = fx
        self.depth_intrinsics.fy = fy
        self.depth_intrinsics.ppx = ppx
        self.depth_intrinsics.ppy = ppy
        self.depth_intrinsics.coeffs = self.dist_coeffs
        self.depth_intrinsics.model = DISTORTION_MAPPING[msg.distortion_model]

    def decode_yolo_results(self, data):
        # Placeholder for YOLO result processing
        try:
            # Parse the JSON string into a Python list of dictionaries
            yolo_data = json.loads(data)
            results = {}

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

                    # Add the result to the dictionary, using class_name as the key
                    if class_name not in results:
                        results[class_name] = []

                    results[class_name].append({
                        "confidence": confidence,
                        "center": (center_x, center_y),
                        "bbox": bbox
                    })

            return results

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode YOLO data: {e}")
            return {}

    def perform_hand_eye_calibration(self,mount_to_camera):
        # Perform hand-eye calibration
        # self.get_logger().info(f"mount_transform:{self.mount_transform}")
        # This should compute the transformation matrix between the camera and the robot arm
        return self.mount_transform @ mount_to_camera

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

    def pixel_to_camera_coords(self, pixel_x, pixel_y):
        depth = self.depth_image[pixel_y,pixel_x]
        try:
            if depth <0.01:
                raise ValueError(f"Invalid depth at pixel ({pixel_x}, {pixel_y}): {depth}")
            x,y,z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [pixel_x,pixel_y], depth)
            # self.get_logger().info(f"x:{x},y:{y},z:{z}")
            return np.array([x, y, z])
        except ValueError as e:
            self.get_logger().error(f"Not in the range: {e}")
            return None

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


def rotation_matrix_to_euler_xyz(R):
    """
    将旋转矩阵R转换为欧拉角（XYZ顺序，单位：弧度）
    """
    assert R.shape == (3, 3), "输入必须是3×3旋转矩阵"

    if abs(R[0, 2]) < 1 - 1e-6:  # 正常情况
        y_angle = np.arcsin(-R[0, 2])
        x_angle = np.arctan2(R[1, 2], R[2, 2])
        z_angle = np.arctan2(R[0, 1], R[0, 0])
    else:  # 接近万向节锁 (gimbal lock)
        # cos(y) == 0，sin(y) == ±1
        y_angle = np.pi/2 if R[0, 2] < 0 else -np.pi/2
        x_angle = np.arctan2(-R[1, 0], R[1, 1])
        z_angle = 0

    return np.array([x_angle, y_angle, z_angle])




if __name__ == '__main__':
    main()
    # rclpy.init(args=None)
    # motion_planner = MotionPlanner()
    