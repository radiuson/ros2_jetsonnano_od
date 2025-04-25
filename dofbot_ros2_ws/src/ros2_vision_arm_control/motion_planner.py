import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
import cv2
from functools import partial
from ros2_vision_arm_control.msg import VisionDetection, BoundingBox
from utils import (TOPIC_YOLO_DEPTH,
                   TOPIC_YOLO_DETECTION,
                   TOPIC_ARM1_CONTROL,
                   TOPIC_ROBOT1_STATUS,
                   TOPIC_ROBOT1_TRANSFORM,
                   TOPIC_ARM2_CONTROL,
                   TOPIC_ROBOT2_STATUS,
                   TOPIC_ROBOT2_TRANSFORM,
                   MOUNT_TO_CAMERA_OFFSET,
                   TOPIC_CAMERA_INFO,
                   DISTORTION_MAPPING,
                   TRIGGER_CAMERA_INFO,
                   TRIGGER_YOLO_DEPTH,
                   DEPTH_IMAGE_SCALE,
                   ARM_COMPEN,
                   TOPIC_YOLO_RESULT,
                   VISUALIZATION,
                   ROTATION_INTERIOR,
                   )

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.bridge = CvBridge()
        self.depth_intrinsics = None
        self.depth_image = None
        self.compen = ARM_COMPEN
        self.leaf_grab = False
        self.tomato_grab = False
        self.yolo_wait_count = 0
        self.arm_state = ['IDLE','IDLE']
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
        self.create_subscription(String, TOPIC_ROBOT1_STATUS, partial(self.arm_state_callback,index = 0), 2)
        self.create_subscription(Float32MultiArray, TOPIC_ROBOT1_TRANSFORM,self.camera_mount_transform1_callback, 2)
        self.create_subscription(String, TOPIC_ROBOT2_STATUS, partial(self.arm_state_callback,index = 1), 2)
        self.create_subscription(Float32MultiArray, TOPIC_ROBOT2_TRANSFORM,self.camera_mount_transform2_callback, 2)
        
        self.yolo_result_subscription = self.create_subscription(
            VisionDetection,
            TOPIC_YOLO_RESULT,  
            self.yolo_result_callback,
            3  # 队列大小
        )
        # 单独分别订阅
        # self.create_subscription(String, TOPIC_YOLO_DETECTION, self.yolo_callback, 2)
        # self.create_subscription(Image, TOPIC_YOLO_DEPTH, self.depth_callback, 2)
        
        self.command_publisher1 = self.create_publisher(
                String,
                TOPIC_ARM1_CONTROL,
                10
            )        
        self.command_publisher2 = self.create_publisher(
                String,
                TOPIC_ARM2_CONTROL,
                10
            )
        init_position = [0.05,0.05,0.23]
        init_x,init_y,init_z = init_position
        self.arm_control_msg = String()
        self.arm_control_msg.data = f"{init_x},{init_y},{init_z},open,80"
        self.command_publisher1.publish(self.arm_control_msg)
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

    def arm_state_callback(self, msg, index):
        # Process arm state
        self.arm_state[index] = msg.data

    def camera_mount_transform1_callback(self, msg):
        # Convert the transform string back to a numpy array
        self.mount_transform1 = np.array(msg.data).reshape(4,4)
        # Perform hand-eye calibration
        self.camera_transform1 = self.perform_hand_eye_calibration(mount_transform =self.mount_transform1, mount_to_camera=MOUNT_TO_CAMERA_OFFSET)
        self.camera_coords1 = self.camera_transform1[:3,3]
        self.camera_rotation1 = self.camera_transform1[:3,:3]

        # self.get_logger().info(f"Mount rotation: {self.mount_transform1[:3,:3]}")

        

        # self.get_logger().info(f"Rotation: {self.camera_rotation1}")

        self.get_logger().info(f"Mount coords: {self.mount_transform1[:3,3]}")
        self.get_logger().info(f"Camera coords: {self.camera_coords1}")

    def camera_mount_transform2_callback(self, msg):
        # Convert the transform string back to a numpy array
        self.mount_transform2 = np.array(msg.data).reshape(4,4)
        # Perform hand-eye calibration
        self.camera_transform2 = self.perform_hand_eye_calibration(mount_transform =self.mount_transform2,mount_to_camera=MOUNT_TO_CAMERA_OFFSET)
        self.camera_coords2 = self.camera_transform2[:3,3]
        self.camera_rotation2 = self.camera_transform2[:3,:3]
        # self.get_logger().info(f"Rotation: {self.camera_rotation}")

        # self.get_logger().info(f"Mount coords: {self.mount_transform2[:3,3]}")
        # self.get_logger().info(f"Camera coords: {self.camera_coords2}")

    def yolo_result_callback(self, msg: VisionDetection):
        # self.get_logger().info("Received YOLO detection message.")
        try:
            if self.leaf_grab is not True:
                    
                target_class = 'leaf'
                max_ymax=0
                found_leaf = None
                for box in msg.boxes:  
                    if box.class_name == target_class:
                        if box.ymax > max_ymax:
                            max_ymax = box.ymax
                            found_leaf = box
                if self.yolo_wait_count < 2:
                    self.yolo_wait_count = self.yolo_wait_count + 1
                    return
                self.yolo_wait_count = 0
                if found_leaf is not None:
                    # 计算中心点坐标（像素坐标）
                    center_x = (found_leaf.xmin + found_leaf.xmax) // 2
                    center_y = ((found_leaf.ymin + found_leaf.ymax) // 2+found_leaf.ymax) //2

                    if self.depth_intrinsics is not None and msg.depth_image is not None:
                        self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth_image, desired_encoding='passthrough') / DEPTH_IMAGE_SCALE
                        leaf_coor_cam = self.pixel_to_camera_coords(center_x, center_y)
                        
                        self.get_logger().info(f"In camera_coor is {leaf_coor_cam}")

                        # 坐标转换为世界坐标系（使用相机的外参）
                        leaf_coor_world = self.camera_rotation1 @ leaf_coor_cam + self.camera_coords1
                        self.get_logger().info(f"Leaf coor is {leaf_coor_cam}")

                        # 是否使用补偿器
                        # if self.compen:
                        #     tomato_coor_world = self.compensator(tomato_coor_world)
                        
                        idle_position = [0.25,0.13,0.35]
                        idle_x = idle_position[0]
                        idle_y = idle_position[1]
                        idle_z = idle_position[2]
                        target_x = leaf_coor_world[0]
                        target_y = leaf_coor_world[1]
                        target_z = leaf_coor_world[2]
                        self.arm_control_msg.data = f"{0.20},{0},{0.20}, open"
                        self.command_publisher2.publish(self.arm_control_msg)
                        self.wait_for_idle([0,1])
                        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, open"
                        self.command_publisher2.publish(self.arm_control_msg)
                        self.wait_for_idle([0,1])
                        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, close"
                        self.command_publisher2.publish(self.arm_control_msg)
                        self.wait_for_idle([0,1])


                        self.arm_control_msg.data = f"{idle_x},{idle_y},{idle_z}, close"
                        self.command_publisher2.publish(self.arm_control_msg)
                        self.wait_for_idle([0,1])

                        # 你可以调用抓取函数或其他控制函数
                        self.leaf_grab = True
                    else:
                        self.get_logger().info("Waiting for depth intrinsics or depth image...")
            if self.leaf_grab is True and self.tomato_grab is not True:
                self.grab_object(msg,'tomato')
        except (ValueError,AttributeError) as e:
            self.get_logger().info(f"Something went wrong {e}")



    def grab_object(self,msg:VisionDetection,target_class='tomato'):
        target_class = target_class
        found_tomato = None
        try:
            # 遍历消息中的所有目标框，找出类别是 "tomato" 的目标
            max_ymax=0
            for box in msg.boxes:  
                if box.class_name == target_class:
                    if box.ymax > max_ymax:
                        max_ymax = box.ymax
                        found_tomato = box

            if self.yolo_wait_count < 2:
                self.yolo_wait_count = self.yolo_wait_count + 1
                return
            self.yolo_wait_count = 0
            if found_tomato is not None:
                self.tomato_grab = True
                # 计算中心点坐标（像素坐标）
                center_x = (found_tomato.xmin + found_tomato.xmax) // 2
                center_y = (found_tomato.ymin + found_tomato.ymax) // 2
                

                if self.depth_intrinsics is not None and msg.depth_image is not None:
                    self.depth_image = self.bridge.imgmsg_to_cv2(msg.depth_image, desired_encoding='passthrough') / DEPTH_IMAGE_SCALE
                    tomato_coor_cam = self.pixel_to_camera_coords(center_x, center_y)
                    rotation = self.object_axis_angle([found_tomato.xmin,
                                                       found_tomato.ymin,
                                                       found_tomato.xmax,
                                                       found_tomato.ymax],self.depth_image)
                    self.get_logger().info(f"In camera_coor is {tomato_coor_cam}")

                    # 坐标转换为世界坐标系（使用相机的外参）
                    tomato_coor_world = self.camera_rotation1 @ tomato_coor_cam + self.camera_coords1
                    self.get_logger().info(f"Tomato coor is {tomato_coor_world}")

                    # 是否使用补偿器
                    # if self.compen:
                    #     tomato_coor_world = self.compensator(tomato_coor_world)

                    # 你可以调用抓取函数或其他控制函数
                    self.grab_tomato(init_position=[0.06,0.06,0.24],
                                     tomato_position=tomato_coor_world,
                                     drop_position=[0,0.07,0.10],
                                     rotation=rotation)
                    self.tomato_grab = False
                else:
                    self.get_logger().info("Waiting for depth intrinsics or depth image...")
            else:
                self.get_logger().info(f"No {target_class} detected.")
        except (KeyError,ValueError,AttributeError) as e:
            if isinstance(e, KeyError):
                    self.get_logger().info(f"No {target_class} detected (KeyError)")
            elif isinstance(e, ValueError):
                self.get_logger().info(f"No valid depth for {target_class} ")
            elif isinstance(e, AttributeError):
                self.get_logger().info(f"Arm state not received{e}")
                
    def object_axis_angle(self, xyxy, depth_image, padding=20):
        x1, y1, x2, y2 = xyxy
        h, w = depth_image.shape

        # 计算需要填充的边界
        top_pad = max(0, padding - y1)
        left_pad = max(0, padding - x1)
        bottom_pad = max(0, y2 + padding - h)
        right_pad = max(0, x2 + padding - w)

        # 加边框，防止越界
        padded_image = cv2.copyMakeBorder(
            depth_image,
            top=top_pad,
            bottom=bottom_pad,
            left=left_pad,
            right=right_pad,
            borderType=cv2.BORDER_CONSTANT,
            value=0
        )

        # 偏移坐标（因为图像变大了）
        x1 += left_pad
        x2 += left_pad
        y1 += top_pad
        y2 += top_pad

        # 裁剪区域
        obj_depth_image = padded_image[y1 - padding:y2 + padding, x1 - padding:x2 + padding]

        # 计算直方图
        hist, bin_edges = np.histogram(obj_depth_image, 120, range=(100 / DEPTH_IMAGE_SCALE, 500 / DEPTH_IMAGE_SCALE))
        min_val = bin_edges[max(0, np.argmax(hist) - 3)]
        max_val = bin_edges[min(len(bin_edges) - 1, np.argmax(hist) + 4)]

        # 生成 mask 并提取点
        mask = (obj_depth_image > min_val) & (obj_depth_image < max_val)
        y_coords, x_coords = np.where(mask)

        if len(x_coords) == 0:
            print("No valid depth points found.")
            return None

        points = np.column_stack((x_coords, y_coords)).astype(np.float32)

        # 最小外接矩形计算角度
        rect = cv2.minAreaRect(points)
        (center_x, center_y), (width, height), angle = rect

        if width < height:
            short_axis_angle = angle + 90
        else:
            short_axis_angle = angle

        print("short_axis_angle:", short_axis_angle)
        return short_axis_angle
    

    # def yolo_callback(self, msg):
    #     # Process YOLO results
    #     self.yolo_results = self.decode_yolo_results(msg.data)
    #     target_class = 'tomato'
    #     try:
    #         tomatoes = self.yolo_results[target_class]
    #         next_tomato = tomatoes[-1]
    #         if self.depth_intrinsics is not None and self.depth_image is not None:
    #             tomato_coor_cam = self.pixel_to_camera_coords(next_tomato['center'][0],next_tomato['center'][1])
    #             self.get_logger().info(f"In camera_coor is {tomato_coor_cam}")
    #             # 如果是由绿色机械臂发布
    #             tomato_coor_world = self.camera_rotation @ tomato_coor_cam + self.camera_coords 
    #             self.get_logger().info(f"Tomato coor is {tomato_coor_world}")
    #             if self.compen:
    #                 tomato_coor_world = self.compensator(tomato_coor_world)
    #             # self.grab_tomato(init_position=[-0.05,0.05,0.23],tomato_position=tomato_coor_world,drop_position=[0,0.05,0.25])
    #             # ros2 topic pub /arm_control std_msgs/msg/String "data: '-0.2820361, 0.04377077,0.25786347, open'"
    #         else:
    #             self.get_logger().info(f"Waiting for depth intrinsics and image")
    #     except (KeyError,ValueError) as e:
    #         if isinstance(e, KeyError):
    #                 self.get_logger().info(f"No {target_class} detected (KeyError)")
    #         elif isinstance(e, ValueError):
    #             self.get_logger().info(f"No valid depth for {target_class} ")

                
    def grab_tomato(self,init_position,tomato_position,drop_position,rotation):
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

        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{(target_x+init_x)/2},{(target_y+init_y)/2},{(target_z+init_z)/2}, open,{rotation}"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, open,{rotation}"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{target_x},{target_y},{target_z}, close,{rotation}"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{drop_x},{drop_y},{drop_z}, close,90"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{drop_x},{drop_y},{drop_z}, open,90"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

        self.arm_control_msg.data = f"{init_x},{init_y},{init_z}, open,90"
        self.command_publisher1.publish(self.arm_control_msg)
        self.wait_for_idle([0,1])

    def wait_for_idle(self, index_list: list, timeout: float = 3.0):
        start_time = time.time()
        time.sleep(0.5)  # 留一点时间让订阅启动

        while True:
            moving_indices = [index for index in index_list if self.arm_state[index] == 'MOVING']
            if not moving_indices:
                self.get_logger().info("All arms are IDLE, ready to move.")
                return True

            if time.time() - start_time > timeout:
                self.get_logger().warn(f"Timeout waiting for arms {moving_indices} to become IDLE.")
                return False

            self.get_logger().info(f"Waiting for arms {moving_indices} to become IDLE...")
            time.sleep(0.2)



    def depth_callback(self, msg, scale=DEPTH_IMAGE_SCALE):
        # Process depth image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough') / scale
        self.get_logger().error(f"depth image received")

    def compensator(self,P_world):
        z = P_world[2]
        # z = z + 0.03 * (np.abs(P_world[1])+np.abs(P_world[0])) + (0.45 - z)*0.02
        z = 1.02 * z
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
        # self.depth_intrinsics.fx = fx
        # self.depth_intrinsics.fy = fy
        self.depth_intrinsics.fx = 607.5303
        self.depth_intrinsics.fy = 607.5303
        self.depth_intrinsics.ppx = ppx
        self.depth_intrinsics.ppy = ppy
        self.depth_intrinsics.coeffs = self.dist_coeffs
        self.depth_intrinsics.model = DISTORTION_MAPPING[msg.distortion_model]
    
    def deproject_pixel_to_point(self,pixel, depth, intrinsics):
        """
        将像素坐标和深度值转换为相机坐标系下的三维坐标。

        参数：
        - pixel: 像素位置 (u, v)
        - depth: 该像素的深度值（单位：米）
        - intrinsics: 相机内参字典，包括 fx, fy, cx, cy

        返回：
        - 3D 坐标 (X, Y, Z)，以相机为原点，单位：米
        """
        u, v = pixel
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']

        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth

        return np.array([X, Y, Z])
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

    def perform_hand_eye_calibration(self,mount_transform,mount_to_camera):
        # Perform hand-eye calibration
        # self.get_logger().info(f"mount_transform:{self.mount_transform}")
        # This should compute the transformation matrix between the camera and the robot arm
        return mount_transform @ mount_to_camera


    def pixel_to_camera_coords(self, pixel_x, pixel_y):
        depth = self.depth_image[pixel_y,pixel_x]
        print(self.depth_image.shape)
        
        self.get_logger().info(f"depth at pixel ({pixel_x}, {pixel_y}): {depth}")
        try:
            if depth <0.01:
                raise ValueError(f"Invalid depth at pixel ({pixel_x}, {pixel_y}): {depth}")
            x,y,z = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [pixel_x,pixel_y], depth+0.005)
            intrinsics = {
                    'fx': 387.771,
                    'fy': 387.771,
                    'cx': 322.846,
                    'cy': 243.269
            }
            u,v,w = self.deproject_pixel_to_point([pixel_x,pixel_y],depth,intrinsics)
            # self.get_logger().info(f"x:{x},y:{y},z:{z}")
            if VISUALIZATION:
                depth_filtered = np.clip(self.depth_image, 0, 60)  # 超过1000的值变成1000
                depth_colormap = cv2.normalize(depth_filtered, None, 0, 255, cv2.NORM_MINMAX)
                depth_colormap = np.uint8(depth_colormap)
                depth_colormap = cv2.applyColorMap(depth_colormap, cv2.COLORMAP_JET)
                a,b,c = self.camera_rotation1 @ [x,y,z] + self.camera_coords1
                u,v,w = self.camera_rotation1 @ [u,v,w] + self.camera_coords1
                # 在图上标注你想查看的点
                cv2.circle(depth_colormap, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
                depth_text = f"x:{a:.3f},y:{b:.3f},z:{c:.3f},depth:{depth:.3f},u:{u:.3f},v:{v:.3f},w:{w:.3f}"
                cv2.putText(depth_colormap, depth_text, (30,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.imshow("Depth Image", depth_colormap)
                cv2.waitKey(1000)  # 1000ms刷新
            return np.array([x, y, z])
        except (ValueError,AttributeError) as e:
            self.get_logger().error(f"{e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(motion_planner)
    try:
        executor.spin()
    finally:
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
    