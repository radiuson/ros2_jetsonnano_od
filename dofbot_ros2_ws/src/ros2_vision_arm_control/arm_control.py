#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import time
import ikpy.chain
import utils
from utils import ikpy_utils
from utils import (URDF_PATH,
                   CAMERA_MOUNT_INDEX,
                   TOPIC_ROBOT_STATUS,
                   TOPIC_ROBOT_TRANSFORM,
                   TOPIC_ARM_CONTROL
                   )

class ArmControl(Node):
    def __init__(self,urdf_path):
        super().__init__('arm_control_node')
        acive_links_mask = [False, True, True, True, True,False,False]
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)
        self.arm_device = Arm_Device()
        self.declare_parameter('servo_speed', 3.5)
        self.servo_speed = self.get_parameter('servo_speed').get_parameter_value().double_value
        self.grabber_positions = {
            'open': 80,
            'close': 160,
        }
        self.command_subscription = self.create_subscription(
            String,
            TOPIC_ARM_CONTROL,
            self.command_callback,
            10
        )
        self.transform_msg = Float32MultiArray()
        self.status_msg = String()
        self.joint_msg = String()
        self.status_publisher = self.create_publisher(String, TOPIC_ROBOT_STATUS, 10)
        self.transform_publisher = self.create_publisher(Float32MultiArray, TOPIC_ROBOT_TRANSFORM, 10)
        
        self.timer = self.create_timer(1.0, self.transform_callback)
    
    def command_callback(self, msg):
        """ 处理收到的控制指令 """
        try:
            # 解析目标位置，例如 "100, 50, 30, open"
            command = msg.data.split(",")
            
            target_position = list(map(float, command[:3]))  # 转换为浮点数
            grabber_state = command[3].strip().lower()
            grabber_position = self.grabber_positions.get(grabber_state, 80)  # 默认打开状态
            
            self.get_logger().info(f"Received command: {target_position}, Grabber: {grabber_state}")
            
            # 控制机械臂
            self.control_arm(target_position, grabber_position)

        except Exception as e:
            self.get_logger().error(f"Failed to process command: {e}")
    
    def read_servolines(self):
        angle = []
        time.sleep(0.02)
        for i in range(6):
            aa = self.arm_device.Arm_serial_servo_read(i + 1)
            angle.append(aa if aa else 0)
            time.sleep(0.002)
        time.sleep(0.002)
        return angle
    

    def servo_write(self, angle, s_time=None):
        calculate_time = self.calculate_servotime(angle)
        if s_time:
            self.arm_device.Arm_serial_servo_write6(*angle, time=s_time)
            time.sleep(s_time / 1000)
            return s_time
        else:
            self.arm_device.Arm_serial_servo_write6(*angle, time=calculate_time)
            time.sleep(calculate_time / 1000)
            return calculate_time

    def calculate_servotime(self, target):
        servotime = np.abs(np.array(self.read_servolines()) - np.array(target))
        return int(max(max(servotime) * self.servo_speed * 5, 500))

    # 按照索引返回关节位置
    def calculate_chain_transform(self,index=CAMERA_MOUNT_INDEX):
        # 计算机械臂的正向运动学
        joint_angles = self.read_servolines()
        converted_joint_angles = ikpy_utils.util_ikpy_d2r(joint_angles)
        chain_transform = self.chain.forward_kinematics(converted_joint_angles,full_kinematics=True)
        if index:
            return np.array(chain_transform[index], dtype=np.float32).ravel().tolist()
        else:
            return chain_transform
    

    
    
    def transform_callback(self):
        # 获取机械臂末端执行器的变换矩阵
        # 发布消息
        self.transform_msg.data = self.calculate_chain_transform()
        self.transform_publisher.publish(self.transform_msg)
        self.get_logger().info("Transform matrix published")

    def control_arm(self, target_position, grabber_position):
        
        self.status_msg.data = "MOVING"
        self.status_publisher.publish(self.status_msg)
        self.get_logger().info("Robot Status: MOVING")
        
        joint_angles = self.calculate_joint_angles(target_position)
        joint_angles.append(grabber_position)
        self.servo_write(joint_angles)

        self.status_msg.data = "IDLE"
        self.status_publisher.publish(self.status_msg)
        self.get_logger().info("Robot Status: IDLE")

    def calculate_joint_angles(self, target_position):
        # Perform inverse kinematics calculation using ikpy
        ik_results = self.chain.inverse_kinematics(target_position = target_position,initial_position = None)
        joint_angles = ikpy_utils.util_ikpy_r2d(ik_results)[:-1]
        print(joint_angles)
        return joint_angles

def main(args=None): 
    rclpy.init(args=args)
    arm_control = ArmControl(URDF_PATH)
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # rclpy.init(args=None)
    # arm_control = ArmControl(utils.URDF_PATH)
    # test = arm_control.calculate_chain_transform()
    # print(test)
    # print(arm_control.transform_to_string(test))
