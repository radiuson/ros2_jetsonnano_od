#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
from std_msgs.msg import String
import numpy as np
import time
import ikpy.chain
import os
import utils
from utils import ikpy_utils
from utils import ROBOT_STATUS,ROBOT_JOINTS,JOINT_INTERVAL,ARM_CONTROL

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
            ARM_CONTROL,
            self.command_callback,
            10
        )

        self.status_msg = String()
        self.joint_msg = String()
        self.status_publisher = self.create_publisher(String, ROBOT_STATUS, 10)
        self.joints_publisher = self.create_publisher(String, ROBOT_JOINTS, 10)
        self.timer = self.create_timer(JOINT_INTERVAL, self.publish_joint_angles)
    
    def command_callback(self, msg):
        """ 处理收到的控制指令 """
        try:
            # 解析目标位置，例如 "100, 50, 30, open"
            command = msg.data.split(",")
            if len(command) != 4:
                self.get_logger().warn("Invalid command format. Expected: x,y,z,gripper_state")
                return
            
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
    
    def publish_joint_angles(self):
        """ 持续发布当前关节角度 """
        joint_angles = self.read_servolines()
        self.joint_msg.data = ",".join(map(str, joint_angles))  # 角度数据用逗号分隔
        self.joints_publisher.publish(self.joint_msg)
        self.get_logger().info(f"Published Joint Angles: {self.joint_msg.data}")

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
        converted_target_position = ikpy_utils.util_ikpy_convert_coor(target_position)
        ik_results = self.chain.inverse_kinematics(target_position = converted_target_position,initial_position = None)
        joint_angles = ikpy_utils.util_ikpy_r2d(ik_results)[:-1]
        print(joint_angles)
        return joint_angles

def main(args=None): 
    rclpy.init(args=args)
    arm_control = ArmControl(utils.URDF_PATH)
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()