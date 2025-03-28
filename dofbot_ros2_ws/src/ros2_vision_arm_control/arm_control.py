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
from utils import ROBOT_STATUS

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
        self.status_msg = String()
        self.status_publisher = self.create_publisher(String, ROBOT_STATUS, 10)

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