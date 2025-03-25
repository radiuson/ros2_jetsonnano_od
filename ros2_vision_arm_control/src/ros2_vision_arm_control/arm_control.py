import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
import numpy as np
import time
import ikpy.chain
import os
from utils import *

class ArmControl(Node):
    def __init__(self,urdf_path):
        super().__init__('arm_control_node')
        acive_links_mask = [False, True, True, True, True,True,False]
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)
        self.arm_device = Arm_Device()
        self.declare_parameter('servo_speed', 3.5)
        self.servo_speed = self.get_parameter('servo_speed').get_parameter_value().double_value
        self.grabber_positions = {
            'open': 80,
            'close': 160,
        }

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
        joint_angles = self.calculate_joint_angles(target_position)
        joint_angles.append(grabber_position)
        self.servo_write(joint_angles)

    def calculate_joint_angles(self, target_position):
        # Perform inverse kinematics calculation using ikpy
        converted_target_position = ikpy_utils.util_ikpy_convert_coor(target_position)
        ik_results = self.chain.inverse_kinematics(target_position = converted_target_position,initial_position = None)
        joint_angles = ikpy_utils.util_ikpy_r2d(ik_results)[:-1]
        print(joint_angles)
        return joint_angles

def main(args=None): 
    rclpy.init(args=args)
    arm_control = ArmControl(urdf_path)
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main()
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'dofbot.urdf')
    rclpy.init(args=None)
    np.set_printoptions(suppress=True)
    arm_control = ArmControl(urdf_path) 
    arm_control.servo_write([90, 90, 90, 90, 90, 90])
    target_position = np.array([0.0, 0.2, 0.2])
    arm_control.control_arm(target_position, arm_control.grabber_positions['close'])