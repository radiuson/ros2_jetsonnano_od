import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
import numpy as np
import time
import ikpy.chain
import os

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        urdf_path = os.path.join(os.path.dirname(__file__), '..', 'urdf', 'dofbot.urdf')
        acive_links_mask = [False, True, True, True, True,True]
        self.chain = ikpy.chain.Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)
        self.arm_device = Arm_Device()
        self.declare_parameter('servo_speed', 3.5)
        self.servo_speed = self.get_parameter('servo_speed').get_parameter_value().double_value
        self.link_list = [0, 30, 83, 83, 80, 90]
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
            self.arm_device.Arm_serial_servo_write6(*angle, s_time)
            time.sleep(s_time / 1000)
            return s_time
        else:
            self.arm_device.Arm_serial_servo_write6(*angle, calculate_time)
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
        ik_results = self.chain.inverse_kinematics(target_position)
        joint_angles = [ik_results[i] for i in range(1, 6)]  # Skip the base joint
        return joint_angles

def main(args=None):
    rclpy.init(args=args)
    arm_control = ArmControl()
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # main()
    rclpy.init(args=None)
    arm_control = ArmControl()
    arm_control.servo_write([0, 90, 90, 0, 90, 90])
    target_position = np.array([0, 0.3, 0.3])
    arm_control.control_arm(target_position, arm_control.grabber_positions['open'])