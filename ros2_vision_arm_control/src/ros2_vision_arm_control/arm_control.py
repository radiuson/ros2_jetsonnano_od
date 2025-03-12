import rclpy
from rclpy.node import Node
from Arm_Lib import Arm_Device
import numpy as np
import time
import ikpy

class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control_node')
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
        # Placeholder for inverse kinematics calculation
        return [0, 0, 0, 0, 0, 0]  # Replace with actual calculation

def main(args=None):
    rclpy.init(args=args)
    arm_control = ArmControl()
    rclpy.spin(arm_control)
    arm_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()