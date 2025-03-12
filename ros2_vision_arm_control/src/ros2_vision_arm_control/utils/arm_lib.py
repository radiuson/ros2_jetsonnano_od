import time
import numpy as np
import serial

class Arm_Device:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.servo_angles = [0] * 6
        self.serial_connection = serial.Serial(port, baudrate)

    def Arm_serial_servo_read(self, servo_id):
        self.serial_connection.write(f'R{servo_id}\n'.encode())
        response = self.serial_connection.readline().decode().strip()
        return int(response) if response.isdigit() else None

    def Arm_serial_servo_write6(self, angle1, angle2, angle3, angle4, angle5, angle6, s_time):
        command = f'S{angle1},{angle2},{angle3},{angle4},{angle5},{angle6},{s_time}\n'
        self.serial_connection.write(command.encode())
        self.servo_angles = [angle1, angle2, angle3, angle4, angle5, angle6]

    def close(self):
        self.serial_connection.close()

def calculate_servotime(current_angles, target_angles, servo_speed=3):
    servotime = np.array(current_angles) - np.array(target_angles)
    return int(max(max(np.abs(servotime)) * servo_speed * 5, 500))