
import time
from Arm_Lib import Arm_Device
def read_servolines(Arm):
    angle = []
    time.sleep(0.02)
    for i in range(6):
        aa = Arm.Arm_serial_servo_read(i+1)
        if aa:
            angle.append(aa)
        else:
            angle.append(0)
        time.sleep(.002)
    time.sleep(.002)
    return angle
def servo_write(Arm,angle,s_time=3000):
    Arm.Arm_serial_servo_write6(angle[0], angle[1], angle[2], angle[3], angle[4], angle[5], s_time)
    time.sleep(s_time/1000)
    return s_time

if __name__ == "__main__":
    Arm = Arm_Device()
    servo_write(Arm=Arm,angle=[0,90,90,0,90,90])