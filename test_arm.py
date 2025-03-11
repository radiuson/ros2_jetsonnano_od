from Arm_Lib import Arm_Device
import time
Arm = Arm_Device()
if Arm:
    print("arm success")
Arm.Arm_serial_set_torque(1)
time.sleep(0.02)
Arm.Arm_serial_servo_write6(0,90,90,0,90,90,3000)
angles = []
for i in range(6):
    aa = Arm.Arm_serial_servo_read(i+1)
    if aa:
        angles.append(aa)
    else:
        angles.append(0)
    time.sleep(.002)
print(angles)
