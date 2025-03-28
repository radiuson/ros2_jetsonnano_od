import matplotlib.pyplot as plt
import numpy as np
from ikpy.chain import Chain
import os
import numpy as np
from utils import URDF_PATH
def util_ikpy_d2r(degrees):
    return np.radians([0,*[(90 - x) for x in degrees]])

def util_ikpy_r2d(radians):
    return [(90 - x) for x in np.degrees(radians[1:])]

def calculate_joint_angles(chain,target_position):
    # Perform inverse kinematics calculation using ikpy
    initial_position = np.radians([0,0,30,-60,-50,0,0])
    ik_results = chain.inverse_kinematics(target_position = target_position,initial_position = initial_position)
    print(np.degrees(ik_results))
    joint_angles = [(int(np.degrees(ik_results[i]))) for i in range(1, 6)]  # Skip the base joint
    print(joint_angles)
    cal_position = chain.forward_kinematics(ik_results)
    print(cal_position)
    return joint_angles


acive_links_mask = [False, True, True, True, True,True,False]
urdf_path = URDF_PATH
chain = Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)
# 绘制机器人
target_position = [0.2, 0.2, 0.2]
ik_result = chain.inverse_kinematics(target_position)
joint_angles=calculate_joint_angles(chain,target_position)
print(joint_angles)
chain.plot(ik_result,ax=None)


plt.show()