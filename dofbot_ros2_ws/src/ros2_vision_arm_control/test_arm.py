import matplotlib.pyplot as plt
import numpy as np
from ikpy.chain import Chain
import os
import numpy as np
from utils import URDF_PATH
from utils import ikpy_utils
def util_ikpy_d2r(degrees):
    return np.radians([0,*[(90 - x) for x in degrees]])
    # return np.radians([0,*[(x) for x in degrees]])

def util_ikpy_r2d(radians):
    return [int(90 - x) for x in np.degrees(radians[1:])]
    # return [int(x) for x in np.degrees(radians[1:])]


def calculate_joint_angles(chain,target_position):
    # Perform inverse kinematics calculation using ikpy
    initial_position = np.radians([0,0,30,-60,-50])
    ik_results = chain.inverse_kinematics(target_position = target_position,initial_position = initial_position)
    print(ik_results)
    joint_angles = util_ikpy_r2d(ik_results)
    print(joint_angles)
    cal_position = chain.forward_kinematics(ik_results)
    print(cal_position)
    return joint_angles


acive_links_mask = [False, True, True, True, True]
# acive_links_mask = [False, True, True, True, True,True,False]
urdf_path = URDF_PATH

# chain = Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)

chain = Chain.from_urdf_file("/home/jetson/code/dofbot_ros2_ws/src/ros2_vision_arm_control/urdf/dofbot copy.urdf",active_links_mask=acive_links_mask)
# 绘制机器人
arm_angle = [0, 89, 94, 38]
test_arm = util_ikpy_d2r(arm_angle)
target_position = [-0.1, 0, 0.25]
# ik_result = chain.inverse_kinematics(target_position)
# joint_angles=calculate_joint_angles(chain,target_position)
# print(joint_angles)
chain.plot(test_arm,ax=None,show=True)


plt.show()