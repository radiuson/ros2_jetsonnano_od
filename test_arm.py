import matplotlib.pyplot as plt
import numpy as np
from ikpy.chain import Chain
import os
import numpy as np

def util_ikpy_d2r(degrees):
    return np.radians([0,*[(90 - x) for x in degrees]])

def util_ikpy_r2d(radians):
    return [(90 - x) for x in np.degrees(radians[1:])]

def util_ikpy_convert_coor(target_position):
    return np.array([-target_position[0], -target_position[1], target_position[2]])   

def calculate_joint_angles(self, target_position):
    # Perform inverse kinematics calculation using ikpy
    initial_position = np.radians([0,0,30,-60,-50,0,0])
    ik_results = chain.inverse_kinematics(target_position = target_position,initial_position = initial_position)
    print(np.degrees(ik_results))
    joint_angles = [(int(np.degrees(ik_results[i]))) for i in range(1, 6)]  # Skip the base joint
    print(joint_angles)
    cal_position = chain.forward_kinematics(ik_results)
    print(cal_position)
    return joint_angles

# 定义一个简单的 3 关节机器人
acive_links_mask = [False, True, True, True, True,True,False]
urdf_path = os.path.join(os.path.dirname(__file__), 'ros2_vision_arm_control','src','urdf', 'dofbot.urdf')
chain = Chain.from_urdf_file(urdf_path,active_links_mask=acive_links_mask)
# 绘制机器人
joints = [90,0,180,90,90,90]
target_position = [0.0, 0.2, 0.2]
converted = [(90 - x) for x in joints]
converted = [0, *converted]
converted = np.radians(converted)
chain.plot(converted,ax=None)
ik_result = chain.inverse_kinematics(util_ikpy_convert_coor(target_position))
chain.plot(ik_result,ax=None)


plt.show()