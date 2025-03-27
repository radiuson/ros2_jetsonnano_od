import numpy as np

def util_ikpy_d2r(degrees):
    return np.radians([0,*[(90 - x) for x in degrees]])

def util_ikpy_r2d(radians):
    return [int(90 - x) for x in np.degrees(radians[1:])]

# Convert the xyz target position to the coordinate system used by ikpy
def util_ikpy_convert_coor(target_position):
    return np.array([-target_position[0], -target_position[1], target_position[2]])   