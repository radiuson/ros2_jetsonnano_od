import numpy as np

def util_ikpy_d2r(degrees):
    return np.radians([0,*[(90 - x) for x in degrees]])
    # return np.radians([0,*[(x) for x in degrees]])

def util_ikpy_r2d(radians):
    return [int(90 - x) for x in np.degrees(radians[1:])]
    # return [int(x) for x in np.degrees(radians[1:])]
