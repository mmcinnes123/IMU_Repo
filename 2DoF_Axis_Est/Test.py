import numpy as np
import itertools

def axisToThetaPhi(j, var):
    if var == 1:
        theta = np.arccos(j[2])
        phi = np.arctan2(j[1], j[0])
    elif var == 2:
        theta = np.arccos(j[0])
        phi = np.arctan2(j[1], j[2])
    else:
        raise ValueError('invalid axis var')
    return theta, phi


e_x = np.array([1, 0.01, 0.01], float)
e_y = np.array([0.01, 1, 0.01], float)
e_z = np.array([0.01, 0.01, 1], float)
e_x /= np.linalg.norm(e_x)
e_y /= np.linalg.norm(e_y)
e_z /= np.linalg.norm(e_z)
init = []
for j1, j2, delta in itertools.product([e_x, e_y, e_z], [e_x, e_y, e_z], np.deg2rad([-90, 0, 90, 180])):
    init.append(np.r_[axisToThetaPhi(j1, 1), axisToThetaPhi(j2, 1), delta, 1, 1])

print(np.array(init, float))
print(len(np.array(init, float)))