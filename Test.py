
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd

my_quat = [-0.766,  0.071, -0.196, 0.608]

my_quat_R = R.from_quat(my_quat)

print("Quaternion_a: ", my_quat)

print("DCM_a: ")
print(my_quat_R.as_matrix())

print("x-axis:", my_quat_R.as_matrix()[:,0])

print("x-axis projected on the XY plane: :", my_quat_R.as_matrix()[0:2,0])

