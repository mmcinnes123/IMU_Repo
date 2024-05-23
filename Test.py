
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd

quat_1 = [0.91826247, -0.00370748, -0.39586257, 0.00855082]
quat_2 = [-0.7240892, -0.07780611, 0.680013, -0.08499038]

R1 = R.from_quat([quat_1[1], quat_1[2], quat_1[3], quat_1[0]])
R2 = R.from_quat([quat_2[1], quat_2[2], quat_2[3], quat_2[0]])

jointR = R1*R2

print(jointR.as_euler())


