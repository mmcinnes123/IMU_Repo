

from helpers_2DoF import get_np_quats_from_txt_file
from helpers_2DoF import get_ang_vels_from_quats
from joint_axis_est_2d import jointAxisEst2D

import qmt
import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)


# Read in some IMU quaternion data from a TMM report .txt file
# input_txt_file = str(askopenfilename(title=' Choose the raw data .txt file with IMU/quat data ... '))
input_txt_file = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P1\RawData\P1_JA_Slow - Report2 - IMU_Quats.txt'
IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(input_txt_file)

# Trim the IMU data based on the period of interest
start_ind = 22 * 100
end_ind = 42 * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]


# Define the input data from the jointAxiEst2D function
quat1 = IMU2_trimmed     # This is the humerus IMU data
quat2 = IMU3_trimmed     # This is the forearm IMU data
rate = 100          # This is the sample rate of the data going into the function
gyr1 = None
gyr2 = None

# params = dict(method='ori')
# results = jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=params, debug=True, plot=False)
# print(results)
#

#
params = dict(method='rot')
results = jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=params, debug=True, plot=False)
print(results)

# TODO: appear to have got both methods working, but very much not in agreement - need to look a little closer/debug, check I'm doing what I think I am.
# Could be that ang val from the quat data is too noisy to produce good estimates ( I feel like this is whyy I just used TMM ang vels when
# doing frame calibration stuff...)
# ??? Is the quat data I'm using here filtered in TMM?
# What is the 'cost' in the debug output?

# # generate example data
# t = qmt.timeVec(T=20, Ts=0.05)
# quat = q1
# axis = np.column_stack([np.cos(t), np.zeros_like(t), np.sin(t)])
# # quat = qmt.quatFromAngleAxis(np.sin(t), axis)
# data = qmt.Struct(t=t, quat=quat)
#
# # run webapp
# webapp = qmt.Webapp('/view/imubox', data=data)
# webapp.run()