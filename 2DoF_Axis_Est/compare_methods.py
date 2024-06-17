

from helpers_2DoF import get_np_quats_from_txt_file
from helpers_2DoF import get_joint_axis_directly_from_ang_vels
from joint_axis_est_2d import jointAxisEst2D

import qmt
import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)



# Read in some IMU quaternion data from a TMM report .txt file
# input_txt_file = str(askopenfilename(title=' Choose the raw data .txt file with IMU/quat data ... '))
input_txt_file = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P1\RawData\P1_JA_Fast - Report2 - IMU_Quats.txt'
IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(input_txt_file)

rate = 100          # This is the sample rate of the data going into the function
FE_start_time = 8
FE_end_time = 17
PS_start_time = 18
PS_end_time = 26

# Trim the IMU data based on the period of interest
start_ind = FE_start_time * 100
end_ind = PS_end_time * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]

# Define the input data from the jointAxiEst2D function
quat1 = IMU2_trimmed     # This is the humerus IMU data
quat2 = IMU3_trimmed     # This is the forearm IMU data
gyr1 = None
gyr2 = None

# Run the ori method
params = dict(method='ori')
ori_results = jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=params, debug=True, plot=False)
print('J1 axis estimation with ori method: ', ori_results['j1'])
print('J2 axis estimation with ori method: ', ori_results['j2'])

# Run the rot method
params = dict(method='rot')
rot_results = jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=params, debug=True, plot=False)
print('J1 axis estimation with rot method: ', rot_results['j1'])
print('J2 axis estimation with rot method: ', rot_results['j2'])
print('and heading offset: ', rot_results['delta']*180/np.pi)


# Get joint axis estimates directly from ang vel data, assuming subject perfectly isolated each joint DoF
start_ind = FE_start_time * 100
end_ind = FE_end_time * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]
params = dict(jointAxis='j1')
FE_axis = get_joint_axis_directly_from_ang_vels(IMU2_trimmed, IMU3_trimmed, rate, params=params, debug_plot=False)
print('J1 estimate direct from angular velocities: ', FE_axis)

# Get joint axis estimates directly from ang vel data, assuming subject perfectly isolated each joint DoF
start_ind = PS_start_time * 100
end_ind = PS_end_time * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]
params = dict(jointAxis='j2')
PS_axis = get_joint_axis_directly_from_ang_vels(IMU2_trimmed, IMU3_trimmed, rate, params=params, debug_plot=False)
print('J2 estimate direct from angular velocities: ', PS_axis)


