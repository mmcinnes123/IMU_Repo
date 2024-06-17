

from helpers_2DoF import get_np_quats_from_txt_file
from helpers_2DoF import get_joint_axis_directly_from_ang_vels
from helpers_2DoF import get_local_ang_vels_from_quats
from helpers_2DoF import plot_gyr_data
from joint_axis_est_2d import jointAxisEst2D

import qmt
import itertools
from scipy.spatial.transform import Rotation as R

import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)


# Read in some IMU quaternion data from a TMM report .txt file
# input_txt_file = str(askopenfilename(title=' Choose the raw data .txt file with IMU/quat data ... '))
input_txt_file = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P2\RawData\P2_JA_Slow - Report3 - Cluster_Quats.txt'
IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(input_txt_file)

rate = 100          # This is the sample rate of the data going into the function
start_time = 15
end_time = 44
# Trim the IMU data based on the period of interest
start_ind = start_time * 100
end_ind = end_time * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]

# Define the input data from the jointAxiEst2D function
quat1 = IMU2_trimmed     # This is the humerus IMU data
quat2 = IMU3_trimmed     # This is the forearm IMU data
gyr1 = None
gyr2 = None

# Run the rot method
params = dict(method='rot')
rot_results = jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=params, debug=True, plot=False)
print('J1 axis estimation with rot method: ', rot_results['j1'])
print('J2 axis estimation with rot method: ', rot_results['j2'])
print('and heading offset: ', rot_results['delta']*180/np.pi)
est_FE_in_clus = rot_results['j1']


""" FINDING REFERNECE J1 AXIS IN HUMERUS CLUSTER FRAME """

# Get the FE axis expressed in the model's humerus body frame

# Based on how the hu joint is defined in the model, the XYZ euler ori offset of the parent frame,
# relative to humerus frame is:
hu_parent_rel2_hum_R = R.from_euler('XYZ', [0, 0, 0.32318], degrees=False)

# Based on how the hu joint is defined in the model, relative to the hu joint parent frame,
# the vector of hu rotation axis (EL_x) is:
EL_axis_rel2_hu_parent = [0.969, -0.247, 0]

# Get the vector of hu rotation axis, relative to the humerus frame
EL_axis_rel2_humerus = hu_parent_rel2_hum_R.apply(EL_axis_rel2_hu_parent)
print("The model's elbow flexion axis in the humerus frame is: ", EL_axis_rel2_humerus)


# Get the cluster frame, expressed in the humerus frame

# For P1
marker_4_in_hum = np.array([0.063474422328633373, -0.13139587787176832, -0.020512161979728986])
marker_1_in_hum = np.array([0.064459068112010853, -0.20484887126065421, -0.030803789834133628])
marker_3_in_hum = np.array([0.064646041343107585, -0.1398308659971248, 0.037837390318982089])
# y_axis is marker 4 to marker 1 (pointing down)
y_axis = marker_1_in_hum - marker_4_in_hum

# x_axis is marker 4 to marker 3 (pointing backwards)
x_axis = marker_3_in_hum - marker_4_in_hum

cluster_in_hum = qmt.quatFrom2Axes(x_axis, y_axis, None, plot=False)


# Now express the FE axis in the cluster frame

FE_in_clus, debug = qmt.rotate(cluster_in_hum, EL_axis_rel2_humerus, debug=True, plot=False)

print('FE axis in humerus cluster frame is:', FE_in_clus)



""" COMPARE WITH OPTIMISATION """




error = qmt.angleBetween2Vecs(FE_in_clus, est_FE_in_clus)
print('Error: ', error*180/np.pi)


# C:\Users\r03mm22\Anaconda3\envs\osim\python.exe C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\2DoF_Axis_Est\Test_2DoF.py
# J1 axis estimation with rot method:  [ 0.03143068 -0.15197927  0.98788381]
# J2 axis estimation with rot method:  [-0.00567985  0.99552285  0.09435041]
# and heading offset:  -7.144593368967835
# The model's elbow flexion axis in the humerus frame is:  [0.99727829 0.07352556 0.        ]
# FE axis in humerus cluster frame is: [ 0.02075694 -0.21315791  0.97678189]
# Error:  3.6152628461894496
    