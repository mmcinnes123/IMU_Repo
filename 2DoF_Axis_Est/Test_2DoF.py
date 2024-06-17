

from helpers_2DoF import get_np_quats_from_txt_file
from helpers_2DoF import get_joint_axis_directly_from_ang_vels
from helpers_2DoF import get_local_ang_vels_from_quats
from helpers_2DoF import plot_gyr_data
from joint_axis_est_2d import jointAxisEst2D

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join

import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)


"""" Setting specific to subject """
subject_code = 'P1'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
OMC_dir = join(parent_dir, 'OMC')
raw_data_dir = join(parent_dir, 'RawData')
model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

# Data to use for the optimisation
trial_for_opt = 'JA_Slow'
IMU_type = 'Perfect'
time_dict = {'P1': {'JA_Slow': {'PS_start': 0, 'PS_end': 0, 'FE_start': 0, 'FE_end': 0}}}
sample_rate = 100          # This is the sample rate of the data going into the function

# TODO: turn the stuff below into a function which uses the settings above

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

# Read in calibrated model file to get position of humerus markers in humerus frame
my_model = osim.Model(model_file)
marker_1_in_hum = my_model.getMarkerSet().get('Hum_Clus_1').get_location().to_numpy()
marker_3_in_hum = my_model.getMarkerSet().get('Hum_Clus_3').get_location().to_numpy()
marker_4_in_hum = my_model.getMarkerSet().get('Hum_Clus_4').get_location().to_numpy()

# y_axis is marker 4 to marker 1 (pointing down)
y_axis = marker_1_in_hum - marker_4_in_hum

# x_axis is marker 4 to marker 3 (pointing backwards)
x_axis = marker_3_in_hum - marker_4_in_hum

# Get the cluster CF expressed in the humerus CF, using the marker positions
cluster_in_hum = qmt.quatFrom2Axes(x_axis, y_axis, None, plot=False)

# Now express the FE axis in the cluster frame
FE_in_clus, debug = qmt.rotate(cluster_in_hum, EL_axis_rel2_humerus, debug=True, plot=False)

print('FE axis in humerus cluster frame is:', FE_in_clus)


# TODO: Write same code to find PS in radius frame


""" FINDING FE AND PS FROM OPTIMISATION RESULT """



assert IMU_type in ['Real', 'Perfect'], 'IMU type not Real or Perfect'
if IMU_type == 'Perfect':
    report_ext = ' - Report3 - Cluster_Quats.txt'
elif IMU_type == 'Real':
    report_ext = ' - Report2 - IMU_Quats.txt'
else:
    report_ext = None

# Define the file name
tmm_txt_file_name = subject_code + '_' + trial_for_opt + report_ext
tmm_txt_file = join(raw_data_dir, tmm_txt_file_name)

# Read in the IMU quaternion data from a TMM report .txt file
IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(tmm_txt_file)

# Get the start and end time for which to run the optimisation
subject_time_dict = time_dict[subject_code]
PS_start_time = subject_time_dict['PS_start']
PS_end_time = subject_time_dict['PS_end']
FE_start_time = subject_time_dict['FE_start']
FE_end_time = subject_time_dict['FE_end']

# Trim the IMU data based on the period of interest
start_ind = FE_start_time * 100
end_ind = PS_end_time * 100
IMU2_trimmed = IMU2_np[start_ind:end_ind]
IMU3_trimmed = IMU3_np[start_ind:end_ind]

# Run the rot method
params = dict(method='rot')
rot_results = jointAxisEst2D(IMU2_trimmed, IMU3_trimmed, None, None, sample_rate, params=params, debug=True, plot=False)
print('J1 axis estimation with rot method: ', rot_results['j1'])
print('J2 axis estimation with rot method: ', rot_results['j2'])
print('and heading offset: ', rot_results['delta']*180/np.pi)






# """ COMPARE WITH OPTIMISATION """
#
# error = qmt.angleBetween2Vecs(FE_in_clus, est_FE_in_clus)
# print('Error: ', error*180/np.pi)


    