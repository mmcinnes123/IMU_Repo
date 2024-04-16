# A script to compare raw IMU orientation data with marker cluster data
# Comparing 'real' vs 'perfect' IMUs gives an indication of IMU error
# Relative orientation of two IMUs is assessed, as opposed to absolute orientation of one IMU
import numpy as np

from functions import *
import os
from scipy.spatial.transform import Rotation as R


""" SETTINGS """

# Quick Settings
IMU_input_file = 'P3_JA_Slow - Report2 - IMU_Quats.txt'
OMC_input_file = 'P3_JA_Slow - Report3 - Cluster_Quats.txt'
subject_code = 'P3'
trim_bool = True
start_time = 0
end_time = 95
sample_rate = 100

# Define some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
raw_data_dir = os.path.join(parent_dir, 'RawData')
IMU_input_file_path = os.path.join(raw_data_dir, IMU_input_file)
OMC_input_file_path = os.path.join(raw_data_dir, OMC_input_file)
accuracy_results_folder = os.path.join(parent_dir, 'IMU_Accuracy_Results')
if os.path.exists(accuracy_results_folder) == False:
    os.mkdir(accuracy_results_folder)


# Local misalignment rotations
IMU1_local_misalignment = R.from_quat([-1.200e-03, 1.000e-04, 1.010e-02, 9.999e-01])
IMU2_local_misalignment = R.from_quat([4.e-04, 3.e-03, 2.e-04, 1.e+00])
IMU3_local_misalignment = R.from_quat([0.0037, 0.0023, 0.0113, 0.9999])



""" MAIN """

# Read in IMU/Cluster orientation data from trial of interest
IMU1, IMU2, IMU3 = get_scipyR_from_txt_file(IMU_input_file_path, trim_bool, start_time, end_time, sample_rate)
OMC1, OMC2, OMC3 = get_scipyR_from_txt_file(OMC_input_file_path, trim_bool, start_time, end_time, sample_rate)

# Apply local misalignment rotations to the cluster CFs to better align them with the IMU frames
OMC1 = IMU1_local_misalignment * OMC1
OMC2 = IMU1_local_misalignment * OMC2
OMC3 = IMU1_local_misalignment * OMC3

# Get the relative orientation between IMUs/Clusters
IMU_joint12 = IMU1.inv() * IMU2
IMU_joint23 = IMU2.inv() * IMU3
IMU_joint13 = IMU1.inv() * IMU3
OMC_joint12 = OMC1.inv() * OMC2
OMC_joint23 = OMC2.inv() * OMC3
OMC_joint13 = OMC1.inv() * OMC3



""" COMPARE """

# Get the single-angle geodesic distance between Joint_R IMU and Joint_R OMC
# Rotational difference between IMU and OMC joint:
joint12_rot_diff = IMU_joint12.inv() * OMC_joint12
joint23_rot_diff = IMU_joint23.inv() * OMC_joint23
joint13_rot_diff = IMU_joint13.inv() * OMC_joint13
# Magnitude of that rotational difference:
joint12_dist_error = joint12_rot_diff.magnitude() * 180/np.pi
joint23_dist_error = joint23_rot_diff.magnitude() * 180/np.pi
joint13_dist_error = joint13_rot_diff.magnitude() * 180/np.pi


# Get the projected vector angles in the transverse, frontal, and sagittal planes, and compare IMU with OMC
"""
For joint12 (thorax-humerus), projected vectors are defined as follows: 
    Transverse plane: z_axis relative to Z on the XZ plane
    Frontal plane: y_axis relative to -X on the XY plane
    Sagittal plane: y_axis relative to Z on the YZ plane
    
For joint23 (humerus-forearm), projected vectors are defined as follows: 
    Transverse plane: y_axis relative to Z on the XZ plane
    Frontal plane: z_axis relative to Z on the ZY plane
    Sagittal plane: y_axis relative to Y on the XY plane
        """""

# Define a function for getting the projected vector of interest from the joint rotation matrix
def get_proj_vec_from_joint_rot_mat(rot_mat_R, local_axis, global_plane):

    matrices = rot_mat_R.as_matrix()

    column_dict = {'x': 0, 'y': 1, 'z': 2}
    row_dict = {'X': 0, 'Y': 1, 'Z': 2}

    column = column_dict[local_axis]
    row_a = row_dict[global_plane[0]]
    row_b = row_dict[global_plane[1]]

    array_of_vecs = matrices[:, [row_a, row_b], column]

    return array_of_vecs



# Define a function for calculating the angle between the local axis, and the chosen global axis
def get_proj_vec_angle(proj_vec, global_axis, global_plane):

    if global_axis == global_plane[0]:
        global_axis_vec = [1, 0]
    else:
        global_axis_vec = [0, 1]

    array_of_angles = np.arccos(np.dot(proj_vec, global_axis_vec) /
                                (np.linalg.norm(proj_vec, axis=1) * np.linalg.norm(global_axis_vec))) * 180 / np.pi

    return array_of_angles



# Define a function for getting the error between IMU and OMC projected angles
def get_error_arrays_and_stats(IMU_proj_vecs, OMC_proj_vecs, IMU_angles, OMC_angles):

    # Filter out projected vector angles when vector is close to normal to plane (i.e. not stable projection)
    # Use the magnitude of the projected vector as a measure, since mag will fluctuate between 1 and 0 as it moves
    # from perpendicular to normal to the plane
    # Filter both IMU and OMC arrays when either are close to normal

    keep_condition = (np.linalg.norm(IMU_proj_vecs, axis=1) > 0.5) & (np.linalg.norm(OMC_proj_vecs, axis=1) > 0.5)
    IMU_angles_filtered = np.where(keep_condition, IMU_angles, np.nan)
    OMC_angles_filtered = np.where(keep_condition, OMC_angles, np.nan)

    # Use these filtered arrays to calculate error
    error_arr = IMU_angles_filtered - OMC_angles_filtered

    # Remove nans for max and RMSE calcs
    error_arr_no_nans = error_arr[~np.isnan(error_arr)]

    # Get the maximum error
    max_error = np.amax(error_arr_no_nans)

    # Calculate RMSE across the time-varying error
    RMSE = find_RMSE_of_error_array(error_arr_no_nans)

    # Calculate Pearson correlation coefficient
    IMU_angles_filtered_no_nans = IMU_angles_filtered[~np.isnan(IMU_angles_filtered)]# Remove nans to calculate pearson
    OMC_angles_filtered_no_nans = OMC_angles_filtered[~np.isnan(OMC_angles_filtered)]# Remove nans to calculate pearson
    R = pearsonr(IMU_angles_filtered_no_nans, OMC_angles_filtered_no_nans)[0]

    return RMSE, R, error_arr, max_error, IMU_angles_filtered, OMC_angles_filtered





    #TODO: Change frontal global axis to -X

joint12_axis_dict = {'transverse': {'local_axis': 'z', 'global_plane': 'XZ', 'global_axis': 'Z'},
                     'frontal': {'local_axis': 'z', 'global_plane': 'XY', 'global_axis': 'Y'},
                     'sagittal': {'local_axis': 'y', 'global_plane': 'YZ', 'global_axis': 'YZ'}}

# Instantiate dict of plotting info for each plane
plotting_dict = {'transverse':
                     {'IMU_angles': [],
                      'OMC_angles': [],
                      'IMU_angles_filtered': [],
                      'OMC_angles_filtered': [],
                      'error_arr': [],
                      'RMSE': [],
                      'max_error': []},
                 'frontal':
                     {'IMU_angles': [],
                      'OMC_angles': [],
                      'IMU_angles_filtered': [],
                      'OMC_angles_filtered': [],
                      'error_arr': [],
                      'RMSE': [],
                      'max_error': []},
                 'sagittal':
                     {'IMU_angles': [],
                      'OMC_angles': [],
                      'IMU_angles_filtered': [],
                      'OMC_angles_filtered': [],
                      'error_arr': [],
                      'RMSE': [],
                      'max_error': []},
                 }

for key in joint12_axis_dict.keys():
    local_axis = joint12_axis_dict[key]['local_axis']
    global_axis = joint12_axis_dict[key]['global_axis']
    global_plane = joint12_axis_dict[key]['global_plane']

    IMU_proj_vecs = get_proj_vec_from_joint_rot_mat(IMU_joint12, local_axis, global_plane)
    OMC_proj_vecs = get_proj_vec_from_joint_rot_mat(OMC_joint12, local_axis, global_plane)
    IMU_angles = get_proj_vec_angle(IMU_proj_vecs, global_axis, global_plane)
    OMC_angles = get_proj_vec_angle(OMC_proj_vecs, global_axis, global_plane)
    RMSE, R, error_arr, max_error, IMU_angles_filtered, OMC_angles_filtered = \
        get_error_arrays_and_stats(IMU_proj_vecs, OMC_proj_vecs, IMU_angles, OMC_angles)

    # Add values to dict for plotting
    for key2 in plotting_dict[key].keys():
        plotting_dict[key][key2].append(eval(key2))


plot_vec_angles_error(plotting_dict, start_time, end_time, accuracy_results_folder, joint_of_interest='Joint12')






# IMU_joint12_array_of_vecs = get_proj_vec_from_joint_rot_mat(IMU_joint12, local_axis='z', global_plane='XZ')
#
# IMU_joint12_array_of_angles = get_proj_vec_angle(IMU_joint12_array_of_vecs, global_axis='Z', global_plane='XZ')
#
#




# # Define a function which takes a joint rotation matrix, and returns time-varying angles of projected vectors on the
# # X, Y and Z planes of the proximal frame
# def get_vector_angle_proj(joint_R):
#
#     # Instantiate angle and euler arrays
#     N = len(joint_R)
#     angles_transverse = np.zeros((N))
#     angles_frontal = np.zeros((N))
#     angles_sagittal = np.zeros((N))
#     transverse_keep_conditions = np.zeros((N))
#     frontal_keep_conditions = np.zeros((N))
#     sagittal_keep_conditions = np.zeros((N))
#
#     for row in range(N):
#
#         # Distal frame matrix and axes:
#         mat = joint_R[row].as_matrix()
#         x_axis = mat[:, 0]
#         y_axis = mat[:, 1]
#         z_axis = mat[:, 2]
#
#         # Calculate the angle between selected axis of the distal frame, relative to
#         # an axis of the proximal frame, projected onto a plane of the proximal frame
#
#         # Angle in transverse plane - z_axis relative to Z on the XZ plane
#         z_axis_on_XZ = [z_axis[0], z_axis[2]]
#         Z_on_XZ = [0, 1]
#         angle_transverse = angle_between_two_2D_vecs(z_axis_on_XZ, Z_on_XZ)
#         # Is axis sufficiently perpendicular to plane of interest?
#         transverse_keep_condition = (np.linalg.norm(z_axis_on_XZ) > 0.5)
#
#         # Angle in frontal plane - y_axis relative to -X on the XY plane
#         y_axis_on_XY = [y_axis[0], y_axis[1]]
#         negX_on_XY = [-1, 0]
#         angle_frontal = angle_between_two_2D_vecs(y_axis_on_XY, negX_on_XY)
#         # Is axis sufficiently perpendicular to plane of interest?
#         frontal_keep_condition = (np.linalg.norm(y_axis_on_XY) > 0.5)
#
#         # Angle in sagittal plane - y_axis relative to Z on the YZ plane
#         y_axis_on_YZ = [y_axis[1], y_axis[2]]
#         Z_on_YZ = [0, 1]
#         angle_sagittal = angle_between_two_2D_vecs(y_axis_on_YZ, Z_on_YZ)
#         # Is axis sufficiently perpendicular to plane of interest?
#         sagittal_keep_condition = (np.linalg.norm(y_axis_on_YZ) > 0.5)
#
#         # Append results to arrays
#         angles_transverse[row] = angle_transverse
#         angles_frontal[row] = angle_frontal
#         angles_sagittal[row] = angle_sagittal
#         transverse_keep_conditions[row] = transverse_keep_condition
#         frontal_keep_conditions[row] = frontal_keep_condition
#         sagittal_keep_conditions[row] = sagittal_keep_condition
#
#
#     return angles_transverse, angles_frontal, angles_sagittal, transverse_keep_conditions, frontal_keep_conditions, sagittal_keep_conditions
#
#
# IMU_joint12_angles_transverse, IMU_joint12_angles_frontal, IMU_joint12_angles_sagittal, \
#     IMU_transverse_keep_conditions, IMU_frontal_keep_conditions, IMU_sagittal_keep_conditions = get_vector_angle_proj(IMU_joint12)
# OMC_joint12_angles_transverse, OMC_joint12_angles_frontal, OMC_joint12_angles_sagittal, \
#     OMC_transverse_keep_conditions, OMC_frontal_keep_conditions, OMC_sagittal_keep_conditions = get_vector_angle_proj(OMC_joint12)
#
#
# # Get the projected angles difference
# joint12_transverse_diff = IMU_joint12_angles_transverse - OMC_joint12_angles_transverse
#
#
# # Plot the project angles and their differences
# RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3 = \
#     plot_compare_vectors(OMC_joint12_angles_transverse, OMC_joint12_angles_frontal, OMC_joint12_angles_sagittal,
#                          IMU_joint12_angles_transverse, IMU_joint12_angles_frontal, IMU_joint12_angles_sagittal,
#                          IMU_transverse_keep_conditions, IMU_frontal_keep_conditions, IMU_sagittal_keep_conditions,
#                          OMC_transverse_keep_conditions, OMC_frontal_keep_conditions, OMC_sagittal_keep_conditions,
#                          start_time, end_time, accuracy_results_folder, joint_of_interest='Joint12')
#
#
#

# Get the angle between projected vectors describing error in the transverse plane, sagittal plane, and frontal plane
# Will need to build in discard constraints or input timings of movement of interest.
# Joint_1 vector_angle_1 IMU
# Joint_1 vector_angle_2 IMU
# Joint_1 vector_angle_3 IMU
# Joint_2 vector_angle_1 IMU
# Joint_2 vector_angle_2 IMU
# Joint_2 vector_angle_3 IMU
# '' OMC
# Joint_1 vector_angle_1_difference


# Get the RMSE values for each error metric
# Joint1_geod_dist_RMSE
# Joint_1 vector_angle_1_RMSE
# Joint_1 vector_angle_2_RMSE
# Joint_1 vector_angle_3_RMSE
# Joint2_geod_dist_RMSE
# Joint_2 vector_angle_1_RMSE
# Joint_2 vector_angle_2_RMSE
# Joint_2 vector_angle_3_RMSE

