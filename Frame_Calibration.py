# This script calculates the error in IMU orientation estimates relative
# to a marker-based coordinate frame which is rigidly attached.
# It reads in angular velocity data, calculates the misalignment between global and local frames,
# applies the misalignment, then compares orientations.
# The data used is each IMU/marker assembly (Thorax, Humerus, then Forearm)
# being slowly rotated around each of its local axes in turn.
# The output is the local and global misalignments (printed to Run),
# and a .png graph of the time-varying orientation errors, with RMSE values.

from functions import *
import os

# TODO: Rewrite this whole script so there's one function, rather than a million lines of code


""" SETTINGS """

# Quick Settings
subject_code = 'P3'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
sample_rate = 100
max_gap = 0.5     # In [s], the maximum allowable gap in data to be filled with SLERP
cut_off = 20       # Minimum angular velocity magnitude used to calculate misalignment
delay = 8           # Shift used to better match IMU and cluster ang vel data

Thorax_start_time = 4
Thorax_end_time = 30
Humerus_start_time = 49
Humerus_end_time = 68
Forearm_start_time = 76
Forearm_end_time = 97

global_input_file = 'P3_CAL_copy - Report5 - Global_Ang_Vels.txt'
local_input_file = 'P3_CAL_copy - Report6 - Local_Ang_Vels.txt'
IMU_quats_file = 'P3_CAL_copy - Report7 - IMU_Quats_filtered.txt'
Cluster_quats_file = 'P3_CAL_copy - Report8 - Cluster_Quats_filtered.txt'

# Define file paths
raw_data_dir = os.path.join(parent_dir, 'RawData')
global_input_file_path = os.path.join(raw_data_dir, global_input_file)
local_input_file_path = os.path.join(raw_data_dir, local_input_file)
IMU_quats_file_path = os.path.join(raw_data_dir, IMU_quats_file)
Cluster_quats_file_path = os.path.join(raw_data_dir, Cluster_quats_file)



""" MAIN """

# A function for reading 3D angular velocity vectors from .txt TMM report file
def read_angvel_data_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    Thorax_Stylus_Cluster_df = df.filter(["Stylus_Cluster_Thorax_X", "Stylus_Cluster_Thorax_Y", "Stylus_Cluster_Thorax_Z"], axis=1)
    Thorax_Cluster_df = df.filter(["Cluster_Thorax_X", "Cluster_Thorax_Y", "Cluster_Thorax_Z"], axis=1)
    Thorax_IMU_df = df.filter(["IMU1_X", "IMU1_Y", "IMU1_Z"], axis=1)
    Humerus_Stylus_Cluster_df = df.filter(["Stylus_Cluster_Humerus_X", "Stylus_Cluster_Humerus_Y", "Stylus_Cluster_Humerus_Z"], axis=1)
    Humerus_Cluster_df = df.filter(["Cluster_Humerus_X", "Cluster_Humerus_Y", "Cluster_Humerus_Z"], axis=1)
    Humerus_IMU_df = df.filter(["IMU2_X", "IMU2_Y", "IMU2_Z"], axis=1)
    Forearm_Stylus_Cluster_df = df.filter(["Stylus_Cluster_Forearm_X", "Stylus_Cluster_Forearm_Y", "Stylus_Cluster_Forearm_Z"], axis=1)
    Forearm_Cluster_df = df.filter(["Cluster_Forearm_X", "Cluster_Forearm_Y", "Cluster_Forearm_Z"], axis=1)
    Forearm_IMU_df = df.filter(["IMU3_X", "IMU3_Y", "IMU3_Z"], axis=1)
    return Thorax_IMU_df, Thorax_Cluster_df, Thorax_Stylus_Cluster_df, \
        Humerus_IMU_df, Humerus_Cluster_df, Humerus_Stylus_Cluster_df, \
        Forearm_IMU_df, Forearm_Cluster_df, Forearm_Stylus_Cluster_df

# A function for calculating the misalignment between two frames, based on vectors measured within those frames,
# which should theoretically be perfectly aligned.
def get_misalign_R_from_angvels(IMU_ang_vels_df, Cluster_ang_vels_df, start_time, end_time):

    ang_vel_arr_real = IMU_ang_vels_df.to_numpy()
    ang_vel_arr_perfect = Cluster_ang_vels_df.to_numpy()

    # Trim the array based on the start and end times specified:
    # A delay is used to shift the IMU data forward to better match the ang vel vectors (to account for IMUs SFA delay)
    ang_vel_arr_real = ang_vel_arr_real[((start_time*sample_rate)+delay):((end_time*sample_rate)+delay)]
    ang_vel_arr_perfect = ang_vel_arr_perfect[(start_time*sample_rate):(end_time*sample_rate)]

    # Filter the arrays based on the magnitude of the angular velocity so we only compare vectors with larger velocity
    indices_to_delete = np.concatenate((np.where(np.linalg.norm(ang_vel_arr_real, axis=1) < cut_off),
                                        np.where(np.linalg.norm(ang_vel_arr_perfect, axis=1) < cut_off)), axis=1)
    ang_vel_arr_real = np.delete(ang_vel_arr_real, indices_to_delete, axis=0)
    ang_vel_arr_perfect = np.delete(ang_vel_arr_perfect, indices_to_delete, axis=0)

    # Normalise the angular velocity vectors so we're only comparing directions (this only affects rssd output)
    for row in range(len(ang_vel_arr_real)):
        ang_vel_arr_real[row] = ang_vel_arr_real[row]/np.linalg.norm(ang_vel_arr_real[row])
        ang_vel_arr_perfect[row] = ang_vel_arr_perfect[row]/np.linalg.norm(ang_vel_arr_perfect[row])

    # Use scipy's 'align_vectors' to find the rotation which would best align the two sets of vectors
    a = ang_vel_arr_perfect
    b = ang_vel_arr_real
    local_misalignment, rssd, sens = R.align_vectors(a, b, return_sensitivity=True)
    print(f"As Quat: {local_misalignment.as_quat()}")
    print(f"As Euler (yxz) {local_misalignment.as_euler('yxz', degrees=True)}")
    print(f"RSSD: {rssd}")
    # print(sens)

    return local_misalignment


def preprocess_global_angvels(IMU_ang_vels_df, Cluster_ang_vels_df, start_time, end_time):

    ang_vel_arr_real = IMU_ang_vels_df.to_numpy()
    ang_vel_arr_perfect = Cluster_ang_vels_df.to_numpy()

    # Trim the array based on the start and end times specified:
    # A delay is used to shift the IMU data forward to better match the ang vel vectors (to account for IMUs SFA delay)
    ang_vel_arr_real = ang_vel_arr_real[((start_time*sample_rate)+delay):((end_time*sample_rate)+delay)]
    ang_vel_arr_perfect = ang_vel_arr_perfect[(start_time*sample_rate):(end_time*sample_rate)]

    # Filter the arrays based on the magnitude of the angular velocity so we only compare vectors with larger velocity
    indices_to_delete = np.concatenate((np.where(np.linalg.norm(ang_vel_arr_real, axis=1) < cut_off),
                                        np.where(np.linalg.norm(ang_vel_arr_perfect, axis=1) < cut_off)), axis=1)

    ang_vel_arr_real = np.delete(ang_vel_arr_real, indices_to_delete, axis=0)
    ang_vel_arr_perfect = np.delete(ang_vel_arr_perfect, indices_to_delete, axis=0)

    # Normalise the angular velocity vectors so we're only comparing directions (this only affects rssd output)
    for row in range(len(ang_vel_arr_real)):
        ang_vel_arr_real[row] = ang_vel_arr_real[row]/np.linalg.norm(ang_vel_arr_real[row])
        ang_vel_arr_perfect[row] = ang_vel_arr_perfect[row]/np.linalg.norm(ang_vel_arr_perfect[row])

    return ang_vel_arr_real, ang_vel_arr_perfect




# Use the functions above to calculate the global misalignment based on the angular velocity vectors

Thorax_IMU_df, Thorax_Cluster_df, Thorax_Stylus_Cluster_df, \
    Humerus_IMU_df, Humerus_Cluster_df, Humerus_Stylus_Cluster_df, \
    Forearm_IMU_df, Forearm_Cluster_df, Forearm_Stylus_Cluster_df = read_angvel_data_from_file(global_input_file_path)

Thorax_IMU_arr, Thorax_cluster_arr = preprocess_global_angvels(Thorax_IMU_df, Thorax_Cluster_df, Thorax_start_time,
                                                               Thorax_end_time)
Humerus_IMU_arr, Humerus_cluster_arr = preprocess_global_angvels(Humerus_IMU_df, Humerus_Cluster_df, Humerus_start_time,
                                                                 Humerus_end_time)
Forearm_IMU_arr, Forearm_cluster_arr = preprocess_global_angvels(Forearm_IMU_df, Forearm_Cluster_df, Forearm_start_time,
                                                                 Forearm_end_time)

# Use scipy's 'align_vectors' to solve for the average misalignment (this shouldn't actually apply to the local CFs)
a = np.concatenate((Thorax_IMU_arr, Humerus_IMU_arr, Forearm_IMU_arr), axis=0)
b = np.concatenate((Thorax_cluster_arr, Humerus_cluster_arr, Forearm_cluster_arr), axis=0)
rot, rssd, sens = R.align_vectors(a, b, return_sensitivity=True)
global_misalignment = rot
print('\nGlobal Misalignment:')
print(f"As Quat: {global_misalignment.as_quat()}")
print(f"As Euler (yxz) {rot.as_euler('yxz', degrees=True)}")
print(f"RSSD: {rssd}")
# print(sens)



# Use the functions above to calculate the local misalignment based on the angular velocity vectors

Thorax_IMU_df, Thorax_Cluster_df, Thorax_Stylus_Cluster_df, \
    Humerus_IMU_df, Humerus_Cluster_df, Humerus_Stylus_Cluster_df, \
    Forearm_IMU_df, Forearm_Cluster_df, Forearm_Stylus_Cluster_df = read_angvel_data_from_file(local_input_file_path)

print('\nLocal Misalignment Thorax:')
Thorax_local_misalignment = get_misalign_R_from_angvels(Thorax_IMU_df, Thorax_Cluster_df, Thorax_start_time, Thorax_end_time)
print('\nLocal Misalignment Humerus:')
Humerus_local_misalignment = get_misalign_R_from_angvels(Humerus_IMU_df, Humerus_Cluster_df, Humerus_start_time, Humerus_end_time)
print('\nLocal Misalignment Forearm:')
Forearm_local_misalignment = get_misalign_R_from_angvels(Forearm_IMU_df, Forearm_Cluster_df, Forearm_start_time, Forearm_end_time)


""" CALCULATING ORIENTATION ERROR """

# Read in the orientation data
Thorax_IMU_quats, Humerus_IMU_quats, Forearm_IMU_quats = read_data_frame_from_file(IMU_quats_file_path)
Thorax_Cluster_quats, Humerus_Cluster_quats, Forearm_Cluster_quats = read_data_frame_from_file(Cluster_quats_file_path)

# Use my own SLERP code to fill in gaps in quaternion data
Thorax_IMU_quats = fill_nan_gaps_in_quat_df(Thorax_IMU_quats, max_gap*sample_rate)
Humerus_IMU_quats = fill_nan_gaps_in_quat_df(Humerus_IMU_quats, max_gap*sample_rate)
Forearm_IMU_quats = fill_nan_gaps_in_quat_df(Forearm_IMU_quats, max_gap*sample_rate)
Thorax_Cluster_quats = fill_nan_gaps_in_quat_df(Thorax_Cluster_quats, max_gap*sample_rate)
Humerus_Cluster_quats = fill_nan_gaps_in_quat_df(Humerus_Cluster_quats, max_gap*sample_rate)
Forearm_Cluster_quats = fill_nan_gaps_in_quat_df(Forearm_Cluster_quats, max_gap*sample_rate)

# Create scipy arrays from the dfs
Thorax_IMU_R = R.from_quat(Thorax_IMU_quats.to_numpy()[:, [1, 2, 3, 0]])
Humerus_IMU_R = R.from_quat(Humerus_IMU_quats.to_numpy()[:, [1, 2, 3, 0]])
Forearm_IMU_R = R.from_quat(Forearm_IMU_quats.to_numpy()[:, [1, 2, 3, 0]])
Thorax_Cluster_R = R.from_quat(Thorax_Cluster_quats.to_numpy()[:, [1, 2, 3, 0]])
Humerus_Cluster_R = R.from_quat(Humerus_Cluster_quats.to_numpy()[:, [1, 2, 3, 0]])
Forearm_Cluster_R = R.from_quat(Forearm_Cluster_quats.to_numpy()[:, [1, 2, 3, 0]])

# Apply the global misalignment Rot to the IMU orientation data to bring the IMUs into the OMC frame
Thorax_IMU_globally_aligned = global_misalignment.inv() * Thorax_IMU_R
Humerus_IMU_globally_aligned = global_misalignment.inv() * Humerus_IMU_R
Forearm_IMU_globally_aligned = global_misalignment.inv() * Forearm_IMU_R

    # TODO: Be more confident that the equation below is correct way to apply the local alignment
# Apply the local misalignments to each cluter orientation data to better align them with the IMU casings
Thorax_Cluster_locally_aligned = Thorax_local_misalignment.inv() * Thorax_Cluster_R
Humerus_Cluster_locally_aligned = Humerus_local_misalignment.inv() * Humerus_Cluster_R
Forearm_Cluster_locally_aligned = Forearm_local_misalignment.inv() * Forearm_Cluster_R

# Now we can directly compare the orientation of IMU_globally_aligned, and Cluster_locally_aligned
Thorax_single_angle_diff = (Thorax_Cluster_locally_aligned.inv() * Thorax_IMU_globally_aligned).magnitude() * 180 / np.pi
Humerus_single_angle_diff = (Humerus_Cluster_locally_aligned.inv() * Humerus_IMU_globally_aligned).magnitude() * 180 / np.pi
Forearm_single_angle_diff = (Forearm_Cluster_locally_aligned.inv() * Forearm_IMU_globally_aligned).magnitude() * 180 / np.pi


# Calculate RMSE and plot the time-varying error
figure_results_dir = parent_dir
IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = \
    plot_compare_real_vs_perfect(Thorax_single_angle_diff, Humerus_single_angle_diff, Forearm_single_angle_diff, figure_results_dir)

# Write final RMSE values to a csv
final_RMSE_values_df = pd.DataFrame.from_dict(
    {"Thorax IMU orientation error:": IMU1_single_angle_RMSE,
     "Humerus IMU orientation error:": IMU2_single_angle_RMSE,
     "Forearm IMU orientation error:": IMU3_single_angle_RMSE}, orient='index')
final_RMSE_values_df.to_csv(parent_dir + "\\" + "Real_vs_Perfect_Ori_Errors.csv",
                            mode='w', encoding='utf-8', na_rep='nan')


""" USING OLD METHOD TO CALCULATE ERROR IN CHANGE IN ORIENTATION """
#
# # Calculate with my conventional way:
# time_index = 103 * sample_rate  # At time at which all the IMUs are relatively still
# Thorax_IMU_at_T = Thorax_IMU_R[time_index]
# Thorax_Cluster_at_T = Thorax_Cluster_R[time_index]
# Humerus_IMU_at_T = Humerus_IMU_R[time_index]
# Humerus_Cluster_at_T = Humerus_Cluster_R[time_index]
# Forearm_IMU_at_T = Forearm_IMU_R[time_index]
# Forearm_Cluster_at_T = Forearm_Cluster_R[time_index]
#
# # Get CHANGE in orientations, relative to orientation at specified static time
# ori_change_Thorax_IMU = Thorax_IMU_at_T.inv() * Thorax_IMU_R
# ori_change_Thorax_Cluster = Thorax_Cluster_at_T.inv() * Thorax_Cluster_R
# ori_change_Humerus_IMU = Humerus_IMU_at_T.inv() * Humerus_IMU_R
# ori_change_Humerus_Cluster = Humerus_Cluster_at_T.inv() * Humerus_Cluster_R
# ori_change_Forearm_IMU = Forearm_IMU_at_T.inv() * Forearm_IMU_R
# ori_change_Forearm_Cluster = Forearm_Cluster_at_T.inv() * Forearm_Cluster_R
#
# # Compare IMU vs Cluster change in orientations
# Thorax_single_angle_diff = (ori_change_Thorax_Cluster.inv() * ori_change_Thorax_IMU).magnitude() * 180 / np.pi
# Humerus_single_angle_diff = (ori_change_Humerus_Cluster.inv() * ori_change_Humerus_IMU).magnitude() * 180 / np.pi
# Forearm_single_angle_diff = (ori_change_Forearm_Cluster.inv() * ori_change_Forearm_IMU).magnitude() * 180 / np.pi
#
# figure_results_dir = raw_data_dir
# # Calculate RMSE and plot the time-varying error
# IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = \
#     plot_compare_real_vs_perfect(Thorax_single_angle_diff, Humerus_single_angle_diff, Forearm_single_angle_diff, figure_results_dir)
