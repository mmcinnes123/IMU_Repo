
# This script calculates the error in IMU orientation estimates relative
# to a marker-based coordinate frame which is rigidly attached.
# It reads in angular velocity data, calculates the misalignment between global and local frames,
# applies the misalignment, then compares orientations.
# The data used is each IMU/marker assembly (Thorax, Humerus, then Forearm)
# being slowly rotated around each of its local axes in turn.
# The output is the local and global misalignments (printed to Run),
# and a .png graph of the time-varying orientation errors, with RMSE values.

from helpers import read_angvel_data_from_file
from helpers import get_misalign_R_from_angvels
from helpers import preprocess_angvels

import os
import numpy as np

""" SETTINGS """

# Quick Settings
# subject_code = 'P3'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Frame Calibration Verification'
sample_rate = 100
max_gap = 0.5     # In [s], the maximum allowable gap in data to be filled with SLERP
cut_off = 20       # Minimum angular velocity magnitude used to calculate misalignment
delay = 8           # Shift used to better match IMU and cluster ang vel data

# Timings - input start and end time of when each assembly started and stopped rotating
Thorax_start_time = 79
Thorax_end_time = 107
Humerus_start_time = 46
Humerus_end_time = 71
Forearm_start_time = 10
Forearm_end_time = 37

# Define the file names you want to take the data from
global_input_file = 'CAL_offset_test - Report5 - Global_Ang_Vels.txt'
local_input_file = 'CAL_offset_test - Report6 - Local_Ang_Vels.txt'

# Define file paths
raw_data_dir = os.path.join(parent_dir, 'RawData')
global_input_file_path = os.path.join(raw_data_dir, global_input_file)
local_input_file_path = os.path.join(raw_data_dir, local_input_file)


""" GET GLOBAL MISALIGNMENT """

# Read in global angular velocity data
Thorax_IMU_df, Thorax_Cluster_df, Humerus_IMU_df, Humerus_Cluster_df, Forearm_IMU_df, Forearm_Cluster_df = \
    read_angvel_data_from_file(global_input_file_path)

# Use the functions above to calculate the global misalignment based on the angular velocity vectors
Thorax_IMU_arr, Thorax_cluster_arr = preprocess_angvels(Thorax_IMU_df, Thorax_Cluster_df, Thorax_start_time,
                                                        Thorax_end_time, sample_rate, cut_off, delay)
Humerus_IMU_arr, Humerus_cluster_arr = preprocess_angvels(Humerus_IMU_df, Humerus_Cluster_df, Humerus_start_time,
                                                          Humerus_end_time, sample_rate, cut_off, delay)
Forearm_IMU_arr, Forearm_cluster_arr = preprocess_angvels(Forearm_IMU_df, Forearm_Cluster_df, Forearm_start_time,
                                                          Forearm_end_time, sample_rate, cut_off, delay)

# Use scipy's 'align_vectors' to solve for misalignment between global frames, according to each IMU
print('\nGlobal Misalignment (Thorax IMU):')
global_misalignment_thorax_IMU = get_misalign_R_from_angvels(Thorax_cluster_arr, Thorax_IMU_arr)
print('\nGlobal Misalignment (Humerus IMU):')
global_misalignment_humerus_IMU = get_misalign_R_from_angvels(Humerus_cluster_arr, Humerus_IMU_arr)
print('\nGlobal Misalignment (Forearm IMU):')
global_misalignment_forearm_IMU = get_misalign_R_from_angvels(Forearm_cluster_arr, Forearm_IMU_arr)

# Using a concatenation of all three IMUs, solve for the average misalignment
all_real_ang_vels = np.concatenate((Thorax_IMU_arr, Humerus_IMU_arr, Forearm_IMU_arr), axis=0)
all_perfect_ang_vels = np.concatenate((Thorax_cluster_arr, Humerus_cluster_arr, Forearm_cluster_arr), axis=0)
print('\nAverage Global Misalignment:')
average_global_misalignment = get_misalign_R_from_angvels(all_perfect_ang_vels, all_real_ang_vels)

# Let the global misalignment be defined by the thorax IMU
global_misalignment = global_misalignment_thorax_IMU


""" GET LOCAL MISALIGNMENT """

# Read in local angular velocity data
Thorax_IMU_df, Thorax_Cluster_df, Humerus_IMU_df, Humerus_Cluster_df, Forearm_IMU_df, Forearm_Cluster_df = \
    read_angvel_data_from_file(local_input_file_path)

# Use the functions above to calculate the local misalignment based on the angular velocity vectors
Thorax_IMU_arr, Thorax_cluster_arr = preprocess_angvels(Thorax_IMU_df, Thorax_Cluster_df, Thorax_start_time,
                                                               Thorax_end_time, sample_rate, cut_off, delay)
Humerus_IMU_arr, Humerus_cluster_arr = preprocess_angvels(Humerus_IMU_df, Humerus_Cluster_df, Humerus_start_time,
                                                          Humerus_end_time, sample_rate, cut_off, delay)
Forearm_IMU_arr, Forearm_cluster_arr = preprocess_angvels(Forearm_IMU_df, Forearm_Cluster_df, Forearm_start_time,
                                                          Forearm_end_time, sample_rate, cut_off, delay)

# Use scipy's 'align_vectors' to solve for misalignment between local frames, according to each IMU
print('\nLocal Misalignment Thorax:')
Thorax_local_misalignment = get_misalign_R_from_angvels(Thorax_cluster_arr, Thorax_IMU_arr)
print('\nLocal Misalignment Humerus:')
Humerus_local_misalignment = get_misalign_R_from_angvels(Humerus_cluster_arr, Humerus_IMU_arr)
print('\nLocal Misalignment Forearm:')
Forearm_local_misalignment = get_misalign_R_from_angvels(Forearm_cluster_arr, Forearm_IMU_arr)