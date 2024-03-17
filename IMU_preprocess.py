# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file
# It also does an initial comparison of the raw orientation data from the 'real' and 'perfect' IMUs
# Output is a .csv with RMSE values for each IMU, and a .png plot
import numpy as np
import pandas as pd

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from scipy.signal import savgol_filter


    # TODO: use os.join(x,y) instead of adding strings
    # TODO: Look at OMC data and choose calibration pose times

""" SETTINGS """

# Quick Settings
subject_code = 'P2'
trial_name_dict = {'CP': {'N_self': 1, 'Alt_self': 2}, 'JA_Slow': {'N_self': 1, 'Alt_self': 2},
                   'JA_Fast': {'N_self': 1, 'Alt_self': 2}, 'ROM': {'N_self': 1, 'Alt_self': 2},
                   'ADL': {'N_self': 1, 'Alt_self': 2}}     # Looking at OMC data, input time values next to each type of pose
IMU_type_dict = {'IMU': ' - Report2 - IMU_Quats.txt', 'Cluster': ' - Report3 - Cluster_Quats.txt',
                 'Stylus': ' - Report4 - Cluster_Quats_Stylus_CFs.txt'}     # Edit this depending on what data you want to look at
sample_rate = 100
static_time = 1    # Input first known static time to use as reference for change in orientation error

# Required Files in Folder
template_file = "APDM_template_4S.csv"
APDM_settings_file = "APDMDataConverter_Settings.xml"

# Specify some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
raw_data_dir = os.path.join(parent_dir, 'RawData')

# Create a new results directory
results_dir = parent_dir + "\Preprocessed_Data"
if os.path.exists(results_dir) == False:
    os.mkdir(results_dir)

osim.Logger.setLevelString("Off")


""" MAIN """

# Function to extract quaternion orientation data from .txt file and save as .sto file
def write_movements_and_calibration_stos(file_path, cal_pose_time_dict, IMU_type, trial_results_dir):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # Write data to APDM format .csv
    file_tag = IMU_type + '_Quats_all'
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, trial_results_dir, file_tag)
    # Write data to .sto using OpenSim APDM converter tool
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=trial_results_dir + "\\" + file_tag + ".csv",
                         output_file_name=trial_results_dir + "\\" + file_tag + ".sto")

    # Iterate through list of calibration poses and associated times to create separate .sto files
    for pose_name in cal_pose_time_dict:

        cal_pose_time = cal_pose_time_dict[pose_name]   # The time at which to extract the data

        # Extract one row based on time of calibration pose
        IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
        IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
        IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)

        # Write data to APDM format .csv
        file_tag = IMU_type + '_Quats_' + str(pose_name) + '_' + str(cal_pose_time) + 's'
        write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, trial_results_dir, file_tag)
        # Write data to .sto using OpenSim APDM converter tool
        APDM_2_sto_Converter(APDM_settings_file, input_file_name=trial_results_dir + "\\" + file_tag + ".csv",
                             output_file_name=trial_results_dir + "\\" + file_tag + ".sto")

    return IMU1_df, IMU2_df, IMU3_df


# Iterating through each trial, for each type of quaternion data (real IMU or 'perfect' cluster-based quaternions),
# apply the function above to create orientation dataframes and write data to .sto files for full time-frame,
# and for each moment in time specified in the trial_name_dict

for trial_name in trial_name_dict:

    cal_pose_time_dict = trial_name_dict[trial_name]
    # raw_data_file = x|x

    # Create a new results directory
    trial_results_dir = os.path.join(results_dir, trial_name)
    if os.path.exists(trial_results_dir) == False:
        os.mkdir(trial_results_dir)

    for IMU_key in IMU_type_dict:

        raw_data_file = subject_code + '_' + trial_name + IMU_type_dict[IMU_key]
        raw_data_file_path = os.path.join(raw_data_dir, raw_data_file)
        IMU1_df, IMU2_df, IMU3_df = write_movements_and_calibration_stos(raw_data_file_path, cal_pose_time_dict, IMU_key, trial_results_dir)




# # Some code if you only want to read in two files
# input_file_Perfect = "P2_JA_Slow - Report3 - Cluster_Quats.txt"     # Name of the txt file with perfect IMU data
# input_file_Real = "P2_JA_Slow - Report2 - IMU_Quats.txt"     # Name of the txt file with real IMU data
# file_path_Perfect = os.path.join(raw_data_dir, input_file_Perfect)
# file_path_Real = os.path.join(raw_data_dir, input_file_Real)
# cal_pose_time_dict = {"Pose2_assisted": 26}  # List of pose times for calibration
# IMU1_df_Perfect, IMU2_df_Perfect, IMU3_df_Perfect = read_data_frame_from_file(file_path_Perfect)
# IMU1_df_Real, IMU2_df_Real, IMU3_df_Real = read_data_frame_from_file(file_path_Real)
#



