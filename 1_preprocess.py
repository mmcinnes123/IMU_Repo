# This script preprocess IMU data, ready for use in OpenSim.
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file
# It also uses a dictionary of specified times to create .sto files for each moment a calibration pose was performed
# It also creates these outputs for multiple verions of the IMU data (e.g. 'real' and 'perfect' IMUs)

from constants import APDM_settings_file, APDM_template_file, sample_rate
from helpers_preprocess import read_data_frame_from_file
from helpers_preprocess import write_to_APDM
from helpers_preprocess import APDM_2_sto_Converter
from helpers_preprocess import extract_cal_row

import os
import ast
import opensim as osim


""" SETTINGS """

# Quick Settings
subject_code = 'P4'
# Looking at OMC data, input time values next to each type of pose
new_trial_name_dict = {'CP': {'N_self': 8, 'Alt_self': 21, 'N_asst': 13, 'Alt_asst': 21, 'Alt2_self': 29}, 'JA_Slow': {'N_self': 10, 'Alt_self': 14, 'Alt2_self': 106}, 'JA_Fast': {'N_self': 7, 'Alt_self': 10}, 'ROM': {'N_self': 5, 'Alt_self': 8}, 'ADL': {'N_self': 5, 'Alt_self': 10}}
save_new_dict = False    # Whether to write trial_name_dict above into the text file
IMU_type_dict = {'Real': ' - Report2 - IMU_Quats.txt', 'Perfect': ' - Report3 - Cluster_Quats.txt'}     # Edit this depending on what data you want to look at

# Specify some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
raw_data_dir = os.path.join(parent_dir, 'RawData')

# Create a new results directory
sto_files_dir = os.path.join(parent_dir, 'Preprocessed_Data')
os.makedirs(sto_files_dir, exist_ok=True)

# Repress opensim logging
osim.Logger.setLevelString("Off")


""" MAIN """


# Save trial name and times dict to .txt
if save_new_dict == True:
    file_obj = open(parent_dir + '\\' + subject_code + '_cal_pose_dict.txt', 'w')
    file_obj.write(str(new_trial_name_dict))
    file_obj.close()
    trial_name_dict = new_trial_name_dict
else:
    # Read the existing txt file to get the dict
    file_obj = open(parent_dir + '\\' + subject_code + '_cal_pose_dict.txt', 'r')
    content = file_obj.read()
    trial_name_dict = ast.literal_eval(content)
    file_obj.close()


# Function to extract quaternion orientation data from .txt file and save as .sto file
def write_movements_and_calibration_stos(file_path, cal_pose_time_dict, IMU_type, trial_results_dir):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # Write data to APDM format .csv
    file_tag = IMU_type + '_Quats_all'
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, APDM_template_file, trial_results_dir, file_tag)
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
        file_tag = IMU_type + '_Quats_' + str(pose_name)
        write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, APDM_template_file, trial_results_dir, file_tag)
        # Write data to .sto using OpenSim APDM converter tool
        APDM_2_sto_Converter(APDM_settings_file, input_file_name=trial_results_dir + "\\" + file_tag + ".csv",
                             output_file_name=trial_results_dir + "\\" + file_tag + ".sto")



# Iterating through each trial, for each type of quaternion data (real IMU or 'perfect' cluster-based quaternions),
# apply the function above to create orientation dataframes and write data to .sto files for full time-frame,
# and for each moment in time specified in the trial_name_dict

for trial_name in trial_name_dict:

    cal_pose_time_dict = trial_name_dict[trial_name]

    # Create a new results directory
    trial_results_dir = os.path.join(sto_files_dir, trial_name)
    os.makedirs(trial_results_dir, exist_ok=True)

    for IMU_key in IMU_type_dict:

        raw_data_file = subject_code + '_' + trial_name + IMU_type_dict[IMU_key]
        raw_data_file_path = os.path.join(raw_data_dir, raw_data_file)
        write_movements_and_calibration_stos(raw_data_file_path, cal_pose_time_dict, IMU_key, trial_results_dir)



