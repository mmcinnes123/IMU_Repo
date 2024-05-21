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
from helpers_preprocess import write_movements_and_calibration_stos

import os
import ast
import opensim as osim


""" SETTINGS """

# Quick Settings
subject_code = 'P5'
# Looking at OMC data, input time values next to each type of pose
new_trial_name_dict = {'CP': {'N_self': 8, 'Alt_self': 21, 'N_asst': 13, 'Alt_asst': 21, 'Alt2_self': 29},
                       'JA_Slow': {'N_self': 10, 'Alt_self': 14, 'Alt2_self': 106},
                       'JA_Fast': {'N_self': 7, 'Alt_self': 10},
                       'ROM': {'N_self': 5, 'Alt_self': 8},
                       'ADL': {'N_self': 5, 'Alt_self': 10}}
save_new_dict = True    # Whether to write trial_name_dict above into the text file
IMU_type_dict = {'Real': ' - Report2 - IMU_Quats.txt', 'Perfect': ' - Report3 - Cluster_Quats.txt'}     # Edit this depending on what data you want to look at

# Specify some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
raw_data_dir = os.path.join(parent_dir, 'RawData')
sto_files_dir = os.path.join(parent_dir, 'Preprocessed_Data')

# Create a new results directory
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


# For each trial in trial_name_dict, and each IMU type, move the data from the .txt file into an .sto file
for trial_name in trial_name_dict:

    cal_pose_time_dict = trial_name_dict[trial_name]

    # Create a new results directory
    trial_results_dir = os.path.join(sto_files_dir, trial_name)
    os.makedirs(trial_results_dir, exist_ok=True)

    for IMU_key in IMU_type_dict:

        raw_data_file = subject_code + '_' + trial_name + IMU_type_dict[IMU_key]
        raw_data_file_path = os.path.join(raw_data_dir, raw_data_file)

        # For the whole trial, and for each time_stamp in cal_pose_time_dict, create an .sto file from the .txt file
        write_movements_and_calibration_stos(raw_data_file_path, cal_pose_time_dict, IMU_key, trial_results_dir)



