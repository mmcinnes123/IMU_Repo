# Script to compile results from .csv files
# Input: all the labels etc used to find each trail's results csv file
# Output: a single csv called 'AllResults_forR.csv'

import numpy as np
import pandas as pd
import os
from constants import data_dir

# TODO: This should really be written so it reads the file already written and just adds new results

# Quick Settings
directory = data_dir
from_subject = 1
to_subject = 20
list_of_subjects = [f'P{str(i).zfill(3)}' for i in range(from_subject, (to_subject+1))]
IMU_type_list = ['Perfect', 'Real']
trial_name = 'JA_Slow'

# Calibration Name Dictionary
# --------------------------
# Key: Calibration type
# Value: None (populated during runtime)
#
# 1. OpenSim Default Calibrations:
#    - 'OSIM_N_self'
#    - 'OSIM_N_asst'
#    - 'OSIM_Alt_self'
#    - 'OSIM_Alt_asst'
#
# 2. Method 7 Variations:
#    - 'METHOD_7_ISO_5reps'
#    - 'METHOD_7_ISO_1rep'
#    - 'METHOD_7_ADL_both'

# Add all methods you want to compile results for
calibration_name_dict = {'OSIM_N_self': None,
                         'OSIM_N_asst': None,
                         'OSIM_Alt_self': None,
                         'OSIM_Alt_asst': None,
                         'METHOD_7_ISO_5reps': None,
                         'METHOD_7_ISO_1rep': None,
                         'METHOD_7_ADL_both': None
                         }

# folders_to_exclude = [r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P22\Real\IMU_IK_results_METHOD_4b\JA_Slow\Comparison_P22_METHOD_4b_JA_Slow']
folders_to_exclude = []

""" CREATING DATAFRAME FOR R"""

all_data = pd.DataFrame(columns=['Subject', 'Trial', 'CalibrationName', 'JA', 'RMSE', 'R', 'peakROM', 'troughROM'])


# Get the results for each calibration method
for IMU_type in IMU_type_list:

    for calibration_name in calibration_name_dict:

        for subject_code in list_of_subjects:

            # Define the file paths
            comparison_folder = 'Comparison_' + subject_code + '_' + calibration_name + '_' + trial_name
            file_name = subject_code + '_' + calibration_name + '_' + trial_name + '_Final_RMSEs.csv'
            file_path = os.path.join(directory, subject_code, IMU_type, 'IMU_IK_results_' + calibration_name, trial_name,
                                     comparison_folder, file_name)

            # Exclude this from the analysis for now
            if comparison_folder not in folders_to_exclude:

                # Read RMSEs from the CSV file
                if os.path.exists(file_path):

                    with open(file_path, 'r') as file:
                        Results_df = pd.read_csv(file, index_col=0, header=0, sep=',', dtype={1: np.float64, 2: np.float64})

                    for row in range(len(Results_df)):
                        new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_name],
                                                'CalibrationName': [calibration_name], 'IMU_type': [IMU_type],
                                                'JA': Results_df.index[row],
                                                'RMSE': Results_df.loc[Results_df.index[row], 'RMSE'],
                                                'R': Results_df.loc[Results_df.index[row], 'R'],
                                                'peakROM': Results_df.loc[Results_df.index[row], 'peakROM'],
                                                'troughROM': Results_df.loc[Results_df.index[row], 'troughROM']})
                        
                        # Check for NaN values only if JA doesn't start with 'thorax' and doesn't end with 'ori'
                        current_ja = str(Results_df.index[row])
                        if not (current_ja.startswith('thorax') or current_ja.endswith('ori')):
                            nan_columns = new_row.columns[new_row.isna().any()].tolist()
                            if nan_columns:
                                print(f"Warning: NaN values found in {subject_code}, {calibration_name}, {IMU_type}, {current_ja} for columns: {nan_columns}")
                        
                        all_data = pd.concat([all_data, new_row], ignore_index=True)
                else:
                    print(f'File for {calibration_name}, {IMU_type} IMUs, {trial_name}, {subject_code} does not exist.')

out_file_dir = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\Results', 'Main_Results.csv')
all_data.to_csv(out_file_dir)