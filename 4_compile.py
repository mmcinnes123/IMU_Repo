# Script to compile results from .csv files
# Input: all the labels etc used to find each trail's results csv file
# Output: a single csv called 'AllResults_forR.csv'

import numpy as np
import pandas as pd
import os


# Quick Settings
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
list_of_subjects = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
IMU_type_list = ['Real']
trial_name = 'JA_Slow'
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
                        all_data = pd.concat([all_data, new_row], ignore_index=True)
                else:
                    print(f'File for {calibration_name}, {IMU_type} IMUs, {trial_name}, {subject_code} does not exist.')

out_file_dir = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\Results', 'Main_Results.csv')
all_data.to_csv(out_file_dir)

