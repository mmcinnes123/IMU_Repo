# Script to compile results from .csv files
# Input: all the labels etc used to find each trail's results csv file
# Output: a single csv called 'AllResults_forR.csv'

import numpy as np
import pandas as pd
import os


# Quick Settings
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
list_of_subjects = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6']
IMU_type_list = ['Perfect', 'Real']
trial_name = 'JA_Slow'
calibration_name_dict = {'OSIM_N_self': None,
                         'OSIM_Alt_self': None,
                         'ALL_MANUAL': None,
                         'METHOD_1_Alt_self': None,
                         'METHOD_2_Alt_self': None,
                         'METHOD_3': None,
                         }




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

            # Read RMSEs from the CSV file
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

all_data.to_csv(os.path.join(directory, 'R Analysis', 'AllResults_forR.csv'))

