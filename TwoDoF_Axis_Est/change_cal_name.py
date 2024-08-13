# A script to change file names when you want to change the name of a certain calibration method

import os
from os.path import join
import shutil


def get_file_names(parent_dir, subject_code, IMU_type, trial_name, org_cal_name, cal_name):

    # Files/folders to change
    comparison_csv = join(parent_dir, subject_code, IMU_type, 'IMU_IK_results_' + org_cal_name, trial_name, 'Comparison_' + subject_code + '_' + org_cal_name + '_' + trial_name, subject_code + '_' + cal_name + '_' + trial_name + '_Final_RMSEs.csv')
    comparison_dir = join(parent_dir, subject_code, IMU_type, 'IMU_IK_results_' + org_cal_name, trial_name, 'Comparison_' + subject_code + '_' + cal_name + '_' + trial_name)
    IK_results_dir = join(parent_dir, subject_code, IMU_type, 'IMU_IK_results_' + cal_name)
    calibrated_models_dir = join(parent_dir, subject_code, IMU_type, 'Calibrated_Models', cal_name)

    file_names = [comparison_csv, comparison_dir, IK_results_dir, calibrated_models_dir]

    return file_names


def change_cal_name(org_name, new_name, IMU_type, trial_name, subject_code_list):

    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'

    for subject_code in subject_code_list:

        org_file_names = get_file_names(parent_dir, subject_code, IMU_type, trial_name, org_name, org_name)
        new_file_names = get_file_names(parent_dir, subject_code, IMU_type, trial_name, org_name, new_name)

        for file in range(len(org_file_names)):

            if os.path.exists(org_file_names[file]):
                os.rename(org_file_names[file], new_file_names[file])
                print('Renamed: ', org_file_names[file])
            else:
                print(org_file_names[file], 'does not exist')


def delete_cal_files(org_name, IMU_type, trial_name, subject_code_list):

    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'

    for subject_code in subject_code_list:

        org_file_names = get_file_names(parent_dir, subject_code, IMU_type, trial_name, org_name, org_name)

        if os.path.exists(org_file_names[0]):
            os.remove(org_file_names[0])
        if os.path.exists(org_file_names[1]):
            shutil.rmtree(org_file_names[1])
        if os.path.exists(org_file_names[2]):
            shutil.rmtree(org_file_names[2])
        if os.path.exists(org_file_names[3]):
            shutil.rmtree(org_file_names[3])




subject_code_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

org_name = 'METHOD_4d_alt'
new_name = 'METHOD_7_ADL_both'
change_cal_name(org_name, new_name, 'Perfect', 'JA_Slow', subject_code_list)

# delete_cal_files('METHOD_4b', 'Perfect', 'JA_Slow', subject_code_list)
