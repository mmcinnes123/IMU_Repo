# This script preprocess IMU data, ready for use in OpenSim.
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file
# It also uses a dictionary of specified times to create .sto files for each moment a calibration pose was performed
# It also creates these outputs for multiple verions of the IMU data (e.g. 'real' and 'perfect' IMUs)

from helpers_preprocess import write_movements_and_calibration_stos
from helpers_preprocess import get_trial_pose_time_dict_from_file

from os.path import join
from os import makedirs
import opensim as osim
from tkinter.filedialog import askopenfilename, askdirectory


""" SETTINGS """

# Quick Settings
subject_code_list = [f'P{i}' for i in range(1, 23)]
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
IMU_type_dict = {'Real': ' - Report2 - IMU_Quats.txt', 'Perfect': ' - Report3 - Cluster_Quats.txt'}     # Edit this depending on what data you want to look at

# Repress opensim logging
osim.Logger.setLevelString("Off")


""" MAIN """

# For each subject, each trial in trial_name_dict, and each IMU type, move the data from the .txt file into an .sto file
for subject_code in subject_code_list:

    print('Creating preprocessed data (.stos) for subject: ', subject_code)

    # Read the existing event dict txt file to get the pose names and associated times
    trial_name_dict = get_trial_pose_time_dict_from_file(directory, subject_code)

    # Specify some file paths
    parent_dir = directory + '\\' + subject_code
    raw_data_dir = join(parent_dir, 'RawData')
    sto_files_dir = join(parent_dir, 'Preprocessed_Data')
    makedirs(sto_files_dir, exist_ok=True)

    for trial_name in trial_name_dict:

        print('for trial: ', trial_name)

        cal_pose_time_dict = trial_name_dict[trial_name]

        # Create a new results directory
        trial_results_dir = join(sto_files_dir, trial_name)
        makedirs(trial_results_dir, exist_ok=True)

        for IMU_key in IMU_type_dict:

            print('for IMU type: ', IMU_key)

            raw_data_file = subject_code + '_' + trial_name + IMU_type_dict[IMU_key]
            raw_data_file_path = join(raw_data_dir, raw_data_file)

            # For the whole trial, and for each time_stamp in cal_pose_time_dict, create an .sto file from the .txt file
            write_movements_and_calibration_stos(raw_data_file_path, cal_pose_time_dict, IMU_key, trial_results_dir)


""" TEST """

run_test = False
if run_test:

    IMU_key = 'Real'
    cal_pose_time_dict = {'N_self': 13, 'Alt_self': 16}
    raw_data_file_path = str(askopenfilename(title=' Choose the raw data .txt file with IMU/quat data ... '))
    trial_results_dir = str(askdirectory(title=' Choose the folder where you want to save the results ... '))
    write_movements_and_calibration_stos(raw_data_file_path, cal_pose_time_dict, IMU_key, trial_results_dir)