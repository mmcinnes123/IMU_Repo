# This script analyses how successfully a subject_code performed each pose, relative to the target (default)
# pose of the model

from pose_analysis_helpers import get_trial_pose_time_dict_from_file
from pose_analysis_helpers import get_HT_angles_from_sto
from pose_analysis_helpers import get_coords_from_mot_file
from pose_analysis_helpers import get_default_model_pose_dict
from pose_analysis_helpers import split_pose_name

import pandas as pd
from os.path import join
import numpy as np


# Set main directory
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'


""" INDIVIDUAL RESULTS """
# The following code reads the results of the OMC IK for each subject_code, extracts the joint angle values during each
# pose (using a pre-saved event dict as a reference for the time of each pose in each trial) and compares the
# measured joint angles with the target joint angles, then saves the value and the error to individual pose_results.csvs

write_results = False
if write_results:

    # Settings
    subject_code_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

    for subject_code in subject_code_list:

        # Initiate a subject_code results dict
        subject_results = pd.DataFrame()

        # Define some file paths
        parent_dir = join(directory, subject_code)
        OMC_dir = join(parent_dir, 'OMC')

        # Get the dict with info about times at which each pose happened, in each trial (created previously)
        trial_pose_time_dict = get_trial_pose_time_dict_from_file(directory, subject_code)

        for trial_name in trial_pose_time_dict:

            # Extract the dict which is specific to that trial, with pose names and associated time stamps
            pose_time_dict = trial_pose_time_dict[trial_name]

            for pose_name, pose_time in pose_time_dict.items():

                if pose_name.startswith(('N', 'Alt')):

                    # Split the pose_name into a code, and whether it was assisted
                    pose_code, pose_type = split_pose_name(pose_name)

                    if pose_time is not None:

                        # Find the directory with the IK results files
                        IK_results_dir = join(OMC_dir, trial_name + '_IK_Results')
                        mot_file = join(IK_results_dir, 'OMC_IK_results.mot')
                        analysis_sto_file = join(IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')

                        # Get some joint angles/coords from the mot file
                        elbow_flexion, elbow_pronation = get_coords_from_mot_file(mot_file, pose_time)

                        # Get the values for HT joint angles from the sto file
                        HT_abd, HT_flex, HT_rot = get_HT_angles_from_sto(analysis_sto_file, pose_time)

                    else:

                        print(f"No pose data for {subject_code}, {trial_name}, {pose_name}")
                        elbow_flexion, elbow_pronation = [np.nan], [np.nan]
                        HT_abd, HT_flex, HT_rot = [np.nan], [np.nan], [np.nan]

                    # Compile the pose JAs into one dict
                    pose_dict = {'elbow_flexion': elbow_flexion,
                                 'elbow_pronation': elbow_pronation,
                                 'HT_abd': HT_abd,
                                 'HT_flex': HT_flex,
                                 'HT_rot': HT_rot}

                    # Get the default model pose JAs, based on the pose_name
                    model_pose_dict = get_default_model_pose_dict(pose_name)

                    for JA in pose_dict:

                        # Get the difference between the actual pose and the target pose
                        JA_error = pose_dict[JA][0] - model_pose_dict[JA][0]

                        # Log the results
                        new_df_row = pd.DataFrame({'Subject': [subject_code],
                                                   'Trial': [trial_name],
                                                   'Pose_code': [pose_code],
                                                   'Pose_type': [pose_type],
                                                   'Pose_time': [pose_time],
                                                   'JA': [JA],
                                                   'Value': [pose_dict[JA][0]],
                                                   'Error': [JA_error]})
                        subject_results = pd.concat([subject_results, new_df_row], ignore_index=True)

                    inspect_results = False
                    if inspect_results:
                        pd.set_option('display.width', None)
                        print(subject_results)

        # Save the results for each subject_code
        print(f'Writing subject_code {subject_code} results to .csv.')
        output_file_name = str(subject_code) + r"_Pose_Results.csv"
        subject_results.to_csv(join(parent_dir, output_file_name),
                           mode='w', encoding='utf-8', na_rep='nan')


""" COMPILE ALL RESULTS """
# The following code reads each individual's pose results .csv files and compiles all results into one .csv

compile_results = True
if compile_results:

    # Settings
    subject_code_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

    # Make an empty dict, based on the template above
    all_results = pd.DataFrame()

    # Read in each subject_code's results .csv and compile into a 'final results' /csv
    for subject_code in subject_code_list:

        print(f"Reading pose results for subject_code: {subject_code}...")

        # Read in the dataframe from each subject_code's .csv file
        parent_dir = join(directory, subject_code)
        output_file_name = str(subject_code) + r"_Pose_Results.csv"
        with open(join(parent_dir, output_file_name), 'r') as file:
            subject_df = pd.read_csv(file, index_col=0, header=0, sep=',')

        all_results = pd.concat([all_results, subject_df], ignore_index=True)

    pd.set_option('display.width', None)

    # Save all results to folder for analysis in R
    results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\Results'
    all_results.to_csv(join(results_dir, 'Pose_Results.csv'))
    print('Written all pose results to file in R Analysis')
