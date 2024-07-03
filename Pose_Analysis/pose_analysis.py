# This script analyses how successfully a subject performed each pose, relative to the 'perfect' default pose of the model

# Imports
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

# Create a template dict used for saving results
all_results_template = pd.DataFrame(columns=['Subject', 'Trial', 'Pose_code', 'Pose_type', 'Pose_time',
                                             'elbow_flexion', 'elbow_pronation',
                                             'HT_abd', 'HT_flex', 'HT_rot',
                                             'elbow_flexion_error', 'elbow_pronation_error',
                                             'HT_abd_error', 'HT_flex_error', 'HT_rot_error'])

""" MAIN """

write_results = False
if write_results:

    # Settings
    subject_code_list = [f'P{i}' for i in range(1, 23)]

    for subject_code in subject_code_list:

        # Instantiate a subject results dict
        subject_results = all_results_template

        # Define some file paths
        parent_dir = join(directory, subject_code)
        OMC_dir = join(parent_dir, 'OMC')

        # Get the dict which defines when each pose happened in each trial
        trial_pose_time_dict = get_trial_pose_time_dict_from_file(directory, subject_code)

        for trial_name in trial_pose_time_dict:

            # Extract the dict which is specific to that trial, with pose names and associated time stamps
            pose_time_dict = trial_pose_time_dict[trial_name]

            for pose_name, pose_time in pose_time_dict.items():

                if pose_name.startswith(('N', 'Alt')):

                    if pose_time == None:
                        print(f"No pose data for {subject_code}, {trial_name}, {pose_name}")
                        elbow_flexion, elbow_pronation = [np.nan], [np.nan]
                        HT_abd, HT_flex, HT_rot = [np.nan], [np.nan], [np.nan]

                    else:
                        # Find the directory with the IK results files
                        IK_results_dir = join(OMC_dir, trial_name + '_IK_Results')
                        mot_file = join(IK_results_dir, 'OMC_IK_results.mot')
                        analysis_sto_file = join(IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')

                        # Get some joint angles/coords from the mot file
                        elbow_flexion, elbow_pronation = get_coords_from_mot_file(mot_file, pose_time)

                        # Get the values for HT joint angles from the sto file
                        HT_abd, HT_flex, HT_rot = get_HT_angles_from_sto(analysis_sto_file, pose_time)

                    # Compile the pose JAs into one dict
                    pose_dict = {'elbow_flexion': elbow_flexion,
                                 'elbow_pronation': elbow_pronation,
                                 'HT_abd': HT_abd,
                                 'HT_flex': HT_flex,
                                 'HT_rot': HT_rot}

                    # Get the default model pose JAs, based on the pose_name
                    model_pose_dict = get_default_model_pose_dict(pose_name)

                    # Find the pose error (difference between performed pose, and model default pose) for each JA
                    error_dict = {JA + '_error': pose_dict[JA][0] - model_pose_dict[JA][0] for JA in model_pose_dict}

                    # Split the pose_name into a code, and whether it was assisted
                    pose_code, pose_type = split_pose_name(pose_name)

                    # Add the results from this pose into the results dataframe
                    meta_dict = {'Subject': [subject_code],
                                 'Trial': [trial_name],
                                 'Pose_code': [pose_code],
                                 'Pose_type': [pose_type],
                                 'Pose_time': [pose_time]}
                    new_df_row = pd.DataFrame({**meta_dict, **pose_dict, **error_dict})
                    subject_results = pd.concat([subject_results, new_df_row], ignore_index=True)

                    inspect_results = False
                    if inspect_results:
                        pd.set_option('display.width', None)
                        print(subject_results)

        # Save the results for each subject
        print(f'Writing subject {subject_code} results to .csv.')
        output_file_name = str(subject_code) + r"_Pose_Results.csv"
        subject_results.to_csv(join(parent_dir, output_file_name),
                           mode='w', encoding='utf-8', na_rep='nan')





""" COMPILE ALL RESULTS """

compile_results = True
if compile_results:

    # Settings
    subject_code_list = [f'P{i}' for i in range(1, 23)]

    # Make an empty dict, based on the template above
    all_results = all_results_template

    # Read in each subject's results .csv and compile into a 'final results' /csv
    for subject_code in subject_code_list:

        print(f"Reading pose results for subject: {subject_code}...")

        # Read in the dataframe from each subject's .csv file
        parent_dir = join(directory, subject_code)
        output_file_name = str(subject_code) + r"_Pose_Results.csv"
        with open(join(parent_dir, output_file_name), 'r') as file:
            subject_df = pd.read_csv(file, index_col=0, header=0, sep=',')

        all_results = pd.concat([all_results, subject_df], ignore_index=True)

    pd.set_option('display.width', None)

    # Save all results to folder for analyis in R
    all_results.to_csv(join(directory, 'R Analysis', 'R Pose', 'PoseResults_forR.csv'))
    print('Written all pose results to file in R Analysis')
