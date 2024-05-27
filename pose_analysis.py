# This script analyses how successfully a subject performed each pose, relative to the 'perfect' default pose of the model
import pandas as pd

# Imports
from pose_analysis_helpers import get_trial_pose_time_dict_from_file
from pose_analysis_helpers import get_coords_from_mot
from pose_analysis_helpers import get_coord_at_time_t
from pose_analysis_helpers import get_HT_angles_from_sto
from pose_analysis_helpers import get_coords_from_mot_file
from pose_analysis_helpers import get_default_model_pose_dict

from os.path import join
import numpy as np

""" SETTINGS """
subject_code_list = ['P1']




""" MAIN """

all_results = pd.DataFrame(columns=['Trial', 'Pose', 'Pose_time', 'elbow_flexion', 'elbow_pronation', 'HT_abd', 'HT_flex', 'HT_rot',
                                    'elbow_flexion_error', 'elbow_pronation_error', 'HT_abd_error', 'HT_flex_error', 'HT_rot_error'])

for subject_code in subject_code_list:

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_dir = join(parent_dir, 'OMC')

    # Get the dict which defines when each pose happened in each trial
    trial_pose_time_dict = get_trial_pose_time_dict_from_file(parent_dir, subject_code)

    for trial_name in trial_pose_time_dict:

        # Extract the dict which is specific to that trial, with pose names and associated time stamps
        pose_time_dict = trial_pose_time_dict[trial_name]

        for pose_name, pose_time in pose_time_dict.items():

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

            # Add the results from this pose into the results dataframe
            meta_dict = {'Trial': [trial_name],
                         'Pose': [pose_name],
                         'Pose_time': [pose_time]}
            new_df_row = pd.DataFrame({**meta_dict, **pose_dict, **error_dict})
            all_results = pd.concat([all_results, new_df_row], ignore_index=True)





            # TODO: rejig these functions so they are grouped better
            # TODO: choose some print options so we can see specifics if we want to


pd.set_option('display.width', None)
print(all_results)


# TODO: Save results to file per person
# TODO: Compile all subject results - ready for visualisation in R - be sure to maintain bias