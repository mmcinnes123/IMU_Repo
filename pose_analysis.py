# This script analyses how successfully a subject performed each pose, relative to the 'perfect' default pose of the model
import pandas as pd

# Imports
from pose_analysis_helpers import get_trial_pose_time_dict_from_file
from pose_analysis_helpers import get_coords_from_mot
from pose_analysis_helpers import get_coord_at_time_t
from pose_analysis_helpers import get_HT_angles_from_sto

from os.path import join
import numpy as np

""" SETTINGS """
subject_code_list = ['P1']




""" MAIN """

all_results = pd.DataFrame(columns=['Trial', 'Pose', 'elbow_flexion', 'elbow_pronation', 'HT_abd', 'HT_flex', 'HT_rot'])

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

            # Find the directory with the IK results
            IK_results_dir = join(OMC_dir, trial_name + '_IK_Results')
            mot_file = join(IK_results_dir, 'OMC_IK_results.mot')
            analysis_sto_file = join(IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')

            # Read in the .mot coords file
            coords_table = get_coords_from_mot(mot_file)

            # Get the coords (elbow flexion, pronation) at the specified pose time
            elbow_flexion = get_coord_at_time_t(coords_table, pose_time, key='EL_x')
            elbow_pronation = get_coord_at_time_t(coords_table, pose_time, key='PS_y')

            # Get the values for HT joint angles from the sto file
            HT_abd, HT_flex, HT_rot = get_HT_angles_from_sto(analysis_sto_file, pose_time)

            # Compile the pose JAs into one dict
            pose_dict = {'elbow_flexion': [elbow_flexion],
                         'elbow_pronation': [elbow_pronation],
                         'HT_abd': [HT_abd],
                         'HT_flex': [HT_flex],
                         'HT_rot': [HT_rot]}

            # Specify the default model pose Jas
            if pose_name in ['N_self', 'N_asst']:
                model_pose_dict = {'elbow_flexion': np.array([0]),
                                   'elbow_pronation': np.array([90]),
                                   'HT_abd': np.array([0]),
                                   'HT_flex': np.array([0]),
                                   'HT_rot': np.array([90])}

            elif pose_name in ['Alt_self', 'Alt_asst']:
                model_pose_dict = {'elbow_flexion': np.array([90]),
                                   'elbow_pronation': np.array([90]),
                                   'HT_abd': np.array([0]),
                                   'HT_flex': np.array([0]),
                                   'HT_rot': np.array([90])}

            elif pose_name == 'Alt2_self':
                model_pose_dict = {'elbow_flexion': np.array([90]),
                                   'elbow_pronation': np.array([90]),
                                   'HT_abd': np.array([0]),
                                   'HT_flex': np.array([90]),
                                   'HT_rot': np.array([90])}
            else:
                model_pose_dict = None
                print('Pose name does not match one listed here.')



            # Find the pose error in each JA
            error_dict = {'Trial': [trial_name],
                         'Pose': [pose_name],
                         'elbow_flexion': None,
                         'elbow_pronation': None,
                         'HT_abd': None,
                         'HT_flex': None,
                         'HT_rot': None}

            # Get the error (difference between performed pose, and model default pose) for each JA
            for JA in model_pose_dict:
                error_dict[JA] = model_pose_dict[JA][0] - pose_dict[JA][0]

            # Add the results from this pose into the results dataframe
            new_df_row = pd.DataFrame(error_dict)
            all_results = pd.concat([all_results, new_df_row], ignore_index=True)





            # TODO: rejig these functions so they are grouped better
            # TODO: choose some print options so we can see specifics if we want to


pd.set_option('display.width', None)
print(all_results)


# TODO: Save results to file per person
# TODO: Compile all subject results - ready for visualisation in R - be sure to maintain bias