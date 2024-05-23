# This script analyses how successfully a subject performed each pose, relative to the 'perfect' default pose of the model
import pandas as pd

# Imports
from pose_analysis_helpers import get_trial_pose_time_dict_from_file
from pose_analysis_helpers import get_coords_from_mot
from pose_analysis_helpers import get_coord_at_time_t
from pose_analysis_helpers import get_HT_angles_from_sto

from os.path import join

""" SETTINGS """
subject_code_list = ['P1']




""" MAIN """

for subject_code in subject_code_list:

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_dir = join(parent_dir, 'OMC')

    # Get the dict which defines when each pose happened in each trial
    trial_pose_time_dict = get_trial_pose_time_dict_from_file(parent_dir, subject_code)

    for trial_name in trial_pose_time_dict:
        print(trial_name)
        pose_time_dict = trial_pose_time_dict[trial_name]
        for pose_name, pose_time in pose_time_dict.items():
            print(pose_name)
            print(pose_time)
            # Find the directory with the IK results
            IK_results_dir = join(OMC_dir, trial_name + '_IK_Results')
            mot_file = join(IK_results_dir, 'OMC_IK_results.mot')
            analysis_sto_file = join(IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')

            # Read in the .mot coords file
            coords_table = get_coords_from_mot(mot_file)

            # Get the coords (elbow flexion, pronation) at the specified pose time
            elbow_flexion = get_coord_at_time_t(coords_table, pose_time, key='EL_x')
            elbow_pronation = get_coord_at_time_t(coords_table, pose_time, key='PS_y')

            # Or make this a dict, depending on what you do with it
            df_row = pd.DataFrame({'Trial': [trial_name], 'Pose': [pose_name],
                                    'elbow_flexion': [elbow_flexion], 'elbow_pronation': [elbow_pronation]})


            HT_abd, HT_flex, HT_rot = get_HT_angles_from_sto(analysis_sto_file, pose_time)

            print(HT_abd, HT_flex, HT_rot)

            # TODO: rejig these functions so they are grouped better



    # TODO: Get default joint angles (don't need to do this with code, can just use whatever values I use in 2_get_set_model_default.py

# Get all JAs from OMC IK results



# Get all JAs from default model pose



# Get all JA errors



# Save results to file