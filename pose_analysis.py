# This script analyses how successfully a subject performed each pose, relative to the 'perfect' default pose of the model


# Imports
from pose_analysis_helpers import get_trial_pose_time_dict_from_file

import os

""" SETTINGS """
# subject_code_list = ['P1']
subject_code = 'P1'

# Define some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
OMC_dir = os.path.join(parent_dir, 'OMC')

# Get the dict which defines when each pose happened in each trial
trial_pose_time_dict = get_trial_pose_time_dict_from_file(parent_dir, subject_code)

print(trial_pose_time_dict)
""" MAIN """

# Get all JAs from OMC IK results



# Get all JAs from default model pose



# Get all JA errors



# Save results to file