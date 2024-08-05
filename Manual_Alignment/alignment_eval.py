# This script analyses how well the manual placement of the sensors matches the CFs of the underlying body segments
# Using perfect IMU data (i.e. marker cluster CFs) and calibrated OMC model
import pandas as pd
import qmt

from alignment_eval_helpers import get_all_clus_in_body_frame
from alignment_eval_helpers import plot_local_vec_on_global_plane

import opensim as osim
from os.path import join


# Repress opensim logging
osim.Logger.setLevelString("Off")

# Define the list of subjects
subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

""" 
Define the settings for each JA of interest.
    local_axis - the axis of the IMU which we want to measure the direction of
    global_axis_1 - one of two axis of the body coordinate frame which defines the plane of interest (points left)
    global_axis_2 - one of two axis of the body coordinate frame which defines the plane of interest (points down)
    target_global_axis - the axis of the body coordinate frame which should be well-aligned with the IMU local axis
"""

sh_flex_settings = dict(JA_name='Shoulder Flexion', local_axis='y', global_axis_1='Z', global_axis_2='Y', target_global_axis='-Y')
sh_abd_settings = dict(JA_name='Shoulder Abduction', local_axis='y', global_axis_1='X', global_axis_2='Y', target_global_axis='-Y')
sh_rot_settings = dict(JA_name='Shoulder Rotation', local_axis='z', global_axis_1='X', global_axis_2='Z', target_global_axis='X')
el_flex_settings = dict(JA_name='Elbow Flexion', local_axis='y', global_axis_1='Y', global_axis_2='X', target_global_axis='-Y')
el_abd_settings = dict(JA_name='Elbow Abduction', local_axis='z', global_axis_1='Z', global_axis_2='Y', target_global_axis='Z')
pro_sup_settings = dict(JA_name='Pronation/Supination', local_axis='z', global_axis_1='Z', global_axis_2='X', target_global_axis='Z')


""" 
From the calibrated OMC models, get the orientation of the marker cluster frames (representing 'perfect' IMUs) 
defined in the frame of the associated body (e.g. humerus or radius frame).
"""

hum_clus_all, rad_clus_all = get_all_clus_in_body_frame(subject_list)

""" 
For each joint angle, for each subject, get the 2D vector projected onto the plane of interest, plot these on a 2D plot, 
 and get the angular difference ('error') between the vector and the 'target vector', and the variation in that error. 
 """

all_results = pd.DataFrame(columns=['JA', 'mean_error', 'SD_error'])

for JA_settings in [sh_flex_settings, sh_abd_settings, sh_rot_settings]:
    mean_error, SD_error = plot_local_vec_on_global_plane(hum_clus_all,
                                   JA_name=JA_settings['JA_name'],
                                   local_axis=JA_settings['local_axis'],
                                   global_axis_1=JA_settings['global_axis_1'],
                                   global_axis_2=JA_settings['global_axis_2'],
                                   target_global_axis=JA_settings['target_global_axis'])
    new_row = pd.DataFrame({'JA': [JA_settings['JA_name']], 'mean_error': [mean_error],
                            'SD_error': [SD_error]})
    # Log the results
    all_results = pd.concat([all_results, new_row], ignore_index=True)

for JA_settings in [el_flex_settings, el_abd_settings, pro_sup_settings]:
    mean_error, SD_error = plot_local_vec_on_global_plane(rad_clus_all,
                                   JA_name=JA_settings['JA_name'],
                                   local_axis=JA_settings['local_axis'],
                                   global_axis_1=JA_settings['global_axis_1'],
                                   global_axis_2=JA_settings['global_axis_2'],
                                   target_global_axis=JA_settings['target_global_axis'])
    new_row = pd.DataFrame({'JA': [JA_settings['JA_name']], 'mean_error': [mean_error],
                            'SD_error': [SD_error]})
    # Log the results
    all_results = pd.concat([all_results, new_row], ignore_index=True)

# Save results to csv
save_file_path = join(r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\R Analysis\Manual Alignment', 'Alignment_results.csv')
all_results.to_csv(save_file_path, index=False)
