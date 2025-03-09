# A script for characterising the ROM and variation in each joint angle, given a certain movement period from the OMC IK results

from helpers_compare import convert_osim_table_to_df
from helpers_compare import get_body_quats_from_analysis_sto
from helpers_compare import trim_df
from helpers_compare import get_HT_angles
from helpers_compare import get_ROM_and_var
from helpers_compare import debug_print
from helpers_compare import get_JAs_from_OMC_IK_results
from helpers_calibration import get_event_dict_from_file

import copy
import os
import math
import opensim as osim
import pandas as pd


""" INITIAL SETTINGS """
subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
# subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 2)]
# motion_list = ['ISO_1rep', 'ISO_5rep', 'both', 'kettle', 'drink']
motion_list = ['ISO_PS_5rep', 'ISO_FE_5rep']
save_results_to_csv = True

results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Motion_var_and_ROM_results'

base_motion_period_dict = {
    'ISO_1rep': {'trial_name': 'JA_Slow', 'event_to_start': 'FE5_start', 'event_to_end': 'PS2_start'},
    'ISO_5rep': {'trial_name': 'JA_Slow', 'event_to_start': 'FE_start', 'event_to_end': 'PS_end'},
    'ISO_PS_5rep': {'trial_name': 'JA_Slow', 'event_to_start': 'PS_start', 'event_to_end': 'PS_end'},
    'ISO_FE_5rep': {'trial_name': 'JA_Slow', 'event_to_start': 'FE_start', 'event_to_end': 'FE_end'},
    'both': {'trial_name': 'ADL', 'event_to_start': 'kettle1_start', 'event_to_end': 'drink1_end'},
    'kettle': {'trial_name': 'ADL', 'event_to_start': 'kettle1_start', 'event_to_end': 'kettle1_end'},
    'drink': {'trial_name': 'ADL', 'event_to_start': 'drink1_start', 'event_to_end': 'drink1_end'}
}

results_dict = {'ISO_1rep': [], 'ISO_5rep': [], 'ISO_PS_5rep': [], 'ISO_FE_5rep': [], 'both': [], 'kettle': [], 'drink': []}


for subject_code in subject_list:

    print(f'Processing subject {subject_code}')

    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code

    # Get the dict with the timings for FE and PS events
    motion_period_dict = copy.deepcopy(base_motion_period_dict)  # Create a fresh copy for each subject_code
    if subject_code == 'P008':
        motion_period_dict['both'].update({'event_to_start': 'drink1_start', 'event_to_end': 'kettle1_end'})
    subject_event_dict = get_event_dict_from_file(subject_code)

    for motion in motion_list:

        print(f'Processing motion: {motion}')

        trial_name = motion_period_dict[motion]['trial_name']
        event_to_start = motion_period_dict[motion]['event_to_start']
        event_to_end = motion_period_dict[motion]['event_to_end']

        # Get the start and end time for which to run the optimisation
        start_time = float(subject_event_dict[trial_name][event_to_start])
        end_time = float(subject_event_dict[trial_name][event_to_end])
        # debug_print(motion=motion, event_to_start=event_to_start, event_to_end=event_to_end)

        """ READ IN DATA """
        OMC_angles = get_JAs_from_OMC_IK_results(subject_code, trial_name, start_time, end_time)

        """ GET MOTION CHARACTERISTICS OF EACH JOINT ANGLE """

        # For each joint angle of interest, get the range, variation, and no of data samples
        subject_motion_results = {}
        for joint_name in [col for col in OMC_angles.columns if col != 'time']:

            min, max, range, std = get_ROM_and_var(OMC_angles, joint_name, debug=False)

            # Append the results as a list for this joint
            subject_motion_results[joint_name] = {'min': min, 'max': max, 'range': range, 'std': std}

        subject_motion_results_df = pd.DataFrame.from_dict(subject_motion_results, orient="index")
        # debug_print(subject_motion_results_df=subject_motion_results_df)

        # Add the results for that motion into the right slot in the results dict
        results_dict[motion].append(subject_motion_results_df)


""" GET AVERAGE RESULTS FOR EACH MOTION """

for motion in motion_list:

    print(f'Averaging results for motion: {motion}')

    motion_results_df_list = results_dict[motion]

    averages = pd.concat([each.stack() for each in motion_results_df_list], axis=1)\
                 .apply(lambda x:x.mean(),axis=1)\
                 .unstack()
    debug_print(averages=averages)

    if save_results_to_csv:

        # Save results in a csv for each motion
        results_file_name = motion + '_avg_motion_results.csv'
        results_file_path = os.path.join(results_dir, results_file_name)
        averages.round(2).to_csv(results_file_path)
