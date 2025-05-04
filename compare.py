# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

from helpers_compare import check_sensible_GHs
from helpers_compare import convert_osim_table_to_df
from helpers_compare import get_body_quats_from_analysis_sto
from helpers_compare import trim_df
from helpers_compare import get_HT_angles
from helpers_compare import get_range_dict
from helpers_compare import find_heading_offset
from helpers_compare import plot_compare_any_JAs
from helpers_compare import plot_compare_body_oris
from helpers_compare import alt_plot_for_thesis_compare_any_JAs
from helpers_compare import plot_BA_any_JAs
from helpers_compare import plot_BA_any_JAs_with_peak_data
from constants import data_dir

import pandas as pd
import math
from tkinter.filedialog import askopenfilename, askdirectory
import os
import opensim as osim
from os.path import join


def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type, test):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, {calibration_name}, {IMU_type}')

    if trial_name != 'JA_Slow':
        print('Compare code written specifically for JA_Slow. Need to handle RoM code. Quitting.')
        quit()

    """ SETTINGS """
    # Define some file names
    parent_dir = data_dir + '\\' + subject_code
    JA_range_dict_file = os.path.join(parent_dir, subject_code + '_JA_range_dict.txt')
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name

    IMU_type_dir = join(parent_dir, IMU_type)
    IMU_IK_results_dir = join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
    OMC_IK_results_dir = join(parent_dir, 'OMC', trial_name + '_IK_Results')
    IMU_IK_results_file = join(IMU_IK_results_dir, 'all_IMU_IK_results.csv')
    OMC_IK_results_file = join(OMC_IK_results_dir, 'all_OMC_IK_results.csv')
    results_dir = os.path.join(IMU_IK_results_dir, "Comparison_" + compare_name)
    os.makedirs(results_dir, exist_ok=True) # Make results folder if it doesn't exist yet

    if test:
        results_dir = str(askdirectory(title=' Choose the folder where you want to save the compare results ... '))
        OMC_IK_results_file = str(askopenfilename(title=' Choose the OMC .sto analysis file ... '))
        IMU_IK_results_file = str(askopenfilename(title=' Choose the IMU all IK results .csv file ... '))


    """ READ IN DATA """

    OMC_angles = pd.read_csv(OMC_IK_results_file, encoding='utf-8', na_values='nan', index_col=None)
    IMU_angles = pd.read_csv(IMU_IK_results_file, encoding='utf-8', na_values='nan', index_col=None)

    # Check all dataframes are the same length
    assert len(OMC_angles) == len(IMU_angles), 'Data frames not same length'

    """ MAKE/GET THE JA TIME RANGE DICT """

    # Use interactive span selector to choose time range for each JA movement period
    range_dict = get_range_dict(JA_range_dict_file, IMU_angles, OMC_angles)

    """ ANALYSE JOINT ANGLE ERRORS """

    # Instantiate an empty results df
    results_df = pd.DataFrame(columns=['JA', 'RMSE', 'R', 'peakROM', 'troughROM'])

    # For each joint angle of interest, get the error metrics between IMU and OMC results
    for joint_name in [col for col in IMU_angles.columns if col != 'time']:
        RMSE, R, mean_peak_error, mean_trough_error = \
            plot_compare_any_JAs(joint_name, IMU_angles, OMC_angles, start_time, end_time, results_dir,
                                 range_dict)
        new_row = pd.DataFrame({'JA': joint_name, 'RMSE': [RMSE], 'R': [R], 'peakROM': [mean_peak_error], 'troughROM': [mean_trough_error]})
        results_df = pd.concat([results_df, new_row], ignore_index=True)

    # # Create another plot with nice formatting
    # for joint_name in [col for col in IMU_angles.columns if col != 'time']:
    #     alt_plot_for_thesis_compare_any_JAs(joint_name, IMU_angles, OMC_angles, start_time, end_time, results_dir,
    #                              range_dict, compare_name)

    # Create a bland altman plot
    for joint_name in [col for col in IMU_angles.columns if col not in ['time', 'thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation']]:
        plot_BA_any_JAs_with_peak_data(joint_name, IMU_angles, OMC_angles, start_time, end_time, results_dir,
                                 range_dict)

    """ ANALYSE MODEL BODY ORIENTATIONS """

    # This section requires body_quats, read in from analysis .sto file (see compileJAs.py)
    # This function has been changed so no longers reads .sto so section has been commented out

    # # Find the average heading offset between the IMU thorax body and the OMC thorax body frames
    # heading_offset = find_heading_offset(OMC_body_quats, IMU_body_quats, report=False)
    # # For each model body, calculate the angular distance error between IMU and OMC model
    # RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
    #     plot_compare_body_oris(OMC_body_quats, IMU_body_quats,
    #                        heading_offset, start_time, end_time, results_dir)
    # # Add body orientation results to results df
    # new_row1 = pd.DataFrame({'JA': 'thorax_ori', 'RMSE': [RMSE_thorax_ori]})
    # new_row2 = pd.DataFrame({'JA': 'humerus_ori', 'RMSE': [RMSE_humerus_ori]})
    # new_row3 = pd.DataFrame({'JA': 'radius_ori', 'RMSE': [RMSE_radius_ori]})
    # results_df = pd.concat([results_df, new_row1, new_row2, new_row3])


    """ WRITE RESULTS """

    all_data = results_df

    # Write final RMSE values to a csv
    print('Writing results to .csv.')
    all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                    mode='w', encoding='utf-8', na_rep='nan', index=False)



# Run single test (set test=True to choose input and output files)
if __name__ == '__main__':

    run_IK_compare('P019', 'JA_Slow', 'OSIM_N_self', 6, 80, False, 'Perfect', test=False)
