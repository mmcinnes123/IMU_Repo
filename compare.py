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

import pandas as pd
import math
from tkinter.filedialog import askopenfilename, askdirectory
import os
import opensim as osim


def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type, test):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, {calibration_name}, {IMU_type}')

    if trial_name != 'JA_Slow':
        print('Compare code written specifically for JA_Slow. Need to handle RoM code. Quitting.')
        quit()

    """ SETTINGS """
    # Define some file names
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    JA_range_dict_file = os.path.join(parent_dir, subject_code + '_JA_range_dict.txt')
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name

    if test:
        results_dir = str(askdirectory(title=' Choose the folder where you want to save the compare results ... '))
        OMC_mot_file = str(askopenfilename(title=' Choose the OMC .mot coords file ... '))
        OMC_analysis_sto_path = str(askopenfilename(title=' Choose the OMC .sto analysis file ... '))
        IMU_mot_file = str(askopenfilename(title=' Choose the IMU .mot coords file ... '))
        IMU_analysis_sto_path = str(askopenfilename(title=' Choose the IMU .sto analysis file ... '))

    else:
        IMU_type_dir = os.path.join(parent_dir, IMU_type)
        IMU_IK_results_dir = os.path.join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
        OMC_IK_results_dir = os.path.join(parent_dir, 'OMC', trial_name + '_IK_Results')
        IMU_analysis_sto_path = os.path.join(IMU_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
        OMC_analysis_sto_path = os.path.join(OMC_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
        IMU_mot_file = os.path.join(IMU_IK_results_dir, 'IMU_IK_results.mot')
        OMC_mot_file = os.path.join(OMC_IK_results_dir, 'OMC_IK_results.mot')
        results_dir = os.path.join(IMU_IK_results_dir, "Comparison_" + compare_name)

    # Make results folder if it doesn't exist yet
    os.makedirs(results_dir, exist_ok=True)

    """ READ IN DATA """

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)

    # Set start and end time based on the length of the IMU data if trim_bool is false
    if not trim_bool:
        start_time = math.floor(IMU_table.getIndependentColumn()[0])
        end_time = math.floor(IMU_table.getIndependentColumn()[-1])

    # Check that the IMU solution seems sensible by checking GH coords are within a certain range
    check_sensible_GHs(IMU_mot_file, IMU_table, GH_coord_limit=180)

    # Turn the osim coords tables into dataframes
    OMC_coords = convert_osim_table_to_df(OMC_table)
    IMU_coords = convert_osim_table_to_df(IMU_table)

    # Read in the orientation data of the model bodies from the analysis sto results file
    IMU_body_quats = get_body_quats_from_analysis_sto(IMU_analysis_sto_path, start_time, end_time)
    OMC_body_quats = get_body_quats_from_analysis_sto(OMC_analysis_sto_path, start_time, end_time)

    # Trim the dataframes based on the specified start and end times
    OMC_coords = trim_df(OMC_coords, start_time, end_time)
    IMU_coords = trim_df(IMU_coords, start_time, end_time)
    OMC_body_quats = trim_df(OMC_body_quats, start_time, end_time)
    IMU_body_quats = trim_df(IMU_body_quats, start_time, end_time)

    # Check all dataframes are the same length
    assert len(OMC_coords) == len(IMU_coords) == len(OMC_body_quats) == len(IMU_body_quats), 'Data frames not same length'

    """ GET HT ANGLES """

    # Get projected vector humero-thoracic joint angles from the model body orientations
    IMU_HT_angles = get_HT_angles(IMU_body_quats)
    OMC_HT_angles = get_HT_angles(OMC_body_quats)

    # Join the coord and HT angles data together to make one data frame with all angles
    rename_dict = {'TH_y': 'thorax_rotation', 'TH_x': 'thorax_forward_tilt', 'TH_z': 'thorax_lateral_tilt', 'EL_x': 'elbow_flexion', 'PS_y':'elbow_pronation'}
    IMU_angles = IMU_coords[['TH_y', 'TH_x', 'TH_z', 'EL_x', 'PS_y']].copy().rename(columns=rename_dict)
    IMU_angles = pd.concat([IMU_angles, IMU_HT_angles], axis=1)
    OMC_angles = OMC_coords[['TH_y', 'TH_x', 'TH_z', 'EL_x', 'PS_y']].copy().rename(columns=rename_dict)
    OMC_angles = pd.concat([OMC_angles, OMC_HT_angles], axis=1)

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

    """ ANALYSE MODEL BODY ORIENTATIONS """

    # Find the average heading offset between the IMU thorax body and the OMC thorax body frames
    heading_offset = find_heading_offset(OMC_body_quats, IMU_body_quats, report=False)

    # For each model body, calculate the angular distance error between IMU and OMC model
    RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
        plot_compare_body_oris(OMC_body_quats, IMU_body_quats,
                           heading_offset, start_time, end_time, results_dir)

    # Add body orientation results to results df
    new_row1 = pd.DataFrame({'JA': 'thorax_ori', 'RMSE': [RMSE_thorax_ori]})
    new_row2 = pd.DataFrame({'JA': 'humerus_ori', 'RMSE': [RMSE_humerus_ori]})
    new_row3 = pd.DataFrame({'JA': 'radius_ori', 'RMSE': [RMSE_radius_ori]})
    results_df = pd.concat([results_df, new_row1, new_row2, new_row3])


    """ WRITE RESULTS """

    all_data = results_df

    # Write final RMSE values to a csv
    print('Writing results to .csv.')
    all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                    mode='w', encoding='utf-8', na_rep='nan', index=False)


# Run single test
if __name__ == '__main__':

    run_IK_compare('P3', 'JA_Slow', 'METHOD_4_Opt', 0, 90, False, 'Perfect', test=True)
