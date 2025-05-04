# This script caculates the HT joint angles from the body oris (output from analsysi step) and reads the IK results from
# the .mot file, then compiles all kinematic results together into one file
from pandas.core.dtypes.cast import ensure_dtype_can_hold_na

from helpers_compare import check_sensible_GHs
from helpers_compare import convert_osim_table_to_df
from helpers_compare import get_body_quats_from_analysis_sto
from helpers_compare import trim_df
from helpers_compare import get_HT_angles
from constants import data_dir

import pandas as pd
import math
import os
from os.path import join
import opensim as osim

def run_compile_JAs(subject_code, trial_name, calibration_name, IMU_type):

    print(f'\nRunning compileJAs.py for {subject_code}, {trial_name}, {calibration_name}, {IMU_type}')

    """ SETTINGS """
    # Define some file names
    parent_dir = data_dir + '\\' + subject_code
    IMU_type_dir = join(parent_dir, IMU_type)
    IMU_IK_results_dir = join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
    OMC_IK_results_dir = join(parent_dir, 'OMC', trial_name + '_IK_Results')
    IMU_analysis_sto_path = join(IMU_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
    OMC_analysis_sto_path = join(OMC_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
    IMU_mot_file = join(IMU_IK_results_dir, 'IMU_IK_results.mot')
    OMC_mot_file = join(OMC_IK_results_dir, 'OMC_IK_results.mot')
    IMU_IK_results_file = join(IMU_IK_results_dir, 'all_IMU_IK_results.csv')
    OMC_IK_results_file = join(OMC_IK_results_dir, 'all_OMC_IK_results.csv')

    # Read in coordinates from IMU IK results .mot files to get start and end times
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)
    start_time = math.floor(IMU_table.getIndependentColumn()[0])
    end_time = math.floor(IMU_table.getIndependentColumn()[-1])

    # Run compile_JAs for IMU data
    print(f'\n  Running compileJAs.py for IMC')
    compile_JAs(IMU_analysis_sto_path, IMU_mot_file, IMU_IK_results_file, start_time, end_time, 'IMU')

    # Run compile_JAs for OMC data if it hasn't been run already
    if not os.path.exists(OMC_IK_results_file):
        print(f'\n  Running compileJAs.py for OMC')
        compile_JAs(OMC_analysis_sto_path, OMC_mot_file, OMC_IK_results_file, start_time, end_time, 'OMC')


def compile_JAs(analysis_sto_path, mot_file, results_file, start_time, end_time, IMU_or_OMC):

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    table = osim.TimeSeriesTable(mot_file)

    # Check that the IMU solution seems sensible by checking GH coords are within a certain range
    if IMU_or_OMC == 'IMU':
        check_sensible_GHs(mot_file, table, GH_coord_limit=180)

    # Turn the osim coords tables into dataframes
    coords = convert_osim_table_to_df(table)

    # Read in the orientation data of the model bodies from the analysis sto results file
    body_quats = get_body_quats_from_analysis_sto(analysis_sto_path, start_time, end_time)

    # Trim the dataframes based on the specified start and end times
    coords = trim_df(coords, start_time, end_time)
    body_quats = trim_df(body_quats, start_time, end_time)

    """ GET HT ANGLES """

    # Get projected vector humero-thoracic joint angles from the model body orientations
    HT_angles = get_HT_angles(body_quats)

    # Join the coord and HT angles data together to make one data frame with all angles
    rename_dict = {'TH_y': 'thorax_rotation', 'TH_x': 'thorax_forward_tilt', 'TH_z': 'thorax_lateral_tilt',
                   'EL_x': 'elbow_flexion', 'PS_y': 'elbow_pronation'}
    angles = coords[['TH_y', 'TH_x', 'TH_z', 'EL_x', 'PS_y']].copy().rename(columns=rename_dict)
    all_angles = pd.concat([angles, HT_angles], axis=1)

    # Reorder dataframe columns so time is first
    cols = ['time'] + [col for col in all_angles.columns if col != 'time']
    all_angles = all_angles[cols]

    """ WRITE RESULTS """

    # Write final RMSE values to a csv
    print('Writing results to .csv.')
    all_angles.to_csv(results_file, mode='w', encoding='utf-8', na_rep='nan',
                      index=False)


# Run single test
if __name__ == '__main__':

    run_compile_JAs('P019', 'JA_Slow', 'OSIM_N_self', 'Perfect')
