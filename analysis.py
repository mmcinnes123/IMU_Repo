# This script takes the .mot file from an IK solve, and creates an .sto file with the time-series
# position and orientation of every body in the model
# Input is .mot file
# Output is .sto file

from helpers_analysis import run_analyze_tool
from constants import analyze_settings_template_file, sample_rate, data_dir

import opensim as osim
import os
import numpy as np

def run_analysis(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type):

    """ SETTINGS """

    # Define some file paths
    parent_dir = data_dir + '\\' + subject_code
    IMU_type_dir = os.path.join(parent_dir, IMU_type)
    IK_results_dir = os.path.join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
    coord_file_for_analysis = os.path.join(IK_results_dir, "IMU_IK_results.mot")
    calibrated_model_file = os.path.join(IMU_type_dir, 'Calibrated_Models', calibration_name, 'Calibrated_das3.osim')

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(IK_results_dir + r'\Analysis.log')

    # Set end time by checking length of data
    if trim_bool == False:
        coords_table = osim.TimeSeriesTable(coord_file_for_analysis)
        start_time = np.round(coords_table.getIndependentColumn()[0], 2)
        end_time = np.round(coords_table.getIndependentColumn()[-1], 2)
    else:
        start_time = start_time
        end_time = end_time

    """ MAIN """
    print(f'Running analysis for {subject_code}, {calibration_name}, {trial_name}, {IMU_type}.')

    run_analyze_tool(analyze_settings_template_file, IK_results_dir, calibrated_model_file, coord_file_for_analysis, start_time, end_time)



""" TEST """

run_test = False
if run_test:
    run_analysis(subject_code='P4', trial_name='JA_Slow', calibration_name='ALL_MANUAL', start_time=0, end_time=20, trim_bool=False, IMU_type='Real')