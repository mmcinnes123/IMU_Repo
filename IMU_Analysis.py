# This script takes the .mot file from an IK solve, and creates an .sto file with the time-series
# position and orientation of every body in the model
# Input is .mot file
# Output is .sto file

import os
from functions import *

def run_analysis(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool):

    """ SETTINGS """

    sample_rate = 100

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    IK_results_dir = os.path.join(parent_dir, 'IMU_IK_results_' + calibration_name, trial_name)
    coord_file_for_analysis = os.path.join(IK_results_dir, "IMU_IK_results.mot")
    calibrated_model_file = os.path.join(parent_dir, 'Calibrated_Models', calibration_name, 'Calibrated_das3.osim')

    # Analyze Settings
    analyze_settings_template_file = "Analyze_Settings.xml"

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(IK_results_dir + r'\Analysis.log')

    # Set end time by checking length of data
    if trim_bool == False:
        coords_table = osim.TimeSeriesTable(coord_file_for_analysis)
        n_rows = coords_table.getNumRows()
        start_time = 0
        end_time = n_rows / sample_rate
    else:
        start_time = start_time
        end_time = end_time


    """ MAIN """

    run_analyze_tool(analyze_settings_template_file, IK_results_dir, calibrated_model_file, coord_file_for_analysis, start_time, end_time)



