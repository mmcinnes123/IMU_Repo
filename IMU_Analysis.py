# This script takes the .mot file from an IK solve, and creates a .sto states file,
# then a .csv file with the orientations of each body in the model
# Input is .mot file
# Output is .sto states file and .csv file

from IMU_IK_functions import *
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
    new_states_file_path = os.path.join(IK_results_dir, trial_name + '_StatesReporter_states.sto')

    # Analyze Settings
    analyze_settings_template_file = "Analysis_Settings.xml"
    model_file_for_analysis = calibrated_model_file

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(IK_results_dir + r'\Analysis.log')
    osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files

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

    # Create states file from the output .mot file
    create_states_file_from_coordinates_file(analyze_settings_template_file, model_file_for_analysis, coord_file_for_analysis,
                                             IK_results_dir, start_time, end_time, trial_name)

    # Create a csv file with body oris
    print(f'\nCreating {trial_name} body orientations csv file from states file')
    states_table = osim.TimeSeriesTable(new_states_file_path)   # Read in new states
    # Match the model to the states, extract the orientation of each body, then save to csv
    extract_body_quats(states_table, model_file_for_analysis, IK_results_dir, tag=trial_name + '_IMU')



