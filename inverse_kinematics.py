# This script runs IMU IK with the OpenSense API
# Input is .sto files create in 1_preprocess.py, one with calibration pose, one with movements of interest
# Calibrates an .osim model by assigning IMUs to segments
# Outputs .mot IK results

from constants import IMU_IK_settings_file
from helpers_inverse_k import run_osim_IMU_IK
from helpers_inverse_k import get_event_dict_from_file

import os
import opensim as osim



def run_IMU_IK(subject_code, trial_name, calibration_name, IK_start_time, IK_end_time, start_at_pose_bool, IK_trim_bool, IMU_type):

    """ SETTINGS """

    # IMU IK Settings
    IK_output_file_name = "IMU_IK_results.mot"
    visualize_tracking = False
    sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    IMU_type_dir = os.path.join(parent_dir, IMU_type)
    IK_results_parent_dir = os.path.join(IMU_type_dir, 'IMU_IK_results_' + calibration_name)
    IK_results_dir = os.path.join(IK_results_parent_dir, trial_name)
    calibrated_model_file = os.path.join(IMU_type_dir, 'Calibrated_Models', calibration_name, 'Calibrated_das3.osim')
    orientations_file = IMU_type + '_Quats_all.sto'
    orientations_file_path = os.path.join(IMU_type_dir.replace(IMU_type, ''), 'Preprocessed_Data', trial_name, orientations_file)
    # Make new folders if they don't exist yet
    os.makedirs(IK_results_dir, exist_ok=True)
    os.makedirs(IK_results_parent_dir, exist_ok=True)

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(IK_results_dir + r'\IMU_IK.log')

    """ MAIN """

    # Check that the calibrated model has been created
    if os.path.exists(calibrated_model_file) == False:
        print(f"You haven't created the calibrated model for subject {subject_code}, IMU_type: {IMU_type}, calibration_name: {calibration_name} yet")
        print("Quitting.")
        quit()

    if start_at_pose_bool:
        # Get the time at which the alt pose is performed, to start the IK from there
        subject_event_dict = get_event_dict_from_file(subject_code)
        pose_time = subject_event_dict[trial_name]['Alt_self']
        IK_start_time = pose_time

    # Run the IMU IK based on settings inputs above
    print(f'Running IMU IK for {subject_code}, {calibration_name}, {trial_name}, {IMU_type}.')
    run_osim_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file_path, sensor_to_opensim_rotations,
               IK_results_dir, start_at_pose_bool, IK_trim_bool, IK_start_time, IK_end_time, IK_output_file_name, visualize_tracking)


""" TEST """

run_test = False
if run_test:
    subject_code = 'P1'
    trial_name = 'JA_Slow'
    calibration_name = 'METHOD_2_Alt_self'
    IK_start_time = 0
    IK_end_time = 10
    IK_trim_bool = False
    IMU_type = 'Perfect'
    run_IMU_IK(subject_code, trial_name, calibration_name, IK_start_time, IK_end_time, IK_trim_bool, IMU_type)