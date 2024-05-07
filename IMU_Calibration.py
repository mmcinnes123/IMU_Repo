# This script takes an uncalibrated model, applies the chosen calibration method, and outputs a calibrated model
# Files required in same folder as this script:
# Template model file
# Calibration settings template xml file

from IMU_Calibration_helpers import get_calibration_inputs
from IMU_Calibration_helpers import osim_calibrate_model
from IMU_Calibration_helpers import custom_calibrate_model



""" QUICK SETTINGS """

# Define which subjects/IMU types/trial name you want to run the calibration for
subject_code_list = ['P1', 'P2', 'P3']
IMU_type_list = ['Real', 'Perfect']
trial_name = 'CP'   # Specify which trial to use for calibration pose

""" RUN THE OPENSIM CALIBRATIONS """
osim_calibration_name_dict = {'OSIM_N_self': 'N_self', 'OSIM_Alt_self': 'Alt_self'}

for calibration_name, pose_name in osim_calibration_name_dict.items():
    for subject_code in subject_code_list:
        for IMU_type in IMU_type_list:
            print(f'Creating {calibration_name} calibrated model for '
                  f'subject {subject_code}, using {IMU_type} IMUs from trial: {trial_name}')

            # Get the inputs needed to run the calibration
            cal_oris_file_path, calibrated_model_dir, model_file = \
                get_calibration_inputs(subject_code, trial_name, IMU_type, calibration_name, pose_name)

            # Run the opensim calibration
            osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, model_file)


""" RUN THE CUSTOM CALIBRATIONS """
custom_calibration_name_dict = {'METHOD_2_Alt_self': 'Alt_self', 'ALL_MANUAL': 'Alt_self'}

for calibration_name, pose_name in custom_calibration_name_dict.items():
    for subject_code in subject_code_list:
        for IMU_type in IMU_type_list:
            print(f'Creating {calibration_name} calibrated model for '
                  f'subject {subject_code}, using {IMU_type} IMUs from trial: {trial_name}')

            # Get the inputs needed to run the calibration
            cal_oris_file_path, calibrated_model_dir, model_file = \
                get_calibration_inputs(subject_code, trial_name, IMU_type, calibration_name, pose_name)

            # Run my custom calibration
            custom_calibrate_model(calibration_name, cal_oris_file_path, calibrated_model_dir, model_file)

