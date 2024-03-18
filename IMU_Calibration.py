# This script takes an uncalibrated model, applies the chosen calibration method, and outputs a calibrated model

import numpy as np
from scipy.spatial.transform import Rotation as R
from quat_functions import *
from Calibration_functions import *
import opensim as osim
from functions import *
from IMU_IK_functions import *
import os

""" SETTINGS """

# Quick Settings
subject_code = 'P2'
# Specify which orientation data at which time to use for calibration
calibration_orientations_file = 'Cluster_Quats_N_self_6s.sto'
trial_name = 'CP'
calibration_name = 'ALL_POSE_BASED'
# Options:
# Pose-only (OpenSim): get_IMU_cal_POSE_BASED
# Manual alignment: get_IMU_cal_MANUAL
# Combined: Pose-based, but then correct with manual Y: get_IMU_cal_POSE_and_MANUAL_Y
# Manual: Humerus-specific, using humerus IMU y-axis and radius IMU y-axis: get_humerus_IMU_cal_MANUAL_Ys
cal_method_dict = {'Thorax': 'get_IMU_cal_POSE_BASED',
                   'Humerus': 'get_IMU_cal_POSE_BASED',
                   'Radius': 'get_IMU_cal_POSE_BASED'}

# Define some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'), trial_name)
calibration_orientations_file_path = os.path.join(sto_files_dir, calibration_orientations_file)
calibrated_models_dir = os.path.join(parent_dir, 'Calibrated_Models')
if os.path.exists(calibrated_models_dir) == False:
    os.mkdir(calibrated_models_dir)
calibrated_model_dir = os.path.join(calibrated_models_dir, calibration_name)
if os.path.exists(calibrated_model_dir) == False:
    os.mkdir(calibrated_model_dir)

# Calibration Settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'    # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
visualize_calibration = False

# Files required in folder:
calibration_settings_file = "IMU_Calibration_Settings.xml"
model_file = 'das3.osim'

# Create opensim logger file
osim.Logger.removeFileSink()
osim.Logger.addFileSink(calibrated_model_dir + r'\calibration.log')
osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files

""" MAIN """

# Check we've set the default pose of the model correctly
pose_confirmation = input("\nIs the default pose of the model set to match the expected subject pose?: ")
if pose_confirmation == "No":
    quit()

# Calibrate the model based on my own methods (method for each body is defined within get_IMU_offset function)
thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
    get_IMU_offset(cal_method_dict, calibration_orientations_file_path, model_file, calibrated_model_dir, baseIMUHeading)

# Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, calibrated_model_dir)




# # Code to use OpenSim's built-in calibration method
# run_calibrate_model(calibration_settings_file, model_file, sensor_to_opensim_rotations,
#                     calibration_orientations_file_path, baseIMUName, baseIMUHeading,
#                     visualize_calibration, calibrated_models_dir)