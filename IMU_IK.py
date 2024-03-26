# This script runs IMU IK with the OpenSense API
# Input is .sto files create in IMU_preprocess.py, one with calibration pose, one with movements of interest
# Calibrates an .osim model by assigning IMUs to segments
# Outputs .mot IK results


from IMU_IK_functions import *
import os
from functions import *


""" SETTINGS """

# Quick Settings
subject_code = 'P2'
trial_name = 'JA_Slow'      # Choose which trial to run IK
orientations_file = 'Cluster_Quats_all.sto'     # Specify which orientation .sto data to use
trim_bool = False   # Whether to use the start and end times defined below
IK_start_time = 205     # Only used if trim_bool = True
IK_end_time = 220       # Only used if trim_bool = True
calibration_name = 'ALL_POSE_BASED_N_asst'     # Used to find the calibrated model file

# IMU IK Settings
IMU_IK_settings_file = 'IMU_IK_Settings.xml'
IK_output_file_name = "IMU_IK_results.mot"
visualize_tracking = False
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)

# Define some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
IK_results_parent_dir = os.path.join(parent_dir, 'IMU_IK_results_' + calibration_name)
if os.path.exists(IK_results_parent_dir) == False:
    os.mkdir(IK_results_parent_dir)
IK_results_dir = os.path.join(IK_results_parent_dir, trial_name)
if os.path.exists(IK_results_dir) == False:
    os.mkdir(IK_results_dir)
calibrated_model_file = os.path.join(parent_dir, 'Calibrated_Models', calibration_name, 'Calibrated_das3.osim')
orientations_file_path = os.path.join(parent_dir, 'Preprocessed_Data', trial_name, orientations_file)

# Create opensim logger file
osim.Logger.removeFileSink()
osim.Logger.addFileSink(IK_results_dir + r'\IMU_IK.log')



""" MAIN """

# Run the IMU IK based on settings inputs above
run_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file_path, sensor_to_opensim_rotations,
           IK_results_dir, trim_bool, IK_start_time, IK_end_time, IK_output_file_name, visualize_tracking)


