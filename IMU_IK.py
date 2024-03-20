# This script runs IMU IK with the OpenSense API
# Input is .sto files create in IMU_preprocess.py, one with calibration pose, one with movements of interest
# Calibrates an .osim model by assigning IMUs to segments
# Outputs .mot IK results


from IMU_IK_functions import *
import os
from functions import *
from Calibration_functions import get_IMU_offset, apply_cal_to_model


""" SETTINGS """

# Quick Settings
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P2"  # Name of the working folder
orientations_file = parent_dir + r"\Preprocessed_Data" + r'\JA_Slow' + r"\Cluster_Quats_all.sto"    # Specify which orientation .sto data to use
trim_bool = False   # Whether to use the start and end times defined below
IK_start_time = 205     # Only used if trim_bool = True
IK_end_time = 220       # Only used if trim_bool = True
calibration_name = 'ALL_POSE_BASED'     # Used to find the calibrated model file
trial_name = calibration_name + "_JA_Slow"   # Tag to describe this trial, used as IK results folder name

# Files required in folder:
IMU_IK_settings_file = 'IMU_IK_Settings.xml'
model_file = 'das3.osim'

# IMU IK Settings
IK_results_dir = parent_dir + "\\" + trial_name + "_IMU_IK_results"
IK_output_file_name = "IMU_IK_results.mot"
visualize_tracking = False
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)

# Define some file paths
calibrated_model_file = os.path.join(parent_dir, 'Calibrated_Models', calibration_name, 'Calibrated_das3.osim')


# Create a new results directory
if os.path.exists(IK_results_dir) == False:
    os.mkdir(IK_results_dir)

# Create opensim logger file
osim.Logger.removeFileSink()
osim.Logger.addFileSink(IK_results_dir + r'\IMU_IK.log')
osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files



""" MAIN """

# Run the IMU IK  based on settings inputs above
osim.Model.setDebugLevel(0)  # Update debug level so we can see IK progress
run_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file, sensor_to_opensim_rotations,
           IK_results_dir, trim_bool, IK_start_time, IK_end_time, IK_output_file_name, visualize_tracking)


