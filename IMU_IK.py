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
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_01_22"  # Name of the working folder
trial_name = 'CalCodeTest'    # Tag to describe this trial
IK_start_time = 0
IK_end_time = 37

# Files required in folder:
calibration_settings_file = "IMU_Calibration_Settings.xml"
IMU_IK_settings_file = 'IMU_IK_Settings.xml'
model_file = 'das3.osim'
# + Geometry file for model

# Specify the results directory
results_dir = parent_dir + "\\" + trial_name  # Define the working folder

# Calibration Settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = 'x'    # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
visualize_calibration = False

# IMU IK Settings
calibrated_model_file = results_dir + r'\Calibrated_' + model_file
IK_results_dir = results_dir + "\\" + trial_name + "_IMU_IK_results"
IK_output_file_name = "IMU_IK_results.mot"
visualize_tracking = False

# Define name of .sto files created from pre_process.py
orientations_file = results_dir + r"\APDM_Movements.sto"
calibration_orientations_file = results_dir + r"\APDM_Calibration.sto"

# Analyze Settings
analyze_settings_template_file = "Analysis_Settings.xml"
model_file_for_analysis = calibrated_model_file
coord_file_for_analysis = IK_results_dir + r'\IMU_IK_results.mot'

# Create a new results directory
if os.path.exists(IK_results_dir) == False:
    os.mkdir(IK_results_dir)

# Create opensim logger file
osim.Logger.removeFileSink()
osim.Logger.addFileSink(IK_results_dir + r'\opensim.log')
osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files

""" MAIN """

# # # Calibrate the model based on calibration settings defined above (assign IMUs to segments based on calibration pose)
# run_calibrate_model(calibration_settings_file, model_file, sensor_to_opensim_rotations,
#                     calibration_orientations_file, baseIMUName, baseIMUHeading,
#                     visualize_calibration, results_dir)


# Calibrate the model based on my own methods (method for each body is defined within get_IMU_offset function)
pose_time = 14
thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = get_IMU_offset(pose_time, orientations_file, model_file, results_dir, base_IMU_axis_label='x')
apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, results_dir)


print("\nCalibrated .osim model")

# Check we're happy to go ahead with IK
IK_confirmation = input("\nHappy to go ahead with IK?: ")
if IK_confirmation == "No":
    quit()

# Run the IMU IK  based on settings inputs above
osim.Model.setDebugLevel(0)  # Update debug level so we can see IK progress
run_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file, sensor_to_opensim_rotations,
           IK_results_dir, IK_start_time, IK_end_time, IK_output_file_name, visualize_tracking)


# Create states file from the output .mot file
create_states_file_from_coordinates_file(analyze_settings_template_file, model_file_for_analysis, coord_file_for_analysis,
                                         IK_results_dir, IK_start_time, IK_end_time, trial_name)