# This script takes an uncalibrated model, applies the chosen calibration method, and outputs a calibrated model


from Calibration_functions import *
from IMU_IK_functions import *
import os

""" SETTINGS """

# Quick Settings
subject_code = 'P1'
calibration_name = 'METHOD_2'     # Choose what you want this calibration to be called
trial_name = 'CP'   # Specify which trial to use for calibration pose
IMU_type = 'Perfect'       # either 'Real' or 'Perfect'
pose_name = 'Alt_self'  # Same as used to save the .sto files

""" DEFINING CUSTOM CALIBRATION METHODS """
# Calibration Method Options:
# Pose-only (OpenSim): get_IMU_cal_POSE_BASED
# Manual alignment: get_IMU_cal_MANUAL
# Combined: Pose-based, but then correct with manual Y: get_IMU_cal_POSE_and_MANUAL_Y
# Manual: Humerus-specific, using humerus IMU y-axis and radius IMU y-axis: get_humerus_IMU_cal_MANUAL_Ys
# Humerus method 3: pose defines flexion, humerus IMU defines adb, forearm IMU defines int/ext: get_IMU_cal_hum_method_3

if calibration_name == 'ALL_MANUAL':
    thorax_method = 'get_IMU_cal_MANUAL'
    humerus_method = 'get_IMU_cal_MANUAL'
    radius_method = 'get_IMU_cal_MANUAL'

elif calibration_name == 'METHOD_1':
    thorax_method = 'get_IMU_cal_POSE_BASED'
    humerus_method = 'get_humerus_IMU_cal_MANUAL_Ys'
    radius_method = 'get_IMU_cal_MANUAL'
    pose_name = 'Alt_self'  # Same as used to save the .sto files

elif calibration_name == 'METHOD_2':
    thorax_method = 'get_IMU_cal_POSE_BASED'
    humerus_method = 'get_IMU_cal_hum_method_3'
    radius_method = 'get_IMU_cal_MANUAL'
    pose_name = 'Alt_self'  # Same as used to save the .sto files


""" OTHER SETTINGS """

# Define some file paths
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
IMU_type_dir = os.path.join(parent_dir, IMU_type)
if os.path.exists(IMU_type_dir) == False:
    os.mkdir(IMU_type_dir)
calibration_orientations_file = IMU_type + '_Quats_' + pose_name + '.sto'
sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'), trial_name)
calibration_orientations_file_path = os.path.join(sto_files_dir, calibration_orientations_file)
calibrated_models_dir = os.path.join(IMU_type_dir, 'Calibrated_Models')

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
# osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files



""" MAIN """

# Check we've set the default pose of the model correctly
pose_confirmation = input(f"\nIs the default pose of the model set to match the expected subject pose ({pose_name})?: ")
if pose_confirmation == "No":
    quit()

if calibration_name == 'OSIM_N_self':
    # Use OpenSim's built-in calibration method
    run_calibrate_model(calibration_settings_file, model_file, sensor_to_opensim_rotations,
                        calibration_orientations_file_path, baseIMUName, baseIMUHeading,
                        visualize_calibration, calibrated_model_dir)

elif calibration_name == 'OSIM_Alt_self':
    # Use OpenSim's built-in calibration method
    run_calibrate_model(calibration_settings_file, model_file, sensor_to_opensim_rotations,
                        calibration_orientations_file_path, baseIMUName, baseIMUHeading,
                        visualize_calibration, calibrated_model_dir)

else:

    # Set the cal_method_dict for each body based on the method chosen above
    cal_method_dict = {'Thorax': thorax_method,
                       'Humerus': humerus_method,
                       'Radius': radius_method}

    # Calibrate the model based on my own methods (method for each body is defined within get_IMU_offset function)
    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
        get_IMU_offset(cal_method_dict, calibration_orientations_file_path, model_file, calibrated_model_dir, baseIMUHeading)

    # Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
    apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, calibrated_model_dir)

