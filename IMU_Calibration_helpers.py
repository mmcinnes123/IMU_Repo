# Functions used in IMU_Calibration.py

import os
import opensim as osim
from Calibration_functions import get_IMU_offset
from Calibration_functions import apply_cal_to_model

# Calibration settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
calibration_settings_template_file = "IMU_Calibration_Settings.xml"


def get_make_model_dir(IMU_type_dir, calibration_name):
    calibrated_models_dir = os.path.join(IMU_type_dir, 'Calibrated_Models')
    if os.path.exists(calibrated_models_dir) == False:
        os.mkdir(calibrated_models_dir)
    calibrated_model_dir = os.path.join(calibrated_models_dir, calibration_name)
    if os.path.exists(calibrated_model_dir) == False:
        os.mkdir(calibrated_model_dir)
    return calibrated_model_dir


def get_oris_file(IMU_type, pose_name, sto_files_dir):
    cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
    cal_oris_file_path = os.path.join(sto_files_dir, cal_oris_file)
    return cal_oris_file_path


def get_calibration_inputs(subject_code, trial_name, IMU_type, calibration_name, pose_name):
    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
    sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'),
                                 trial_name)  # dir for preprocess orientations files
    IMU_type_dir = os.path.join(parent_dir, IMU_type)  # working dir for each IMU type
    if os.path.exists(IMU_type_dir) == False:
        os.mkdir(IMU_type_dir)

    # Get/make the directory based on calibration name
    calibrated_model_dir = get_make_model_dir(IMU_type_dir, calibration_name)

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(calibrated_model_dir + r'\calibration.log')

    # Get the orientations data to be used for the calibration
    cal_oris_file_path = get_oris_file(IMU_type, pose_name, sto_files_dir)

    # Check we've set the default pose of the template model correctly
    pose_confirmation = input(
        f"\nIs the default pose of the model set to match the expected subject pose ({pose_name})?: ")
    if pose_confirmation == "No":
        quit()

    # Read in the template model
    model_file = 'das3.osim'

    return cal_oris_file_path, calibrated_model_dir, model_file


def get_cal_method_dict(calibration_name):
    """ DEFINING CUSTOM CALIBRATION METHODS """
    # Calibration Method Options:
    # Pose-only (OpenSim): get_IMU_cal_POSE_BASED
    # Manual alignment: get_IMU_cal_MANUAL
    # Combined: Pose-based, but then correct with manual Y: get_IMU_cal_POSE_and_MANUAL_Y
    # Manual: Humerus-specific, using humerus IMU y-axis and radius IMU y-axis: get_IMU_cal_hum_method_2
    # Humerus method 3: pose defines flexion, humerus IMU defines adb, forearm IMU defines int/ext: get_IMU_cal_hum_method_3

    if calibration_name == 'ALL_MANUAL':
        thorax_method = 'get_IMU_cal_MANUAL'
        humerus_method = 'get_IMU_cal_MANUAL'
        radius_method = 'get_IMU_cal_MANUAL'

    elif calibration_name == 'METHOD_1_Alt_self':
        thorax_method = 'get_IMU_cal_POSE_BASED'
        humerus_method = 'get_IMU_cal_hum_method_2'
        radius_method = 'get_IMU_cal_MANUAL'

    elif calibration_name == 'METHOD_2_Alt_self':
        thorax_method = 'get_IMU_cal_POSE_BASED'
        humerus_method = 'get_IMU_cal_hum_method_3'
        radius_method = 'get_IMU_cal_MANUAL'

    # Set the cal_method_dict for each body based on the method chosen above
    cal_method_dict = {'Thorax': thorax_method,
                       'Humerus': humerus_method,
                       'Radius': radius_method}

    return cal_method_dict


def custom_calibrate_model(calibration_name, cal_oris_file_path, calibrated_model_dir, model_file):
    cal_method_dict = get_cal_method_dict(calibration_name)

    # Calibrate the model based on my own methods (method for each body is defined within get_IMU_offset function)
    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
        get_IMU_offset(cal_method_dict, cal_oris_file_path, model_file, calibrated_model_dir,
                       baseIMUHeading)

    # Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
    apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, calibrated_model_dir)


def osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, model_file):
    # Use OpenSim's built-in calibration method
    run_osim_calibrate_model(calibration_settings_template_file, model_file, sensor_to_opensim_rotations,
                             cal_oris_file_path, baseIMUName, baseIMUHeading, calibrated_model_dir)


def run_osim_calibrate_model(calibration_settings_file, modelFileName, sensor_to_opensim_rotations,
                        calibration_orientations_file, baseIMUName, baseIMUHeading, output_dir):

    # Instantiate an IMUPlacer object
    imuPlacer = osim.IMUPlacer(calibration_settings_file)

    # Set properties for the IMUPlacer
    imuPlacer.set_model_file(modelFileName)
    imuPlacer.set_orientation_file_for_calibration(calibration_orientations_file)
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuPlacer.set_base_imu_label(baseIMUName)
    imuPlacer.set_base_heading_axis(baseIMUHeading)

    # Update settings file
    imuPlacer.printToXML(output_dir + "\\" + calibration_settings_file)

    # Run the IMUPlacer
    visualize_calibration = False
    imuPlacer.run(visualize_calibration)

    # Get the model with the calibrated IMU
    model = imuPlacer.getCalibratedModel()

    # Print the calibrated model to file.
    model.printToXML(output_dir + r'\Calibrated_' + modelFileName)