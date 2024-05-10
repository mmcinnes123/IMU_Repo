# Functions used in IMU_Calibration.py

import os
import opensim as osim


# Calibration settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
calibration_settings_template_file = "IMU_Calibration_Settings.xml"



""" NEW LIST OF FUCNTIONS ACTUALLY IN USE """

def get_calibrated_model_dir(subject_code, IMU_type, calibration_name):

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
    IMU_type_dir = os.path.join(parent_dir, IMU_type)  # working dir for each IMU type
    if os.path.exists(IMU_type_dir) == False:
        os.mkdir(IMU_type_dir)

    # Get/make the directory based on calibration name
    calibrated_model_dir = get_make_model_dir(IMU_type_dir, calibration_name)

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(calibrated_model_dir + r'\calibration.log')

    return calibrated_model_dir

def get_make_model_dir(IMU_type_dir, calibration_name):
    calibrated_models_dir = os.path.join(IMU_type_dir, 'Calibrated_Models')
    if os.path.exists(calibrated_models_dir) == False:
        os.mkdir(calibrated_models_dir)
    calibrated_model_dir = os.path.join(calibrated_models_dir, calibration_name)
    if os.path.exists(calibrated_model_dir) == False:
        os.mkdir(calibrated_model_dir)
    return calibrated_model_dir




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


""" ARCHIVED FUNCTIONS """


# def get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type):
#     parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
#     sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'), trial_name)  # dir for preprocess orientations files
#     cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
#     cal_oris_file_path = os.path.join(sto_files_dir, cal_oris_file)
#     return cal_oris_file_path