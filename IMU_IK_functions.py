import opensim as osim


def APDM_2_sto_Converter(APDM_settings_file, input_file_name, output_file_name):

    # Build an APDM Settings Object
    # Instantiate the Reader Settings Class
    APDMSettings = osim.APDMDataReaderSettings(APDM_settings_file)
    # Instantiate an APDMDataReader
    APDM = osim.APDMDataReader(APDMSettings)

    # Read in table of movement data_out from the specified IMU file
    table = APDM.read(input_file_name)
    # Get Orientation Data as quaternions
    quatTable = APDM.getOrientationsTable(table)

    # Write to file
    osim.STOFileAdapterQuaternion.write(quatTable, output_file_name)


def adjust_calibration_settings(calibration_settings_file, modelFileName, sensor_to_opensim_rotations,
                                orientationsFileName, baseIMUName, baseIMUHeading):

    # Instantiate an IMUPlacer object
    imuPlacer = osim.IMUPlacer(calibration_settings_file)

    # Set properties for the IMUPlacer
    imuPlacer.set_model_file(modelFileName)
    imuPlacer.set_orientation_file_for_calibration(orientationsFileName)
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuPlacer.set_base_imu_label(baseIMUName)
    imuPlacer.set_base_heading_axis(baseIMUHeading)
    # Update settings file
    imuPlacer.printToXML(calibration_settings_file)


def calibrate_model(calibration_settings_file, visulize_calibration, model_file):

    # Instantiate an IMUPlacer object
    imuPlacer = osim.IMUPlacer(calibration_settings_file)

    # Run the IMUPlacer
    imuPlacer.run(visulize_calibration);

    # Get the model with the calibrated IMU
    model = imuPlacer.getCalibratedModel();

    # Print the calibrated model to file.
    model.printToXML('Calibrated_' + model_file)


def adjust_IMU_IK_settings(IMU_IK_settings_file, calibrated_model_file, orientations_file,
                           sensor_to_opensim_rotations, results_directory, start_time, end_time, IK_output_file_name):

    # Instantiate an InverseKinematicsTool
    imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

    # Set tool properties
    imuIK.set_model_file(calibrated_model_file)
    imuIK.set_orientations_file(orientations_file)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuIK.set_results_directory(results_directory)
    imuIK.set_time_range(0, start_time)
    imuIK.set_time_range(1, end_time)
    imuIK.setOutputMotionFileName(IK_output_file_name)

    # Update the settings .xml file
    imuIK.printToXML(IMU_IK_settings_file)



def run_IMU_IK(IMU_IK_settings_file, visualize_tracking):

    # Instantiate an InverseKinematicsTool
    imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

    # Run IK
    imuIK.run(visualize_tracking);

