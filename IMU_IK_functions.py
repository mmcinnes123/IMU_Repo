
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


def run_calibrate_model(calibration_settings_file, modelFileName, sensor_to_opensim_rotations,
                        calibration_orientations_file, baseIMUName, baseIMUHeading,
                        visulize_calibration, output_dir):

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
    imuPlacer.run(visulize_calibration)

    # Get the model with the calibrated IMU
    model = imuPlacer.getCalibratedModel()

    # Print the calibrated model to file.
    model.printToXML(output_dir + r'\Calibrated_' + modelFileName)


def run_osim_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file,
               sensor_to_opensim_rotations, results_directory, trim_bool,
               start_time, end_time, IK_output_file_name, visualize_tracking):

    # Instantiate an InverseKinematicsTool
    imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

    # Set tool properties
    imuIK.set_model_file(calibrated_model_file)
    imuIK.set_orientations_file(orientations_file)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuIK.set_results_directory(results_directory)
    if trim_bool == True:
        imuIK.set_time_range(0, start_time)
        imuIK.set_time_range(1, end_time)
    imuIK.setOutputMotionFileName(IK_output_file_name)

    # Run IK
    imuIK.run(visualize_tracking)

    # Update the settings .xml file
    imuIK.printToXML(results_directory + "\\" + IMU_IK_settings_file)


def create_states_file_from_coordinates_file(analyze_settings_template_file, model_file, coord_file,
                                             results_path, start_time, end_time, trial_name):

    # Instantiate a Analyze Tool
    analyze_tool = osim.AnalyzeTool(analyze_settings_template_file)
    analyze_tool.setModelFilename(model_file)
    analyze_tool.setResultsDir(results_path)
    analyze_tool.setCoordinatesFileName(coord_file)
    analyze_tool.setInitialTime(start_time)
    analyze_tool.setFinalTime(end_time)
    analyze_tool.setName(trial_name)
    analyze_tool.run()








