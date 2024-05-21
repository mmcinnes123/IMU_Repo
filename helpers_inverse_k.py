# Functions used to run 1_preprocess.py

import opensim as osim


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

    # Set IMU weights
    thorax_imu_weight = osim.OrientationWeight('thorax_imu', 1.0)
    humerus_imu_weight = osim.OrientationWeight('humerus_r_imu', 0.1)
    radius_imu_weight = osim.OrientationWeight('radius_r_imu', 1.0)
    print('WARNING: Humerus IMU weight set to 0.1')
    imuIK.upd_orientation_weights().cloneAndAppend(thorax_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(humerus_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(radius_imu_weight)

    # Run IK
    imuIK.run(visualize_tracking)

    # Update the settings .xml file
    imuIK.printToXML(results_directory + "\\" + IMU_IK_settings_file)










