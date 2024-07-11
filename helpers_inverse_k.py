# Functions used to run 1_preprocess.py

import opensim as osim
from os.path import join


def run_osim_IMU_IK(IMU_IK_settings_file, calibrated_model_file, orientations_file,
               sensor_to_opensim_rotations, results_directory, start_at_pose_bool, trim_bool,
               start_time, end_time, IK_output_file_name, visualize_tracking):

    osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files

    # Instantiate an InverseKinematicsTool
    imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

    # Set tool properties
    imuIK.set_model_file(calibrated_model_file)
    imuIK.set_orientations_file(orientations_file)
    imuIK.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuIK.set_results_directory(results_directory)

    if start_at_pose_bool:
        imuIK.set_time_range(0, start_time)
    if trim_bool:
        imuIK.set_time_range(0, start_time)
        imuIK.set_time_range(1, end_time)

    imuIK.setOutputMotionFileName(IK_output_file_name)

    # Set IMU weights
    thorax_imu_weight = osim.OrientationWeight('thorax_imu', 1.0)
    humerus_imu_weight = osim.OrientationWeight('humerus_r_imu', 0.1)
    radius_imu_weight = osim.OrientationWeight('radius_r_imu', 1.0)
    imuIK.upd_orientation_weights().cloneAndAppend(thorax_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(humerus_imu_weight)
    imuIK.upd_orientation_weights().cloneAndAppend(radius_imu_weight)

    # Run IK
    print('Running IMU IK...')
    imuIK.run(visualize_tracking)
    print('IMU IK run finished.')

    # Update the settings .xml file
    imuIK.printToXML(results_directory + "\\" + IMU_IK_settings_file)


def get_event_dict_from_file(subject_code):
    event_files_folder = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\SubjectEventFiles'
    event_file_name = subject_code + '_event_dict.txt'
    event_file = join(event_files_folder, event_file_name)

    file_obj = open(event_file, 'r')
    event_dict_str = file_obj.read()
    file_obj.close()
    event_dict = eval(event_dict_str)

    return event_dict