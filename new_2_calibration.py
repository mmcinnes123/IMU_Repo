

from helpers_calibration import get_cal_ori_file_path
from helpers_calibration import osim_calibrate_model
from helpers_calibration import get_calibrated_model_dir
from helpers_calibration import apply_cal_to_model
from helpers_calibration import get_IMU_offsets_ALL_MANUAL
from helpers_calibration import get_IMU_offsets_METHOD_1
from helpers_calibration import get_IMU_offsets_METHOD_2
from helpers_calibration import get_IMU_offsets_METHOD_3
from helpers_calibration import get_IMU_offsets_METHOD_4a
from helpers_calibration import set_default_model_pose
from constants import template_model_file


""" CALIBRATION OPTIONS """

def run_method(subject_code, IMU_type, calibration_name):

    if calibration_name.startswith('OSIM'):

        trial_name = 'CP'

        if calibration_name == 'OSIM_N_self':
            pose_name = 'N_self'
        elif calibration_name == 'OSIM_Alt_self':
            pose_name = 'Alt_self'
        else:
            pose_name = None

        # Get the path to the orientations file specific to the chosen pose
        cal_oris_file_path = get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type)

        # Set the template model pose
        set_default_model_pose(template_model_file, pose_name)

        # Get/make the directory to save the calibrated model
        calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, calibration_name)

        # Run the opensim calibration
        osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, template_model_file)

    else:

        # Get/make the directory to store the newly calibrated model
        calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, calibration_name)

        if calibration_name == 'ALL_MANUAL':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_ALL_MANUAL()

        elif calibration_name == 'METHOD_1_self':
            pose_name = 'Alt_self'
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_1(subject_code, pose_name, IMU_type, calibrated_model_dir)

        elif calibration_name == 'METHOD_2_self':
            pose_name = 'Alt_self'
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_2(subject_code, IMU_type, pose_name, calibrated_model_dir)

        elif calibration_name == 'METHOD_3_self':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_3(subject_code, IMU_type, calibrated_model_dir)

        elif calibration_name == 'METHOD_4a':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_4a(subject_code, IMU_type)

        else:
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = None, None, None
            print('Method not defined properly.')
            quit()

        # Create the calibrated model, applying the calculated offsets to the default model
        apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file,
                           calibrated_model_dir)


""" RUN THE CALIBRATION """

subject_list = [f'P{i}' for i in range(1, 2)]
IMU_type_list = ['Perfect']
method_name_list = ['METHOD_4a']

for subject_code in subject_list:
    for IMU_type in IMU_type_list:
        for calibration_name in method_name_list:
            run_method(subject_code, IMU_type, calibration_name)
