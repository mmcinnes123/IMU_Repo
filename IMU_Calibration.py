# This script takes an uncalibrated model, applies the chosen calibration method, and outputs a calibrated model
# Files required in same folder as this script:
# Template model file
# Calibration settings template xml file

from IMU_Calibration_helpers import get_calibrated_model_dir
from IMU_Calibration_helpers import get_cal_ori_file_path
from IMU_Calibration_helpers import osim_calibrate_model
from Custom_Calibration_helpers import apply_cal_to_model
from Custom_Calibration_helpers import custom_calibrate_model
from Custom_Calibration_helpers import get_IMU_offsets_METHOD_3

# Read in the template model
template_model_file = 'das3.osim'

# Template for running all iterations
trial_name1 = 'CP'   # Specify which trial to use for calibration pose
trial_name2 = 'JA_Slow'   # Specify which trial to use for calibration pose
# subject_code_list = ['P1', 'P2', 'P3']
# IMU_type_list = ['Real', 'Perfect']
# osim_calibration_name_dict = {'OSIM_N_self': 'N_self', 'OSIM_Alt_self': 'Alt_self'}
# custom_calibration_name_dict = {'METHOD_2_Alt_self': 'Alt_self', 'ALL_MANUAL': 'Alt_self'}



""" QUICK SETTINGS """

# Define which subjects/IMU types/trial name you want to run the calibration for
subject_code_list = ['P1']
IMU_type_list = ['Perfect']

""" RUN THE OPENSIM CALIBRATIONS """
# osim_calibration_name_dict = {'OSIM_N_self': 'N_self', 'OSIM_Alt_self': 'Alt_self'}
#
# for calibration_name, pose_name in osim_calibration_name_dict.items():
#
#     # Check we've set the default pose of the template model correctly
#     pose_confirmation = input(
#         f"\nIs the default pose of the model set to match the expected subject pose ({pose_name})?: ")
#     if pose_confirmation == "No":
#         quit()
#
#     for subject_code in subject_code_list:
#         for IMU_type in IMU_type_list:
#             print(f'Creating {calibration_name} calibrated model for '
#                   f'subject {subject_code}, using {IMU_type} IMUs from trial: {trial_name1}')
#
#             # Get the inputs needed to run the calibration
#             cal_oris_file_path, calibrated_model_dir, model_file = \
#                 get_calibration_inputs(subject_code, trial_name1, IMU_type, calibration_name, pose_name)
#
#             # Run the opensim calibration
#             osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, model_file)


""" RUN THE CUSTOM CALIBRATIONS """
# custom_calibration_name_dict = {'ALL_MANUAL': 'Alt_self', 'METHOD_1_Alt_self': 'Alt_self', 'METHOD_2_Alt_self': 'Alt_self'}
custom_calibration_name_dict = {'METHOD_3': ['Alt_self', 'Alt2_self']}

for calibration_name, pose_names in custom_calibration_name_dict.items():

    # If there is more than 1 pose defined in the method's pose name dict:
    if len(pose_names) > 1:
        pose_name1 = pose_names[0]     # First pose used to do the pose-based calibration part
        pose_name2 = pose_names[1]     # This is the pose/time used in METHOD_3 to get the radius IMU projected on the humerus IMU
    else:
        pose_name1 = pose_names[0]
        pose_name2 = None

    # Check we've set the default pose of the template model correctly
    pose_confirmation = input(
        f"\nIs the default pose of the model set to match the expected subject pose ({pose_name1})?: ")
    if pose_confirmation == "No":
        quit()

    for subject_code in subject_code_list:
        for IMU_type in IMU_type_list:
            print(f'Creating {calibration_name} calibrated model for '
                  f'subject {subject_code}, using {IMU_type} IMUs from trial: {trial_name1}')


            # Get/make the directory to store the newly calibrated model
            calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, calibration_name)

            if calibration_name == 'METHOD_3':

                thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                    get_IMU_offsets_METHOD_3(subject_code, trial_name1, trial_name2,
                                             pose_name1, pose_name2, IMU_type, calibrated_model_dir)

            # Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
            apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file,
                               calibrated_model_dir)
