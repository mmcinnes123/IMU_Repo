# This script takes an uncalibrated model, applies the chosen calibration method, and outputs a calibrated model
# Files required in same folder as this script:
# Template model file
# Calibration settings template xml file


from constants import template_model_file
from helpers_calibration import get_calibrated_model_dir
from helpers_calibration import osim_calibrate_model
from helpers_calibration import get_IMU_offsets_ALL_MANUAL
from helpers_calibration import get_IMU_offsets_METHOD_1
from helpers_calibration import get_IMU_offsets_METHOD_2
from helpers_calibration import get_IMU_offsets_METHOD_3
from helpers_calibration import get_cal_ori_file_path
from helpers_calibration import apply_cal_to_model






""" SETTINGS """

# Define which subjects/IMU types/trial name you want to run the calibration for
subject_code_list = ['P6']
IMU_type_list = ['Perfect', 'Real']
calibration_name_dict = {'OSIM_N_self': ['N_self'], 'OSIM_Alt_self': ['Alt_self'],
                         'ALL_MANUAL': [''], 'METHOD_1_Alt_self': ['Alt_self'],
                         'METHOD_2_Alt_self': ['Alt_self'], 'METHOD_3': ['Alt_self', 'Alt2_self']}
trial_name1 = 'CP'   # Specify which trial to use for calibration pose
trial_name2 = 'CP'   # For methods which use two poses, specify the trial in which to find the pose data
    # For P1, P2, P3, trial_name2 should be JA_Slow, since Alt2 pose wasn't captured during CP
    # For all others, this should be CP


## Template for running all iterations
# calibration_name_dict = {'OSIM_N_self': ['N_self'], 'OSIM_Alt_self': ['Alt_self'],
#                          'ALL_MANUAL': [''], 'METHOD_1_Alt_self': ['Alt_self'],
#                          'METHOD_2_Alt_self': ['Alt_self'], 'METHOD_3': ['Alt_self', 'Alt2_self']}
# subject_code_list = ['P1', 'P2', 'P3', 'P4', 'P5']
# IMU_type_list = ['Real', 'Perfect']


""" MAIN """

for calibration_name, pose_names in calibration_name_dict.items():

    # Check we've set the default pose of the template model correctly
    pose_confirmation = input(
        f"\nIs the default pose of the model set to match the expected subject pose ({pose_names[0]})?: ")
    if pose_confirmation == "No":
        quit()

    for subject_code in subject_code_list:
        for IMU_type in IMU_type_list:

            print(f'Creating {calibration_name} calibrated model for '
                  f'subject {subject_code}, using {IMU_type} IMUs from trial: {trial_name1}')

            # Get/make the directory to store the newly calibrated model
            calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, calibration_name)

            """ RUN OPENSIM BUILT-IN CALIBRATION """

            if calibration_name == 'OSIM_N_self':
                # Get the path to the orientations file specific to the chosen pose
                cal_oris_file_path = get_cal_ori_file_path(subject_code, trial_name1, pose_names[0], IMU_type)
                # Run the opensim calibration
                osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, template_model_file)

            elif calibration_name == 'OSIM_Alt_self':
                # Get the path to the orientations file specific to the chosen pose
                cal_oris_file_path = get_cal_ori_file_path(subject_code, trial_name1, pose_names[0], IMU_type)
                # Run the opensim calibration
                osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, template_model_file)

            else:

                # If there is more than 1 pose defined in the method's pose name dict:
                if len(pose_names) > 1:
                    pose_name1 = pose_names[0]  # First pose used to do the pose-based calibration part
                    pose_name2 = pose_names[1]  # This is the pose/time used in METHOD_3 to get the radius IMU projected on the humerus IMU
                else:
                    pose_name1 = pose_names[0]
                    pose_name2 = None

                """ RUN CUSTOM CALIBRATION  
                
                    Definitions:
            
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
            
                        elif calibration_name == 'METHOD_3':
                            thorax_method = 'get_IMU_cal_POSE_BASED'
                            humerus_method = 'get_IMU_cal_hum_method_4'
                            radius_method = 'get_IMU_cal_MANUAL'
                """


                if calibration_name == 'ALL_MANUAL':
                    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                        get_IMU_offsets_ALL_MANUAL()

                elif calibration_name == 'METHOD_1_Alt_self':
                    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                        get_IMU_offsets_METHOD_1(subject_code, trial_name1, pose_name1, IMU_type, calibrated_model_dir)

                elif calibration_name == 'METHOD_2_Alt_self':
                    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                        get_IMU_offsets_METHOD_2(subject_code, trial_name1, pose_name1, IMU_type, calibrated_model_dir)

                elif calibration_name == 'METHOD_3':
                    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                        get_IMU_offsets_METHOD_3(subject_code, trial_name1, trial_name2,
                                                 pose_name1, pose_name2, IMU_type, calibrated_model_dir)

                else:
                    print('Calibration_name did not match pre-defined calibrations')
                    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = None, None, None


                # Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
                apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file,
                                   calibrated_model_dir)




""" TEST """

# Stand alone function for running a new calibration

def run_single_cal(subject_code, IMU_type, calibration_name, pose_name1, pose_name2):

    # Check we've set the default pose of the template model correctly
    pose_confirmation = \
        input(f"\nIs the default pose of the model set to match the expected subject pose ({pose_name1})?: ")
    if pose_confirmation == "No":
        quit()

    # Get/make the directory to store the newly calibrated model
    calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, calibration_name)

    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
    get_IMU_offsets_METHOD_3(subject_code, trial_name1, trial_name2,
                             pose_name1, pose_name2, IMU_type, calibrated_model_dir)

    apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file,
                   calibrated_model_dir)