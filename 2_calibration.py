

from helpers_calibration import get_cal_ori_file_path
from helpers_calibration import osim_calibrate_model
from helpers_calibration import get_calibrated_model_dir
from helpers_calibration import apply_cal_to_model
from helpers_calibration import get_IMU_offsets_ALL_MANUAL
from helpers_calibration import get_IMU_offsets_METHOD_1
from helpers_calibration import get_IMU_offsets_METHOD_2
from helpers_calibration import get_IMU_offsets_METHOD_3
from helpers_calibration import get_IMU_offsets_METHOD_4a
from helpers_calibration import get_IMU_offsets_METHOD_4c
from helpers_calibration import get_IMU_offsets_METHOD_7
from helpers_calibration import get_IMU_offsets_METHOD_5
from helpers_calibration import set_default_model_pose
from constants import template_model_file


""" CALIBRATION OPTIONS """


def run_method(method_name, subject_code, IMU_type):

    print(f'\nRunning method {method_name} with {IMU_type} IMUs for subject: {subject_code}')

    if method_name.startswith('OSIM'):

        trial_name = 'CP'

        if method_name == 'OSIM_N_self':
            pose_name = 'N_self'
        elif method_name == 'OSIM_N_asst':
            pose_name = 'N_asst'
        elif method_name == 'OSIM_Alt_self':
            pose_name = 'Alt_self'
        elif method_name == 'OSIM_Alt_asst':
            pose_name = 'Alt_asst'
        else:
            pose_name = None

        # Get the path to the orientations file specific to the chosen pose
        cal_oris_file_path = get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type)

        # Set the template model pose
        set_default_model_pose(template_model_file, pose_name)

        # Get/make the directory to save the calibrated model
        calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, method_name)

        # Run the opensim calibration
        osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, template_model_file)

    else:

        # Get/make the directory to store the newly calibrated model
        calibrated_model_dir = get_calibrated_model_dir(subject_code, IMU_type, method_name)

        if method_name == 'ALL_MANUAL':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_ALL_MANUAL()

        elif method_name == 'METHOD_1_self':
            pose_name = 'Alt_self'
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_1(subject_code, pose_name, IMU_type, calibrated_model_dir)

        elif method_name == 'METHOD_2_self':
            pose_name = 'Alt_self'
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_2(subject_code, IMU_type, pose_name, calibrated_model_dir)

        elif method_name == 'METHOD_3_self':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_3(subject_code, IMU_type, calibrated_model_dir)

        elif method_name == 'METHOD_4a':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_4a(subject_code, IMU_type)

        elif method_name == 'METHOD_4c':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_4c(subject_code, IMU_type)

        elif method_name == 'METHOD_5':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_5(subject_code, IMU_type)

        elif method_name == 'METHOD_7_ISO_5reps':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_7(subject_code, IMU_type,
                                         opt_trial_name='JA_Slow',
                                         event_to_start='FE_start', event_to_end='PS_end')

        elif method_name == 'METHOD_7_ISO_1rep':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_7(subject_code, IMU_type,
                                         opt_trial_name='JA_Slow',
                                         event_to_start='FE5_start', event_to_end='PS2_start')

        elif method_name == 'METHOD_7_ADL_both':
            opt_trial_name = 'ADL'
            if subject_code == 'P008':
                event_to_start = 'drink1_start'
                event_to_end = 'kettle1_end'
            else:
                event_to_start = 'kettle1_start'
                event_to_end = 'drink1_end'
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_7(subject_code, IMU_type,
                                         opt_trial_name,
                                         event_to_start, event_to_end)

        elif method_name == 'METHOD_7_ADL_drink':
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
                get_IMU_offsets_METHOD_7(subject_code, IMU_type,
                                         opt_trial_name='ADL',
                                         event_to_start='drink1_start', event_to_end='drink1_end')

        else:
            thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = None, None, None
            print('Method not defined properly.')
            quit()

        # Create the calibrated model, applying the calculated offsets to the default model
        apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, template_model_file,
                           calibrated_model_dir)


""" RUN THE CALIBRATION """

from_subject = 2
to_subject = 20
subject_list = [f'P{str(i).zfill(3)}' for i in range(from_subject, (to_subject+1))]
IMU_type_list = ['Perfect']
# method_name_list = ['OSIM_N_self', 'OSIM_N_asst', 'OSIM_Alt_asst', 'OSIM_Alt_self', 'METHOD_4b']
method_name_list = ['METHOD_7_ISO_1rep']

for method_name in method_name_list:

    if method_name.startswith('OSIM'):
        check_default_pose = input(f"\tUsing pose: {method_name}, has the model default pose been set correctly "
                                   f"based on desired HT angles? ")
        if check_default_pose == 'No':
            quit()

    for IMU_type in IMU_type_list:
        for subject_code in subject_list:
            run_method(method_name, subject_code, IMU_type)


