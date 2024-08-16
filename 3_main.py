# Used to iterate through files


from inverse_kinematics import run_IMU_IK
from analysis import run_analysis
from compare import run_IK_compare

# Quick Settings
trial_name = 'JA_Slow'      # Choose which trial to run IK

# calibration_list = ['OSIM_N_self', 'OSIM_Alt_self', 'ALL_MANUAL', 'METHOD_1_Alt_self', 'METHOD_2_Alt_self', 'METHOD_3']     # Used to find the calibrated model file
# calibration_list = ['OSIM_N_self', 'OSIM_N_asst', 'OSIM_Alt_asst', 'OSIM_Alt_self', 'METHOD_4b']     # Used to find the calibrated model file
calibration_list = ['METHOD_7_ISO_1rep']     # Used to find the calibrated model file

IMU_type_list = ['Real']        # Options: 'Perfect' or 'Real'

subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
# subject_list = [f'P{str(i).zfill(3)}' for i in range(15, 21)]
# subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21) if f'P{str(i).zfill(3)}' not in 'P019']
# subject_list = ['P014']

IK_start_at_pose_bool = True
IK_trim_bool = False
IK_start_time = 7
IK_end_time = 10

analysis_trim_bool = False
analysis_start_time = 0
analysis_end_time = 104

compare_trim_bool = False
compare_start_time = 0
compare_end_time = 103

for calibration_name in calibration_list:

    for IMU_type in IMU_type_list:

        for subject_code in subject_list:

            # run_IMU_IK(subject_code, trial_name, calibration_name, IK_start_time, IK_end_time, IK_start_at_pose_bool, IK_trim_bool, IMU_type)
            # run_analysis(subject_code, trial_name, calibration_name, analysis_start_time, analysis_end_time, analysis_trim_bool, IMU_type)
            run_IK_compare(subject_code, trial_name, calibration_name, compare_start_time, compare_end_time, compare_trim_bool, IMU_type, test=False)



