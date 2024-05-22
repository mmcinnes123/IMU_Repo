# Used to iterate through files


from inverse_kinematics import run_IMU_IK
from analysis import run_analysis
from compare import run_IK_compare

# Quick Settings
subject_list = ['P5']
IMU_type_list = ['Real', 'Perfect']        # Options: 'Perfect' or 'Real'
trial_name = 'JA_Slow'      # Choose which trial to run IK
# calibration_list = ['OSIM_N_self', 'OSIM_Alt_self', 'ALL_MANUAL', 'METHOD_1_Alt_self', 'METHOD_2_Alt_self', 'METHOD_3']     # Used to find the calibrated model file
calibration_list = ['ALL_MANUAL', 'METHOD_1_Alt_self', 'METHOD_2_Alt_self', 'METHOD_3']     # Used to find the calibrated model file
# calibration_list = ['METHOD_3']     # Used to find the calibrated model file

IK_trim_bool = False
IK_start_time = 40
IK_end_time = 80

analysis_trim_bool = False
analysis_start_time = 0
analysis_end_time = 104

compare_trim_bool = False
compare_start_time = 0
compare_end_time = 103

for calibration_name in calibration_list:

    for IMU_type in IMU_type_list:

        for subject_code in subject_list:

            # run_IMU_IK(subject_code, trial_name, calibration_name, IK_start_time, IK_end_time, IK_trim_bool, IMU_type)

            # run_analysis(subject_code, trial_name, calibration_name, analysis_start_time, analysis_end_time, analysis_trim_bool, IMU_type)

            run_IK_compare(subject_code, trial_name, calibration_name, compare_start_time, compare_end_time, compare_trim_bool, IMU_type)



