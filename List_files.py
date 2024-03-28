# Used to iterate through files


from IMU_IK import run_IMU_IK
from IMU_Analysis import run_analysis
from IK_Compare import run_IK_compare

# Quick Settings
subject_code = 'P3'
trial_name = 'JA_Slow'      # Choose which trial to run IK
orientations_file = 'IMU_Quats_all.sto'     # Specify which orientation .sto data to use
# calibration_list = ['ALL_MANUAL', 'ALL_POSE_BASED_N_self', 'ALL_POSE_BASED_Alt_self']     # Used to find the calibrated model file
calibration_list = ['ALL_MANUAL']     # Used to find the calibrated model file
IK_start_time = 0
IK_end_time = 104
IK_trim_bool = True
analysis_start_time = 0
analysis_end_time = 104
analysis_trim_bool = False
compare_start_time = 0
compare_end_time = 90

for calibration_name in calibration_list:

    # run_IMU_IK(subject_code, trial_name, orientations_file, calibration_name, IK_start_time, IK_end_time, IK_trim_bool)

    # run_analysis(subject_code, trial_name, calibration_name, analysis_start_time, analysis_end_time, analysis_trim_bool)

    run_IK_compare(subject_code, trial_name, calibration_name, compare_start_time, compare_end_time)