# This script batch runs the IMU IK, analysis, and comparison scripts for a given subject, calibration method, and movement trial
# Ot relies on the calibrated model already created
# Read run_IMU_IK, run_analysis, and run_IK_compare to find out what each function does

from inverse_kinematics import run_IMU_IK
from analysis import run_analysis
from compare import run_IK_compare
from compileJAs import run_compile_JAs

# TODO: Update data_dir in constants.py to point to the correct overarching data directory

""" SETTINGS"""

# Choose which trial to run the IMU IK, analysis and comparison for
trial_name = 'JA_Slow'      # Choose which trial to run IK


# Available calibration types:
# 1. Basic OSIM Types:
#    - 'OSIM_N_self'     # Neutral pose, self-performed
#    - 'OSIM_N_asst'     # Neutral pose, assisted
#    - 'OSIM_Alt_self'   # Alternative pose, self-performed
#    - 'OSIM_Alt_asst'   # Alternative pose, assisted
#
# 2. Method-specific Types:
#    - 'METHOD_1_Alt_self'
#    - 'METHOD_2_Alt_self'
#    - 'METHOD_3'
#    - 'METHOD_4b'
#    - 'METHOD_7_ISO_1rep'
#
# 3. Special Types:
#    - 'ALL_MANUAL'         # Full manual calibration

# Choose which calibrated model type(s) to run the IMU IK, analysis and comparison for
calibration_list = ['METHOD_7_ISO_1rep', 'OSIM_N_self']     # Used to find the calibrated model file

IMU_type_list = ['Perfect', 'Real']        # Options: 'Perfect' or 'Real'

# Choose which subjects to run the chosen functions for
from_subject = 1
to_subject = 20
subject_list = [f'P{str(i).zfill(3)}' for i in range(from_subject, (to_subject+1))]


# Use these if you want to change the start and end time used for the IK, analysis or comparison.
# (Default is to use the pose which marks the start of the movements of interest as the marker to start the
# inverse kinematics, and to use the full length of IK .mot data)
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
            run_compile_JAs(subject_code, trial_name, calibration_name, IMU_type)
            # run_IK_compare(subject_code, trial_name, calibration_name, compare_start_time, compare_end_time, compare_trim_bool, IMU_type, test=False)



