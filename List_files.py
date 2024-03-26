# Used to iterate through files


from IMU_IK import run_IMU_IK
from IMU_Analysis import run_analysis
from IK_Compare import run_IK_compare

# Quick Settings
subject_code = 'P2'
trial_name = 'JA_Slow'      # Choose which trial to run IK
orientations_file = 'Cluster_Quats_all.sto'     # Specify which orientation .sto data to use
calibration_list = ['ALL_MANUAL', 'ALL_POSE_BASED_N_asst', 'ALL_POSE_BASED_Alt_asst']     # Used to find the calibrated model file

for calibration_name in calibration_list:
    run_IMU_IK(subject_code, trial_name, orientations_file, calibration_name)


for calibration_name in calibration_list:
    run_analysis(subject_code, trial_name, calibration_name)


start_time = 0
end_time = 100

for calibration_name in calibration_list:
    run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time)