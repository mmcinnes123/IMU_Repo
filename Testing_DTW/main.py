
from helpers import run_IK_compare
from helpers import read_in_mot_as_numpy

subject_code = 'P3'
trial_name = 'JA_Fast'
calibration_name = 'METHOD2_Alt_self'
start_time = 15
end_time = 17
trim_bool = True
IMU_type = 'Real'

# Use my functions for printing time-series JAs and calculating RMSEs and Pearson's R
run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)


# Read in same data as np arrays, ready for use in new code
OMC_angle_np, IMU_angle_np = read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

