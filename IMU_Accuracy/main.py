# A script to compare raw IMU orientation data with marker cluster data
"""

Comparing 'real' vs 'perfect' IMUs gives an indication of IMU error
Relative orientation of two IMUs is assessed, as opposed to absolute orientation of one IMU
Function: run_IMU_accuracy reads in IMU and cluster orientations, gets the 'joint rotations' which describes the
relative orientation of IMU1-2, IMU2-3, IMU1-3, then assess the difference between IMU and OMC joint rotations.
Measures of error are:
    1. Single angle quaternion 'geodesic' distance
    2. Difference in projected vector angles, e.g angle between proximal and distal frames in the:
        a) transverse plane
        b) frontal plane
        c) sagittal plane
Output of the function is a .png plot of all the error metrics, and the RMSE values.

"""

from helpers import run_IMU_Accuracy
from helpers import test_run_IMU_accuracy
import pandas as pd
import os
from scipy.spatial.transform import Rotation as R

# TODO: Include local misalignments specific to each IMU and each trial

""" SETTINGS """

# Input the correct time values for the start and end time of interest, and also the time at which elevation starts
# in the JA_Slow trial so that it can be discounted from Joint23 analysis
JA_Slow_subject_dict = {'P1': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65},
                        'P2': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65},
                        'P3': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65}}

CP_subject_dict = {'P1': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30},
                   'P2': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30},
                   'P3': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30}}

# Choose which subject_code you want to run the function for (e.g just P1, or all)
subject_list = ['P1']

""" MAIN """

# Instantiate a dataframe to compile all subjects into one
JA_Slow_all_data = pd.DataFrame(columns=['Subject', 'Trial',
                            'Joint12_Dist_Error_RMSE',
                            'Joint23_Dist_Error_RMSE',
                            'Joint13_Dist_Error_RMSE',
                            'Joint12_Transverse_Error_RMSE',
                            'Joint23_Transverse_Error_RMSE',
                            'Joint13_Transverse_Error_RMSE',
                            'Joint12_Frontal_Error_RMSE',
                            'Joint23_Frontal_Error_RMSE',
                            'Joint13_Frontal_Error_RMSE',
                            'Joint12_Sagittal_Error_RMSE',
                            'Joint23_Sagittal_Error_RMSE',
                            'Joint13_Sagittal_Error_RMSE'])


# Run function for CP trial
for subject in subject_list:
    CP_row = run_IMU_Accuracy(subject,
                              CP_subject_dict[subject]['start_time'],
                              CP_subject_dict[subject]['end_time'],
                              CP_subject_dict[subject]['time_elevation_starts'],
                              trial_name='CP')

# Run function for JA_Slow trial
for subject in subject_list:
    JA_Slow_row = run_IMU_Accuracy(subject,
                                   JA_Slow_subject_dict[subject]['start_time'],
                                   JA_Slow_subject_dict[subject]['end_time'],
                                   JA_Slow_subject_dict[subject]['time_elevation_starts'],
                                   trial_name='JA_Slow')
    JA_Slow_all_data = pd.concat([JA_Slow_all_data, JA_Slow_row], ignore_index=True)


# Print compiled JA_Slow results to csv
R_results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\R Analysis'
JA_Slow_all_data.to_csv(os.path.join(R_results_dir, 'All_IMU_Accuracy_Results_forR.csv'))



""" TEST """

run_test = False

if run_test == True:

    IMU_input_file_path =  r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Frame Calibration Verification\RawData\CAL_offset_test - Report2 - IMU_Quats.txt'
    OMC_input_file_path = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Frame Calibration Verification\RawData\CAL_offset_test - Report3 - Cluster_Quats.txt'
    trim_bool = False
    start_time = 0
    end_time = 100
    sample_rate = 100
    IMU1_local_misalignment = R.from_quat([-0.0143, -0.1127, 0.0224, 0.9933])
    IMU2_local_misalignment = R.from_quat([-0.0356, -0.0017, 0.0088, 0.9993])
    IMU3_local_misalignment = R.from_quat([-0.0276, -0.0031, -0.0058, 0.9996])
    trial_name = 'CAL'
    APDM_template_file = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo', "APDM_template_4S.csv")
    APDM_settings_file = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo',
                                      "APDMDataConverter_Settings.xml")
    trial_results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Frame Calibration Verification'
    time_elevation_starts = 108
    subject_code = 'CAL_offset_test'

    test_run_IMU_accuracy(IMU_input_file_path, OMC_input_file_path, trim_bool, start_time, end_time, sample_rate,
                          IMU1_local_misalignment, IMU2_local_misalignment, IMU3_local_misalignment,
                          trial_name, APDM_template_file, APDM_settings_file, trial_results_dir, time_elevation_starts, subject_code)