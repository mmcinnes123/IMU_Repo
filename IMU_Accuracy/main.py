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
import pandas as pd
import os


""" SETTINGS """

# Input the correct time values for the start and end time of interest, and also the time at which elevation starts
# in the JA_Slow trial so that it can be discounted from Joint23 analysis
JA_Slow_subject_dict = {'P1': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65},
                        'P2': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65},
                        'P3': {'start_time': 0, 'end_time': 95, 'time_elevation_starts': 65}}

CP_subject_dict = {'P1': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30},
                   'P2': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30},
                   'P3': {'start_time': 0, 'end_time': 30, 'time_elevation_starts': 30}}

# Choose which subject you want to run the function for (e.g just P1, or all)
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