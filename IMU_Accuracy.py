# A script to compare raw IMU orientation data with marker cluster data
# Comparing 'real' vs 'perfect' IMUs gives an indication of IMU error
# Relative orientation of two IMUs is assessed, as opposed to absolute orientation of one IMU
import numpy as np
import pandas as pd

from functions import *
import os
from scipy.spatial.transform import Rotation as R

start_time = 0
end_time = 95
time_elevation_starts = 65  # This is used so that humerus-forearm error is only calculated when arm is at side
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'


def run_IMU_Accuracy(subject_code, start_time, end_time, time_elevation_starts):

    """ SETTINGS """

    # Quick Settings
    trial_name = 'JA_Slow'
    trim_bool = True
    sample_rate = 100

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    raw_data_dir = os.path.join(parent_dir, 'RawData')
    IMU_type_dict = {'Real': ' - Report2 - IMU_Quats.txt', 'Perfect': ' - Report3 - Cluster_Quats.txt'}
    IMU_input_file = subject_code + '_' + trial_name + IMU_type_dict['Real']
    OMC_input_file = subject_code + '_' + trial_name + IMU_type_dict['Perfect']
    IMU_input_file_path = os.path.join(raw_data_dir, IMU_input_file)
    OMC_input_file_path = os.path.join(raw_data_dir, OMC_input_file)
    accuracy_results_dir = os.path.join(parent_dir, 'IMU_Accuracy_Results')
    if os.path.exists(accuracy_results_dir) == False:
        os.mkdir(accuracy_results_dir)
    # Settings for writing to csv/sto
    APDM_template_file = "APDM_template_4S.csv"
    APDM_settings_file = "APDMDataConverter_Settings.xml"


    # Local misalignment rotations (calculated in Frame_Calibration.py)
    # IMU1_local_misalignment = R.from_quat([-1.200e-03, 1.000e-04, 1.010e-02, 9.999e-01])
    # IMU2_local_misalignment = R.from_quat([4.e-04, 3.e-03, 2.e-04, 1.e+00])
    # IMU3_local_misalignment = R.from_quat([0.0037, 0.0023, 0.0113, 0.9999])

    # Apply no local alingment for Participants where no alignment data is available
    IMU1_local_misalignment = R.from_quat([0, 0, 0, 1])
    IMU2_local_misalignment = R.from_quat([0, 0, 0, 1])
    IMU3_local_misalignment = R.from_quat([0, 0, 0, 1])


    """ GET RELATIVE ORIENTATIONS """

    # Read in IMU/Cluster orientation data from trial of interest
    IMU1, IMU2, IMU3 = get_scipyR_from_txt_file(IMU_input_file_path, trim_bool, start_time, end_time, sample_rate)
    OMC1, OMC2, OMC3 = get_scipyR_from_txt_file(OMC_input_file_path, trim_bool, start_time, end_time, sample_rate)

    # Apply local misalignment rotations to the cluster CFs to better align them with the IMU frames
    OMC1 = IMU1_local_misalignment * OMC1
    OMC2 = IMU2_local_misalignment * OMC2
    OMC3 = IMU3_local_misalignment * OMC3

    # Get the relative orientation between IMUs/Clusters
    IMU_joint12 = IMU1.inv() * IMU2
    IMU_joint23 = IMU2.inv() * IMU3
    IMU_joint13 = IMU1.inv() * IMU3
    OMC_joint12 = OMC1.inv() * OMC2
    OMC_joint23 = OMC2.inv() * OMC3
    OMC_joint13 = OMC1.inv() * OMC3

    """ WRITE ROTATED ORIENTATIONS TO STO FOR VISUALISATION IN OPENSIM """

    # For visualisation, find heading offset between thorax frames and apply to all IMUs
    heading_rotation = get_heading_rotation_using_2D_vecs(IMU1, OMC1)
    IMU1_globally_aligned = heading_rotation * IMU1
    IMU2_globally_aligned = heading_rotation * IMU2
    IMU3_globally_aligned = heading_rotation * IMU3

    # Write to .sto file for visualisation
    write_rot_quats_to_sto(IMU1_globally_aligned, IMU2_globally_aligned, IMU3_globally_aligned,
                           trial_name, APDM_template_file, APDM_settings_file, accuracy_results_dir, IMU_type='Real')
    write_rot_quats_to_sto(OMC1, OMC2, OMC3,
                           trial_name, APDM_template_file, APDM_settings_file, accuracy_results_dir, IMU_type='Perfect')

    """ COMPARE """

    # Get the single-angle geodesic distance between Joint_R IMU and Joint_R OMC
    # Rotational difference between IMU and OMC joint:
    joint12_rot_diff = IMU_joint12.inv() * OMC_joint12
    joint23_rot_diff = IMU_joint23.inv() * OMC_joint23
    joint13_rot_diff = IMU_joint13.inv() * OMC_joint13
    # Magnitude of that rotational difference:
    joint12_dist_error = joint12_rot_diff.magnitude() * 180/np.pi
    joint23_dist_error = joint23_rot_diff.magnitude() * 180/np.pi
    joint13_dist_error = joint13_rot_diff.magnitude() * 180/np.pi
    # Plot the time-series distance error and get the RMSEs
    joint12_dist_error_RMSE, joint23_dist_error_RMSE, joint13_dist_error_RMSE = \
        plot_single_angle_diff(joint12_dist_error, joint23_dist_error, joint13_dist_error, start_time, end_time, accuracy_results_dir)

    # Get the projected vector angles in the transverse, frontal, and sagittal planes, and compare IMU with OMC
    """
    For joint12 (thorax-humerus), projected vectors are defined as follows:
        Transverse plane: z_axis relative to Z on the XZ plane
        Frontal plane: y_axis relative to Y on the XY plane
        Sagittal plane: y_axis relative to Z on the YZ plane
    
    For joint23 (humerus-forearm), projected vectors are defined as follows:
        Transverse plane: y_axis relative to Z on the XZ plane
        Frontal plane: z_axis relative to Y on the ZY plane
        Sagittal plane: y_axis relative to Y on the XY plane
    
    For joint13 (thorax-forearm), projected vectors are defined as follows:
        Transverse plane: y_axis relative to X on the XZ plane
        Frontal plane: z_axis relative to Y on the YX plane
        Sagittal plane: y_axis relative to Z on the YZ plane
            """""
    joint12_axis_dict = {'transverse': {'local_axis': 'z', 'global_plane': 'XZ', 'global_axis': 'Z'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'XY', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'YZ', 'global_axis': 'Z'}}

    joint23_axis_dict = {'transverse': {'local_axis': 'y', 'global_plane': 'XZ', 'global_axis': 'Z'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'ZY', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'XY', 'global_axis': 'Y'}}

    joint13_axis_dict = {'transverse': {'local_axis': 'y', 'global_plane': 'XZ', 'global_axis': 'X'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'YX', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'YZ', 'global_axis': 'Z'}}

    # For joint12, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plotting)
    joint12_plotting_dict = get_errors_and_plotting_dict(joint12_axis_dict, IMU_joint12, OMC_joint12, start_time, end_time)
    plot_vec_angles_error(joint12_plotting_dict, start_time, end_time, accuracy_results_dir, joint_of_interest='Joint12')

    # For joint23, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plotting)
    joint_23_end_time = time_elevation_starts   # Trim so that we don't include data when arm is elevated (keeps sagittal/frontal etc definition valid)
    joint23_plotting_dict = get_errors_and_plotting_dict(joint23_axis_dict, IMU_joint23, OMC_joint23, start_time, joint_23_end_time)
    plot_vec_angles_error(joint23_plotting_dict, start_time, joint_23_end_time, accuracy_results_dir, joint_of_interest='Joint23')

    # For joint13, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plotting)
    joint13_plotting_dict = get_errors_and_plotting_dict(joint13_axis_dict, IMU_joint13, OMC_joint13, start_time, end_time)
    plot_vec_angles_error(joint13_plotting_dict, start_time, end_time, accuracy_results_dir, joint_of_interest='Joint13')


    """ WRITE RESULTS TO CSV """

    final_results = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_name],
                            'Joint12_Dist_Error_RMSE': joint12_dist_error_RMSE,
                            'Joint23_Dist_Error_RMSE': joint23_dist_error_RMSE,
                            'Joint13_Dist_Error_RMSE': joint13_dist_error_RMSE,
                            'Joint12_Transverse_Error_RMSE': joint12_plotting_dict['transverse']['RMSE'][0],
                            'Joint23_Transverse_Error_RMSE': joint23_plotting_dict['transverse']['RMSE'][0],
                            'Joint13_Transverse_Error_RMSE': joint13_plotting_dict['transverse']['RMSE'][0],
                            'Joint12_Frontal_Error_RMSE': joint12_plotting_dict['frontal']['RMSE'][0],
                            'Joint23_Frontal_Error_RMSE': joint23_plotting_dict['frontal']['RMSE'][0],
                            'Joint13_Frontal_Error_RMSE': joint13_plotting_dict['frontal']['RMSE'][0],
                            'Joint12_Sagittal_Error_RMSE': joint12_plotting_dict['sagittal']['RMSE'][0],
                            'Joint23_Sagittal_Error_RMSE': joint23_plotting_dict['sagittal']['RMSE'][0],
                            'Joint13_Sagittal_Error_RMSE': joint13_plotting_dict['sagittal']['RMSE'][0]})

    final_results.to_csv(accuracy_results_dir + "\\" + subject_code + '_' + trial_name + r"_IMU_accuracy_RMSEs.csv",
                    mode='w', encoding='utf-8', na_rep='nan')

    return final_results


""" COMPILE RESULTS """

# Run the whole function above
# P1_results_row = run_IMU_Accuracy('P1', start_time=0, end_time=95, time_elevation_starts=65)
P2_results_row = run_IMU_Accuracy('P2', start_time=0, end_time=95, time_elevation_starts=65)
# P3_results_row = run_IMU_Accuracy('P3', start_time=0, end_time=95, time_elevation_starts=65)

# # Print all the results to csv
# all_data = pd.concat([P1_results_row, P2_results_row, P3_results_row], ignore_index=True)
# all_data.to_csv(os.path.join(directory, 'R Analysis', 'All_IMU_Accuracy_Results_forR.csv'))
