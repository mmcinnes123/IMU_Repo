# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import os
import pandas as pd
from IK_Compare_helpers import *


def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100

    # Define some file names
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    IMU_type_dir = os.path.join(parent_dir, IMU_type)
    IMU_IK_results_dir = os.path.join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
    OMC_IK_results_dir = os.path.join(parent_dir, 'OMC', trial_name + '_IK_Results')
    results_dir = os.path.join(IMU_IK_results_dir, "Comparison_" + compare_name)
    IMU_analysis_sto_path = os.path.join(IMU_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
    OMC_analysis_sto_path = os.path.join(OMC_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
    IMU_mot_file = os.path.join(IMU_IK_results_dir, 'IMU_IK_results.mot')
    OMC_mot_file = os.path.join(OMC_IK_results_dir, 'OMC_IK_results.mot')

    if os.path.exists(results_dir) == False:
        os.mkdir(results_dir)


    """ READ IN DATA """

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)

    # Set start and end time if trim_bool is false
    if trim_bool == False:
        start_time = 0
        end_time = (IMU_table.getNumRows() - 1) / sample_rate

    # Read in body orientations from newly created csv files (as trimmed np arrays (Nx4))
    print('Getting model body orientations from .sto files...')
    thorax_IMU, humerus_IMU, radius_IMU = get_body_quats_from_analysis_sto(IMU_analysis_sto_path, start_time, end_time)
    thorax_OMC, humerus_OMC, radius_OMC = get_body_quats_from_analysis_sto(OMC_analysis_sto_path, start_time, end_time)
    heading_offset = find_heading_offset(thorax_OMC, thorax_IMU)

    # Trim tables based on time of interest
    OMC_table.trim(start_time, end_time)
    IMU_table.trim(start_time, end_time)

    # Account for discrepancies between trimming function/time values
    n = OMC_table.getNumRows() - IMU_table.getNumRows()     # Check if tables are different lengths
    OMC_table, IMU_table = trim_tables_if_diff_lengths(n, OMC_table, IMU_table)    # Trim the tables to the same length

    # Trim the body ori data to same length for ease of plotting
    thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC = \
        trim_body_ori_data_to_same_length(n, thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC)

    time = np.array(OMC_table.getIndependentColumn())  # Get the time data

    """ ANALYSE ELBOW AND THORAX COORDS """

    # Plot and calculate errors for all the coordinates in the states file

    # Instantiate error dicts to write results into
    RMSE_results_dict = {'thorax_ori': None, 'humerus_ori': None, 'radius_ori': None,
                         'thorax_forward_tilt': None, 'thorax_lateral_tilt': None, 'thorax_rotation': None,
                         'elbow_flexion': None, 'elbow_pronation': None,
                         'HT_abd': None, 'HT_flexion': None, 'HT_rotation': None}
    R_results_dict = {'thorax_forward_tilt': None, 'thorax_lateral_tilt': None, 'thorax_rotation': None,
                      'elbow_flexion': None, 'elbow_pronation': None,
                      'HT_abd': None, 'HT_flexion': None, 'HT_rotation': None}
    peakROM_results_dict = {'thorax_forward_tilt': None, 'thorax_lateral_tilt': None, 'thorax_rotation': None,
                      'elbow_flexion': None, 'elbow_pronation': None,
                      'HT_abd': None, 'HT_flexion': None, 'HT_rotation': None}
    troughROM_results_dict = {'thorax_forward_tilt': None, 'thorax_lateral_tilt': None, 'thorax_rotation': None,
                      'elbow_flexion': None, 'elbow_pronation': None,
                      'HT_abd': None, 'HT_flexion': None, 'HT_rotation': None}

    # Define a dict with labels:keys, for reading in all the coordinates of interest from the states table
    OSim_coords_joint_ref_dict = {'TH_x': 'thorax_forward_tilt', 'TH_z': 'thorax_lateral_tilt',
                                  'TH_y': 'thorax_rotation', 'EL_x': 'elbow_flexion', 'PS_y': 'elbow_pronation'}

    # Iterate through the joint angles specified in the dict, calculating RMSE, R, and plotting results
    print('Plotting results...')
    for key, value in OSim_coords_joint_ref_dict.items():

        # Extract coordinates from states table
        OMC_angle = OMC_table.getDependentColumn(key).to_numpy()
        IMU_angle = IMU_table.getDependentColumn(key).to_numpy()

        # Calcualte error metrics and plot
        RMSE, R, mean_peak_error, mean_trough_error = plot_compare_any_JAs(OMC_angle, IMU_angle, time, start_time, end_time, results_dir, joint_name=value)

        # Add RMSE and R values into the results dicts
        RMSE_results_dict[value] = RMSE
        R_results_dict[value] = R
        peakROM_results_dict[value] = mean_peak_error
        troughROM_results_dict[value] = mean_trough_error


    """ ANALYSE HT ANGLES """

    # Plot and calculate errors for all the HT angles, calculated from relative body oris

    # Calculate the projected vector angles based on the body orientations of thorax and humerus
    abduction_all_OMC, flexion_all_OMC, rotation_elbow_down_all_OMC, rotation_elbow_up_all_OMC = \
        get_vec_angles_from_two_CFs(thorax_OMC, humerus_OMC)
    abduction_all_IMU, flexion_all_IMU, rotation_elbow_down_all_IMU, rotation_elbow_up_all_IMU = \
        get_vec_angles_from_two_CFs(thorax_IMU, humerus_IMU)

    OMC_angle_dict = {'HT_abd': abduction_all_OMC, 'HT_flexion': flexion_all_OMC, 'HT_rotation': rotation_elbow_down_all_OMC}
    IMU_angle_dict = {'HT_abd': abduction_all_IMU, 'HT_flexion': flexion_all_IMU, 'HT_rotation': rotation_elbow_down_all_IMU}
    HT_joint_ref_dict = {'HT_abd': 'Shoulder_Abduction', 'HT_flexion': 'Shoulder_Flexion', 'HT_rotation': 'Shoulder_Rotation'}

    # Iterate through the joint angles specified in the dict, calculating RMSE, R, and plotting results
    for key, value in HT_joint_ref_dict.items():

        OMC_angle = OMC_angle_dict[key]
        IMU_angle = IMU_angle_dict[key]

        # Calcualte error metrics and plot
        RMSE, R, mean_peak_error, mean_trough_error = plot_compare_any_JAs(OMC_angle, IMU_angle, time, start_time, end_time, results_dir,
                                   joint_name=key)

        # Add RMSE and R values into the results dicts
        RMSE_results_dict[key] = RMSE
        R_results_dict[key] = R
        peakROM_results_dict[key] = mean_peak_error
        troughROM_results_dict[key] = mean_trough_error

    """ ANALYSE MODEL BODY ORIENTATIONS """

    RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
        plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                           heading_offset, time, start_time, end_time, results_dir)

    RMSE_results_dict['thorax_ori'] = RMSE_thorax_ori
    RMSE_results_dict['humerus_ori'] = RMSE_humerus_ori
    RMSE_results_dict['radius_ori'] = RMSE_radius_ori



    """ WRITE RESULTS """

    final_RMSE_values_df = pd.DataFrame.from_dict(RMSE_results_dict, orient='index', columns=["RMSE"])
    final_R_values_df = pd.DataFrame.from_dict(R_results_dict, orient='index', columns=["R"])
    final_peakROM_values_df = pd.DataFrame.from_dict(peakROM_results_dict, orient='index', columns=["peakROM"])
    final_troughROM_values_df = pd.DataFrame.from_dict(troughROM_results_dict, orient='index', columns=["troughROM"])
    all_data = pd.concat((final_RMSE_values_df, final_R_values_df, final_peakROM_values_df, final_troughROM_values_df), axis=1)

    # Write final RMSE values to a csv
    print('Writing results to .csv.')
    all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                    mode='w', encoding='utf-8', na_rep='nan')



if __name__ == '__main__':
    run_IK_compare('P3', 'JA_Slow', 'METHOD_2_Alt_self', 0, 90, True, 'Real')

