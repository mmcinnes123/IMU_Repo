# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import os

import pandas as pd

from functions import *

def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100
    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

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
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(results_dir + r"\opensim.log")


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

    # Account for discrepancies between trimming function/time values#
    n = OMC_table.getNumRows() - IMU_table.getNumRows()     # Check if tables are different lengths
    trim_tables_if_diff_lengths(n, OMC_table, IMU_table)    # Trim the tables to the same length
    # Trim the body ori data to same length for ease of plotting
    thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC = \
        trim_body_ori_data_to_same_length(n, thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC)

    time = OMC_table.getIndependentColumn()  # Get the time data

    """ GET TIME SERIES JOINT ANGLES """

    # Define a dict with labels:keys, for reading in all the coordinates of interest from the states table
    joint_ref_dict = {'TH_x': 'Forward_Tilt', 'TH_z': 'Lateral_Tilt', 'TH_y': 'Trunk_Rotation',
                              'EL_x': 'Elbow_Flexion', 'PS_y': 'Pronation'}

    # Plot and calculate errors for all the coordinates in the states file
    for key in joint_ref_dict.keys():

        # Extract coordinates from states table
        OMC_angle = OMC_table.getDependentColumn(key).to_numpy()
        IMU_angle = IMU_table.getDependentColumn(key).to_numpy()

        plot_compare_any_JAs(OMC_angle, IMU_angle, time, start_time, end_time, results_dir, joint_name=joint_ref_dict[key])


    """ PLOT """

    # Plot IMU vs OMC joint angles based on OpenSim coordinates
    # print('Plotting results...')


    # RMSE_thorax_forward_tilt, RMSE_thorax_lateral_tilt, RMSE_thorax_rotation, \
    #     R_thorax_forward_tilt, R_thorax_lateral_tilt, R_thorax_rotation = \
    #     plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
    #                      results_dir, labelA, labelB, joint_of_interest="Thorax")
    #
    # RMSE_elbow_flexion, RMSE_elbow_pronation, RMSE_elbow_pronation_2, \
    #     R_elbow_flexion, R_elbow_pronation, R_elbow_pronation_2 = \
    #     plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
    #                  results_dir, labelA, labelB, joint_of_interest="Elbow")
    #
    # RMSE_HT_Y_plane_of_elevation, RMSE_HT_Z_elevation, RMSE_HT_YY_rotation, \
    #     R_HT_Y_plane_of_elevation, R_HT_Z_elevation, R_HT_YY_rotation = \
    #     plot_compare_JAs_shoulder_eulers(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
    #                                  time, start_time, end_time, results_dir, labelA, labelB)
    #
    # RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
    #     plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
    #                        heading_offset, time, start_time, end_time, results_dir)
    #
    # RMSE_HT_abd, RMSE_HT_flexion, RMSE_HT_rotation, R_HT_abd, R_HT_flexion, R_HT_rotation = \
    #     plot_vector_HT_angles(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
    #                       time, start_time, end_time, results_dir, labelA, labelB)
    #
    # # "Trial Name:": str(compare_name)
    #
    # final_RMSE_values_df = pd.DataFrame.from_dict(
    #     {"thorax_ori": RMSE_thorax_ori, "humerus_ori": RMSE_humerus_ori,
    #      "radius_ori": RMSE_radius_ori, "thorax_forward_tilt": RMSE_thorax_forward_tilt,
    #      "thorax_lateral_tilt": RMSE_thorax_lateral_tilt, "thorax_rotation": RMSE_thorax_rotation,
    #      "elbow_flexion": RMSE_elbow_flexion, "elbow_pronation": RMSE_elbow_pronation,
    #      "HT_abd": RMSE_HT_abd, "HT_flexion": RMSE_HT_flexion,
    #      "HT_rotation": RMSE_HT_rotation, "HT_Y_plane_of_elevation": RMSE_HT_Y_plane_of_elevation,
    #      "HT_Z_elevation": RMSE_HT_Z_elevation, "HT_YY_rotation": RMSE_HT_YY_rotation},
    #     orient='index', columns=["RMSE"])
    #
    # final_R_values_df = pd.DataFrame.from_dict(
    #     {"thorax_forward_tilt": R_thorax_forward_tilt,
    #      "thorax_lateral_tilt": R_thorax_lateral_tilt, "thorax_rotation": R_thorax_rotation,
    #      "elbow_flexion": R_elbow_flexion, "elbow_pronation": R_elbow_pronation,
    #      "HT_abd": R_HT_abd, "HT_flexion": R_HT_flexion,
    #      "HT_rotation": R_HT_rotation, "HT_Y_plane_of_elevation": R_HT_Y_plane_of_elevation,
    #      "HT_Z_elevation": R_HT_Z_elevation, "HT_YY_rotation": R_HT_YY_rotation},
    #     orient='index', columns=["R"])
    #
    # all_data = pd.concat((final_RMSE_values_df, final_R_values_df), axis=1)
    #
    # # Write final RMSE values to a csv
    # print('Writing results to .csv.')
    #
    #
    #
    # all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
    #                             mode='w', encoding='utf-8', na_rep='nan')



if __name__ == '__main__':
    run_IK_compare('P3', 'JA_Slow', 'METHOD_2_Alt_self', 10, 30, True, 'Real')

