# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import os
from functions import *

def fun(compare_name):


    """ SETTINGS """

    # Quick Settings
    trial_name = 'JA_Slow'
    print("Running IK_Compare function for " + compare_name)
    parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P2"  # Name of the working folder
    start_time = 0
    end_time = 100
    calibration_name = 'ALL_POSE_BASED'  # Used to find the calibrated model file
    IMU_IK_results_file_name = 'ALL_POSE_BASED_JA_Slow_IMU_IK_results'  # Used to find the states and .csv file

    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

    # Define some file names
    IMU_IK_results_dir = os.path.join(parent_dir, IMU_IK_results_file_name)
    results_dir = parent_dir + r"\Comparison_" + compare_name
    IMU_states_file = os.path.join(IMU_IK_results_dir, IMU_IK_results_file_name.replace('_IMU_IK_results', '_StatesReporter_states.sto'))
    IMU_csv_file = os.path.join(IMU_IK_results_dir, IMU_IK_results_file_name.replace('_IMU_IK_results', '_IMU_quats.csv'))
    OMC_states_file = os.path.join(parent_dir, 'OMC', trial_name + '_IK_Results', 'OMC_StatesReporter_states.sto')
    OMC_csv_file = os.path.join(parent_dir, 'OMC', trial_name + '_IK_Results', trial_name + '_OMC_quats.csv')

    figure_results_dir = results_dir + "\\TimeRange_" + str(start_time) + "_" + str(end_time) + "s"
    if os.path.exists(results_dir) == False:
        os.mkdir(results_dir)
    if os.path.exists(figure_results_dir) == False:
        os.mkdir(figure_results_dir)
    osim.Logger.addFileSink(results_dir + r"\opensim.log")


    """ MAIN """

    # Read in states for states files
    OMC_table = osim.TimeSeriesTable(OMC_states_file)
    IMU_table = osim.TimeSeriesTable(IMU_states_file)
    print(OMC_table.getNumRows())
    print(IMU_table.getNumRows())

    # Check if they're the same length and remove last row from OMC table if not.
    if OMC_table.getNumRows() != IMU_table.getNumRows():
        OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

    # TODO: Maybe don't need this code anymore...?
    # Find the heading offset between IMU model thorax and OMC model thorax (as a descriptor of global frame offset) (read in untrimmed data)
    thorax_OMC_all, humerus_OMC_all, radius_OMC_all = read_in_quats(start_time, end_time, file_name=OMC_csv_file, trim_bool=False)
    thorax_IMU_all, humerus_IMU_all, radius_IMU_all = read_in_quats(start_time, end_time, file_name=IMU_csv_file, trim_bool=False)
    heading_offset = find_heading_offset(thorax_OMC_all, thorax_IMU_all)

    # Read in body orientations from newly created csv files (as trimmed np arrays (Nx4))
    thorax_OMC, humerus_OMC, radius_OMC = read_in_quats(start_time, end_time, file_name=OMC_csv_file, trim_bool=True)
    thorax_IMU, humerus_IMU, radius_IMU = read_in_quats(start_time, end_time, file_name=IMU_csv_file, trim_bool=True)

    # Account for different in length of results between IMU data and OMC data
    if len(thorax_OMC) == len(thorax_IMU) + 1:
        np.delete(thorax_OMC, [-1], 0)
        np.delete(humerus_OMC, [-1], 0)
        np.delete(radius_OMC, [-1], 0)
    elif len(thorax_OMC) != len(thorax_IMU):
        print('OMC and IMU csvs are different sizes')
        quit()

    # Trim tables based on time of interest
    OMC_table.trim(start_time, end_time)
    IMU_table.trim(start_time, end_time)

    time = OMC_table.getIndependentColumn()  # Get the time data

    """ PLOT """

    # Plot IMU vs OMC joint angles based on OpenSim coordinates
    RMSE_thorax_forward_tilt, RMSE_thorax_lateral_tilt, RMSE_thorax_rotation = \
        plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                         figure_results_dir, labelA, labelB, joint_of_interest="Thorax")

    RMSE_elbow_flexion, RMSE_elbow_pronation, RMSE_elbow_pronation_2 = \
        plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     figure_results_dir, labelA, labelB, joint_of_interest="Elbow")

    RMSE_HT_Y_plane_of_elevation, RMSE_HT_Z_elevation, RMSE_HT_YY_rotation = \
        plot_compare_JAs_shoulder_eulers(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                                     time, start_time, end_time, figure_results_dir, labelA, labelB)

    RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
        plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                           heading_offset, time, start_time, end_time, figure_results_dir)

    RMSE_HT_abd, RMSE_HT_flexion, RMSE_HT_rotation = plot_vector_HT_angles(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                          time, start_time, end_time, figure_results_dir, labelA, labelB)




    # Write final RMSE values to a csv
    final_RMSE_values_df = pd.DataFrame.from_dict(
        {"Trial Name:": str(compare_name),
         "RMSE_thorax_ori": RMSE_thorax_ori, "RMSE_humerus_ori": RMSE_humerus_ori,
         "RMSE_radius_ori": RMSE_radius_ori, "RMSE_thorax_forward_tilt": RMSE_thorax_forward_tilt,
         "RMSE_thorax_lateral_tilt": RMSE_thorax_lateral_tilt, "RMSE_thorax_rotation": RMSE_thorax_rotation,
         "RMSE_elbow_flexion": RMSE_elbow_flexion, "RMSE_elbow_pronation": RMSE_elbow_pronation,
         "RMSE_HT_abd": RMSE_HT_abd, "RMSE_HT_flexion": RMSE_HT_flexion,
         "RMSE_HT_rotation": RMSE_HT_rotation, "RMSE_HT_Y_plane_of_elevation": RMSE_HT_Y_plane_of_elevation,
         "RMSE_HT_Z_elevation": RMSE_HT_Z_elevation, "RMSE_HT_YY_rotation": RMSE_HT_YY_rotation},
        orient='index')


    final_RMSE_values_df.to_csv(figure_results_dir + "\\" + str(compare_name) + r"_Final_RMSEs_" + str(start_time) + "_" + str(end_time) + "s" + ".csv",
                                mode='w', encoding='utf-8', na_rep='nan')


fun(compare_name="ALL_POSE_BASED_JA_Slow")