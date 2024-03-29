# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import os
from functions import *

def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100
    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

    # Define some file names
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    IMU_IK_results_dir = os.path.join(parent_dir, 'IMU_IK_results_' + calibration_name, trial_name)
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


    """ MAIN """

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
    if OMC_table.getNumRows() == IMU_table.getNumRows() + 1:
        OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)
        thorax_OMC = np.delete(thorax_OMC, [-1], 0)
        humerus_OMC = np.delete(humerus_OMC, [-1], 0)
        radius_OMC = np.delete(radius_OMC, [-1], 0)
        thorax_IMU = np.delete(thorax_IMU, [-1], 0)
        humerus_IMU = np.delete(humerus_IMU, [-1], 0)
        radius_IMU = np.delete(radius_IMU, [-1], 0)
    if OMC_table.getNumRows() != IMU_table.getNumRows():
        print('Tables are different sizes.')

    time = OMC_table.getIndependentColumn()  # Get the time data


    """ PLOT """

    # Plot IMU vs OMC joint angles based on OpenSim coordinates
    print('Plotting results...')
    RMSE_thorax_forward_tilt, RMSE_thorax_lateral_tilt, RMSE_thorax_rotation = \
        plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                         results_dir, labelA, labelB, joint_of_interest="Thorax")

    RMSE_elbow_flexion, RMSE_elbow_pronation, RMSE_elbow_pronation_2 = \
        plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     results_dir, labelA, labelB, joint_of_interest="Elbow")

    RMSE_HT_Y_plane_of_elevation, RMSE_HT_Z_elevation, RMSE_HT_YY_rotation = \
        plot_compare_JAs_shoulder_eulers(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                                     time, start_time, end_time, results_dir, labelA, labelB)

    RMSE_thorax_ori, RMSE_humerus_ori, RMSE_radius_ori = \
        plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                           heading_offset, time, start_time, end_time, results_dir)

    RMSE_HT_abd, RMSE_HT_flexion, RMSE_HT_rotation = plot_vector_HT_angles(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                          time, start_time, end_time, results_dir, labelA, labelB)




    # Write final RMSE values to a csv
    print('Writing results to .csv.')
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


    final_RMSE_values_df.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                                mode='w', encoding='utf-8', na_rep='nan')

