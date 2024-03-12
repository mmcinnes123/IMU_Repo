# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os

""" SETTINGS """

# Quick Settings
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_03_11"  # Name of the working folder
input_file_Perfect = "Test_3rdMarch - Report3 - Cluster_Quats.txt"     # Name of the txt file with perfect IMU data
input_file_Real = "Test_3rdMarch - Report2 - IMU_Quats.txt"     # Name of the txt file with real IMU data
cal_pose_time_dict = {"Cal_pose_1": 3}  # List of pose times for calibration
sample_rate = 100
static_time = 3    # Input first known static time to use as reference for changing orientation error

# Required Files in Folder
template_file = "APDM_template_4S.csv"
APDM_settings_file = "APDMDataConverter_Settings.xml"

raw_data_dir = parent_dir + "\RawData"
file_path_Perfect = raw_data_dir + "\\" + input_file_Perfect
file_path_Real = raw_data_dir + "\\" + input_file_Real

# Create a new results directory
results_dir = parent_dir + "\Preprocessed_Data"
if os.path.exists(results_dir) == False:
    os.mkdir(results_dir)

osim.Logger.setLevelString("Off")

""" MAIN """

def write_movements_and_calibration_stos(file_path, cal_pose_time_dict, IMU_type):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # # Write data to APDM format .csv
    # file_tag = "APDM_Quats_" + IMU_type
    # write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, file_tag)
    # # Write data to .sto using OpenSim APDM converter tool
    # APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
    #                      output_file_name=results_dir + "\\" + file_tag + ".sto")
    #
    # # Iterate through list of calibration poses and associated times to create separate .sto files
    # for pose_name in cal_pose_time_dict.keys():
    #
    #     cal_pose_time = cal_pose_time_dict[pose_name]
    #
    #     # Extract one row based on time of calibration pose
    #     IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
    #     IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
    #     IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)
    #
    #     # Write data to APDM format .csv
    #     file_tag = "APDM_Quats_" + IMU_type + "_" + str(cal_pose_time) + "s"
    #     write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, results_dir, file_tag)
    #     # Write data to .sto using OpenSim APDM converter tool
    #     APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
    #                          output_file_name=results_dir + "\\" + file_tag + ".sto")

    return IMU1_df, IMU2_df, IMU3_df

IMU1_df_Perfect, IMU2_df_Perfect, IMU3_df_Perfect = write_movements_and_calibration_stos(file_path_Perfect, cal_pose_time_dict, IMU_type="Perfect")
IMU1_df_Real, IMU2_df_Real, IMU3_df_Real = write_movements_and_calibration_stos(file_path_Real, cal_pose_time_dict, IMU_type="Real")



    # TODO: Tidy below

# Calculate orientation accuracy of real IMUs compared with perfect IMUs
    # Comparing CHANGE in orientation, since exact orientations will be different due to different global frames

def compare_oris_real_vs_perfect(IMU_df_Real, IMU_df_Perfect):

    # Remove any rows (from both real and perfect dfs) where nan values appear (due to cluster occlusion)
    IMU_Perfect = IMU_df_Perfect[IMU_df_Perfect.notna().any(axis=1) & IMU_df_Real.notna().any(axis=1)]
    IMU_Real = IMU_df_Real[IMU_df_Perfect.notna().any(axis=1) & IMU_df_Real.notna().any(axis=1)]


    # Get scipy format orientations, ready for comparison
    IMU_Perfect_R = R.from_quat(IMU_Perfect.to_numpy()[:, [1, 2, 3, 0]])
    IMU_Real_R = R.from_quat(IMU_Real.to_numpy()[:, [1, 2, 3, 0]])

    # Get CHANGE in orientations, relative to orientation at time = ? (Needs to be static - add a variable to quick settings)
    static_time_index = static_time*sample_rate
    ori_change_Perfect_R = IMU_Perfect_R[static_time_index].inv() * IMU_Perfect_R
    ori_change_Real_R = IMU_Real_R[static_time_index].inv() * IMU_Real_R

    # Compare real vs perfect change in orientations
    ori_diff = ori_change_Perfect_R.inv() * ori_change_Real_R
    single_angle_diff = ori_diff.magnitude() * 180 / np.pi

    return single_angle_diff


IMU1_single_angle_diff = compare_oris_real_vs_perfect(IMU1_df_Real, IMU1_df_Perfect)
IMU2_single_angle_diff = compare_oris_real_vs_perfect(IMU2_df_Real, IMU2_df_Perfect)
IMU3_single_angle_diff = compare_oris_real_vs_perfect(IMU3_df_Real, IMU3_df_Perfect)



# Calculate RMSE and plot time-varying error
IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = plot_compare_real_vs_perfect(IMU1_single_angle_diff, IMU2_single_angle_diff, IMU3_single_angle_diff, parent_dir)

# Write final RMSE values to a csv
final_RMSE_values_df = pd.DataFrame.from_dict(
    {"Thorax IMU orientation error:": IMU1_single_angle_RMSE,
     "Humerus IMU orientation error:": IMU2_single_angle_RMSE,
     "Forearm IMU orientation error:": IMU3_single_angle_RMSE,}, orient='index')
final_RMSE_values_df.to_csv(parent_dir + "\\" + "Real_vs_Perfect_Ori_Errors.csv",
                            mode='w', encoding='utf-8', na_rep='nan')
