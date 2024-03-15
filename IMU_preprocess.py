# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file
# It also does an initial comparison of the raw orientation data from the 'real' and 'perfect' IMUs
# Output is a .csv with RMSE values for each IMU, and a .png plot

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os


""" SETTINGS """

# Quick Settings
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P2"  # Name of the working folder
input_file_Perfect = "P2_CP - Report3 - Cluster_Quats.txt"     # Name of the txt file with perfect IMU data
input_file_Real = "P2_CP - Report2 - IMU_Quats.txt"     # Name of the txt file with real IMU data
cal_pose_time_dict = {"Pose2_assisted": 26}  # List of pose times for calibration
sample_rate = 100
static_time = 1    # Input first known static time to use as reference for change in orientation error

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


# Function to extract quaternion orientation data from .txt file and save as .sto file
def write_movements_and_calibration_stos(file_path, cal_pose_time_dict, IMU_type):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # Write data to APDM format .csv
    file_tag = "APDM_Quats_" + IMU_type
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, file_tag)
    # Write data to .sto using OpenSim APDM converter tool
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
                         output_file_name=results_dir + "\\" + file_tag + ".sto")

    # Iterate through list of calibration poses and associated times to create separate .sto files
    for pose_name in cal_pose_time_dict.keys():

        cal_pose_time = cal_pose_time_dict[pose_name]

        # Extract one row based on time of calibration pose
        IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
        IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
        IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)

        # Write data to APDM format .csv
        file_tag = "APDM_Quats_" + IMU_type + "_" + str(cal_pose_time) + "s"
        write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, results_dir, file_tag)
        # Write data to .sto using OpenSim APDM converter tool
        APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
                             output_file_name=results_dir + "\\" + file_tag + ".sto")

    return IMU1_df, IMU2_df, IMU3_df

# Apply function above to create orientation dataframes and to write data to .sto files
IMU1_df_Perfect, IMU2_df_Perfect, IMU3_df_Perfect = write_movements_and_calibration_stos(file_path_Perfect, cal_pose_time_dict, IMU_type="Perfect")
IMU1_df_Real, IMU2_df_Real, IMU3_df_Real = write_movements_and_calibration_stos(file_path_Real, cal_pose_time_dict, IMU_type="Real")


# Function to calculate orientation accuracy of real IMUs compared with perfect IMUs
def compare_oris_real_vs_perfect(IMU_df_Real, IMU_df_Perfect):

    # Get orientation at specified static time
    static_time_index = static_time * sample_rate
    IMU_Perfect_R_atT = R.from_quat(IMU_df_Perfect.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])
    IMU_Real_R_atT = R.from_quat(IMU_df_Real.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])

    # Create an array to fill with error values
    single_angle_diff = np.zeros((len(IMU_df_Real)))

    for row in range(len(IMU_df_Real)):

        # For all rows which don't contain any nan values in either perfect or real df
        if IMU_df_Real.iloc[row].isna().any() == False and IMU_df_Perfect.iloc[row].isna().any() == False:

            # Get scipy format orientations, ready for comparison
            IMU_Perfect_R = R.from_quat(IMU_df_Perfect.iloc[row].to_numpy()[[1, 2, 3, 0]])
            IMU_Real_R = R.from_quat(IMU_df_Real.iloc[row].to_numpy()[[1, 2, 3, 0]])

            # Get CHANGE in orientations, relative to orientation at specified static time
            ori_change_Perfect_R = IMU_Perfect_R_atT.inv() * IMU_Perfect_R
            ori_change_Real_R = IMU_Real_R_atT.inv() * IMU_Real_R

            # Compare real vs perfect change in orientations
            ori_diff = ori_change_Perfect_R.inv() * ori_change_Real_R
            single_angle_diff_i = ori_diff.magnitude() * 180 / np.pi

        else:
            single_angle_diff_i = np.nan     # If there are nans in either perfect or real dfs, set the angle diff to nan

        single_angle_diff[row] = single_angle_diff_i

    return single_angle_diff


# Apply the function above to compare the real and perfect IMUs
IMU1_single_angle_diff = compare_oris_real_vs_perfect(IMU1_df_Real, IMU1_df_Perfect)
IMU2_single_angle_diff = compare_oris_real_vs_perfect(IMU2_df_Real, IMU2_df_Perfect)
IMU3_single_angle_diff = compare_oris_real_vs_perfect(IMU3_df_Real, IMU3_df_Perfect)

# Calculate RMSE and plot the time-varying error
IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = plot_compare_real_vs_perfect(IMU1_single_angle_diff, IMU2_single_angle_diff, IMU3_single_angle_diff, parent_dir)

# Write final RMSE values to a csv
final_RMSE_values_df = pd.DataFrame.from_dict(
    {"Thorax IMU orientation error:": IMU1_single_angle_RMSE,
     "Humerus IMU orientation error:": IMU2_single_angle_RMSE,
     "Forearm IMU orientation error:": IMU3_single_angle_RMSE,}, orient='index')
final_RMSE_values_df.to_csv(parent_dir + "\\" + "Real_vs_Perfect_Ori_Errors.csv",
                            mode='w', encoding='utf-8', na_rep='nan')
