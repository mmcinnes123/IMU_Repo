# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os

""" SETTINGS """

# Quick Settings
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_02_26_Greg"  # Name of the working folder
input_file_Perfect = "26thFeb_Greg - Report3 - Cluster_Quats.txt"     # Name of the txt file with perfect IMU data
input_file_Real = "26thFeb_Greg - Report2 - IMU_Quats.txt"     # Name of the txt file with real IMU data
cal_pose_time_dict = {"Cal_pose_1a": 10, "Cal_pose_2a": 13, "Cal_pose_2b": 15}  # List of pose times for calibration
sample_rate = 100

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

    # Read data in from file
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
    file_tag = "APDM_Quats_" + IMU_type
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, file_tag)
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
                         output_file_name=results_dir + "\\" + file_tag + ".sto")

    # Iterate through list of calibration poses and associated times to create separate .sto files
    for pose_name in cal_pose_time_dict.keys():

        cal_pose_time = cal_pose_time_dict[pose_name]

        # Extract row based on moment of calibration pose
        IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
        IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
        IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)

        # Write calibration quaternions to .sto file (write to APDM .csv first, then convert)
        file_tag = "APDM_Quats_" + IMU_type + "_" + str(cal_pose_time) + "s_"
        write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, results_dir, file_tag)
        APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
                             output_file_name=results_dir + "\\" + file_tag + ".sto")



write_movements_and_calibration_stos(file_path_Perfect, cal_pose_time_dict, IMU_type="Perfect")
write_movements_and_calibration_stos(file_path_Real, cal_pose_time_dict, IMU_type="Real")