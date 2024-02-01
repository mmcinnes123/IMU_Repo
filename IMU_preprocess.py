# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os

""" SETTINGS """

# Quick Settings
trial_name = 'IMU_IMU_cal_pose4'    # Tag to describe this trial
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_01_22"  # Name of the working folder
input_file_name = "22ndJan_Movements - Report2 - IMU_Quats.txt"     # Name of the file with quaternion data
transform_data = True  # If using marker cluster questions, make False
cal_pose_time = 14  # Enter time (s) when subject was in calibration pose
trim_data = False
start_time = 0  # Only relevant if trim_data = True
end_time = 24   # Only relevant if trim_data = True
sample_rate = 100

# Required Files in Folder
template_file = "APDM_template_4S.csv"
APDM_settings_file = "APDMDataConverter_Settings.xml"

raw_data_dir = parent_dir + "\RawData"
input_file_path = raw_data_dir + "\\" + input_file_name

# Create a new results directory
results_dir = parent_dir + "\\" + trial_name
if os.path.exists(results_dir) == False:
    os.mkdir(results_dir)
osim.Logger.addFileSink(results_dir + r'\opensim.log')

""" MAIN """

# Check we've set the default pose of the model correctly
pose_confirmation = input("\nIs the default pose of the model set to match the expected subject pose?: ")
if pose_confirmation == "No":
    quit()

# Read data in from file
IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(input_file_path)

# Trim the data based on start and end time
if trim_data == True:
    IMU1_df = trim_df(IMU1_df, start_time, end_time, sample_rate)
    IMU2_df = trim_df(IMU2_df, start_time, end_time, sample_rate)
    IMU3_df = trim_df(IMU3_df, start_time, end_time, sample_rate)

# # Interpolate for missing data_out
# IMU1_df, IMU1_nan_count = interpolate_df(IMU1_df)
# IMU2_df, IMU2_nan_count = interpolate_df(IMU2_df)
# IMU3_df, IMU3_nan_count = interpolate_df(IMU3_df)
# print("Total missing data_out: " + str(IMU1_nan_count + IMU2_nan_count + IMU3_nan_count))

# Do initial transform of IMU data_out to match OptiTrack Y-up convention, and take transpose
if transform_data == True:
    IMU1_df = intial_IMU_transform_alt(IMU1_df)
    IMU2_df = intial_IMU_transform_alt(IMU2_df)
    IMU3_df = intial_IMU_transform_alt(IMU3_df)

# Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, tag="Movements")
APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_Movements.csv", output_file_name=results_dir + r"\APDM_Movements.sto")


# Extract row based on moment of calibration pose
new_cal_pose_time = cal_pose_time - start_time
IMU1_cal_df = extract_cal_row(IMU1_df, new_cal_pose_time, sample_rate)
IMU2_cal_df = extract_cal_row(IMU2_df, new_cal_pose_time, sample_rate)
IMU3_cal_df = extract_cal_row(IMU3_df, new_cal_pose_time, sample_rate)

# Write calibration quaternions to .sto file (write to APDM .csv first, then convert)
write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, results_dir, tag="Calibration")
APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_Calibration.csv", output_file_name=results_dir + r"\APDM_Calibration.sto")
