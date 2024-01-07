# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter

""" SETTINGS """

# Quick Settings
input_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20\RawData\20thDec_Movements - Report2 - IMU_Quats.txt"
out_put_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20"
trim_data = False
transform_data = True
start_time = 0
end_time = 24
sample_rate = 100

# Required Files in Folder
template_file = "APDM_template_4S.csv"
APDM_settings_file = "APDMDataConverter_Settings.xml"


""" MAIN """

# Read data in from file
IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(input_file)

# Trim the data based on start and end time
if trim_data == True:
    IMU1_df = trim_df(IMU1_df, start_time, end_time, sample_rate)
    IMU2_df = trim_df(IMU2_df, start_time, end_time, sample_rate)
    IMU3_df = trim_df(IMU3_df, start_time, end_time, sample_rate)

# Interpolate for missing data_out
IMU1_df, IMU1_nan_count = interpolate_df(IMU1_df)
IMU2_df, IMU2_nan_count = interpolate_df(IMU2_df)
IMU3_df, IMU3_nan_count = interpolate_df(IMU3_df)
print("Total missing data_out: " + str(IMU1_nan_count + IMU2_nan_count + IMU3_nan_count))

# Do initial transform of IMU data_out to match OptiTrack Y-up convention, and take transpose
if transform_data == True:
    IMU1_df = intial_IMU_transform_alt(IMU1_df)
    IMU2_df = intial_IMU_transform_alt(IMU2_df)
    IMU3_df = intial_IMU_transform_alt(IMU3_df)

# Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, tag="Movements")
APDM_2_sto_Converter(APDM_settings_file, out_put_dir + r"\APDM_Movements.csv")

print("\nPreview orientations")

# Enter calibration pose time after previewing orientations
cal_pose_time = int(input("\nEnter time of calibration pose (s): "))

# Extract row based on moment of calibration pose
new_cal_pose_time = cal_pose_time - start_time
IMU1_cal_df = extract_cal_row(IMU1_df, new_cal_pose_time, sample_rate)
IMU2_cal_df = extract_cal_row(IMU2_df, new_cal_pose_time, sample_rate)
IMU3_cal_df = extract_cal_row(IMU3_df, new_cal_pose_time, sample_rate)

# Write calibration quaternions to .sto file (write to APDM .csv first, then convert)
write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, tag="Calibration")
APDM_2_sto_Converter(APDM_settings_file, out_put_dir + r"\APDM_Calibration.csv")
