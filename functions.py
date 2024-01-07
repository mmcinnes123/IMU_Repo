### Functions needed to read in the raw data_out, apply transformations, and write to APDM template file.
    # Raw data_out is in  MotionMonitor report file
    # APDM template file can be used to visualise quats in OpenSim
    # Transformations:
        # IMU data_out into Y-Up convention
        # LCF alginment - transform cluster data_out based on relative orientation to IMU at t=0, or average

from scipy.spatial.transform import Rotation as R
# import numpy as np
import pandas as pd
# import matplotlib.pyplot as plt
# import statsmodels.api as sm
from quat_functions import *




# Read all data_out in from specified input file
def read_data_frame_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)
    return IMU1_df, IMU2_df, IMU3_df

# Trim the data_out frames based on start and end time
def trim_df(df, start_time, end_time, sample_rate):
    first_index = int(start_time*sample_rate)
    last_index = int(end_time*sample_rate)
    index_range = list(range(first_index, last_index))
    df_new = df.iloc[index_range, :]
    df_new_new = df_new.reset_index(drop=True)
    return df_new_new

def extract_cal_row(df, cal_time, sample_rate):
    first_index = int(cal_time*sample_rate)
    last_index = first_index + 1
    index_range = list(range(first_index, last_index))
    df_new = df.iloc[index_range, :]
    df_new_new = df_new.reset_index(drop=True)
    return df_new_new


# Interpolate all the data_out and return how many missing data_out points there were.
def interpolate_df(df):
    df = df.interpolate(limit=50)
    nan_count = df.isna().sum().sum()
    return df, nan_count

# Transform IMU data_out into Y-up convention (and apply transpose so orientations are in global frame, not local)
# Used when MM settings are: North: -X, Up: +Z, SubjectFacing: -X
def intial_IMU_transform(IMU_df):
    header = IMU_df.columns
    # Create the rotation matrix to transform the IMU orientations from Delsys global CF to OptiTrack global CF
    rot_matrix = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]]
    # Turn the rotation matrix into a quaternion (note, scipy quats are scalar LAST)
    rot_matrix_asR = R.from_matrix(rot_matrix)
    rot_matrix_asquat = rot_matrix_asR.as_quat()
    rot_quat = [rot_matrix_asquat[3], rot_matrix_asquat[0], rot_matrix_asquat[1], rot_matrix_asquat[2]]
    # For every row in IMU data_out, take the transpose, then multiply by the rotation quaternion
    N = len(IMU_df)
    transformed_quats = np.zeros((N, 4))
    for row in range(N):
        quat_i = np.array([IMU_df.values[row, 0], -IMU_df.values[row, 1], -IMU_df.values[row, 2], -IMU_df.values[row, 3]])
        transformed_quats[row] = quat_mul(rot_quat, quat_i)
    transformed_quats_df = pd.DataFrame(transformed_quats, columns=header)
    return transformed_quats_df

# Transform IMU data_out into Y-up convention (and apply transpose so orientations are in global frame, not local)
# Used when MM settings are: North: +X, Up: +Y, SubjectFacing: -Z
def intial_IMU_transform_alt(IMU_df):
    header = IMU_df.columns
    # Create the rotation matrix to transform the IMU orientations from Delsys global CF to OptiTrack global CF
    rot_matrix = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
    # Turn the rotation matrix into a quaternion (note, scipy quats are scalar LAST)
    rot_matrix_asR = R.from_matrix(rot_matrix)
    rot_matrix_asquat = rot_matrix_asR.as_quat()
    rot_quat = [rot_matrix_asquat[3], rot_matrix_asquat[0], rot_matrix_asquat[1], rot_matrix_asquat[2]]
    # For every row in IMU data_out, take the transpose, then multiply by the rotation quaternion
    N = len(IMU_df)
    transformed_quats = np.zeros((N, 4))
    for row in range(N):
        quat_i = np.array([IMU_df.values[row, 0], -IMU_df.values[row, 1], -IMU_df.values[row, 2], -IMU_df.values[row, 3]])
        transformed_quats[row] = quat_mul(rot_quat, quat_i)
    transformed_quats_df = pd.DataFrame(transformed_quats, columns=header)
    return transformed_quats_df

# Write new data_out to APDM file template
def write_to_APDM(df_1, df_2, df_3, df_4, template_file, tag):
    # Make columns of zeros
    N = len(df_1)
    zeros_25_df = pd.DataFrame(np.zeros((N, 25)))
    zeros_11_df = pd.DataFrame(np.zeros((N, 11)))
    zeros_2_df = pd.DataFrame(np.zeros((N, 2)))

    # Make a dataframe with zeros columns inbetween the data_out
    IMU_and_zeros_df = pd.concat([zeros_25_df, df_1, zeros_11_df, df_2, zeros_11_df, df_3, zeros_11_df, df_4, zeros_2_df], axis=1)

    # Read in the APDM template and save as an array
    with open(template_file, 'r') as file:
        template_df = pd.read_csv(file, header=0)
        template_array = template_df.to_numpy()

    # Concatenate the IMU_and_zeros and the APDM template headings
    IMU_and_zeros_array = IMU_and_zeros_df.to_numpy()
    new_array = np.concatenate((template_array, IMU_and_zeros_array), axis=0)
    new_df = pd.DataFrame(new_array)

    # Add the new dataframe into the template
    new_df.to_csv("APDM_" + tag + ".csv", mode='w', index=False, header=False, encoding='utf-8', na_rep='nan')

# Apply offset to calculate body segment orientations based on IMU_offset
def find_segment_quats(segment_imu_offset, IMU_quats):

    offset_R = R.from_matrix(segment_imu_offset)
    offset_R_quat = offset_R.as_quat()
    offset_quat = np.array([offset_R_quat[3], offset_R_quat[0], offset_R_quat[1], offset_R_quat[2]])

    N = len(IMU_quats)
    segment_quats = np.zeros((N, 4))
    for row in range(N):
        imu_quat_i = np.array([IMU_quats.values[row, 0], IMU_quats.values[row,1], IMU_quats.values[row,2], IMU_quats.values[row,3]])
        segment_quats[row] = quat_mul(imu_quat_i, quat_conj(offset_quat))
    segment_quats_df = pd.DataFrame(segment_quats)

    return segment_quats_df