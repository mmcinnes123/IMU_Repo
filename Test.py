
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd


# Some code to read raw IMU data (perfect or real) and plot the relative heading of the IMUs
# So we can compare relative heading of real vs perfect IMUs

parent_path = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P1\RawData'
IMU_oris_file_name = 'P1_JA_Slow - Report2 - IMU_Quats.txt'
Cluster_oris_file_name = 'P1_JA_Slow - Report3 - Cluster_Quats.txt'
IMU_file_path = os.path.join(parent_path, IMU_oris_file_name)
Cluster_file_path = os.path.join(parent_path, Cluster_oris_file_name)

def angle_between_two_2D_vecs(vec1, vec2):
    angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
    if vec1[1] < 0:
        angle = -angle
    return angle


# Read in data from sto
def read_data_frame_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)

    # Trim the dataframes
    start_time = 50
    end_time = 65
    sample_rate = 100
    start_index = start_time * sample_rate
    end_index = end_time * sample_rate
    IMU1_df = IMU1_df.iloc[start_index:end_index]
    IMU2_df = IMU2_df.iloc[start_index:end_index]
    IMU3_df = IMU3_df.iloc[start_index:end_index]

    # Turn dataframes into Scipy arrays
    IMU1_R = R.from_quat(np.array((IMU1_df.loc[:, ['IMU1_Q1', 'IMU1_Q2', 'IMU1_Q3', 'IMU1_Q0']])))
    IMU2_R = R.from_quat(np.array((IMU2_df.loc[:, ['IMU2_Q1', 'IMU2_Q2', 'IMU2_Q3', 'IMU2_Q0']])))
    IMU3_R = R.from_quat(np.array((IMU3_df.loc[:, ['IMU3_Q1', 'IMU3_Q2', 'IMU3_Q3', 'IMU3_Q0']])))

    # Get the matrix, then the local z axis, on the global XZ plane
    IMU1_z_onXZ = IMU1_R.as_matrix()[:, [0]]
    IMU2_z_onXZ = IMU2_R.as_matrix()[[0,2], 2]
    IMU3_z_onXZ = IMU3_R.as_matrix()[[0,2], 2]

    print(IMU1_R[0].as_matrix())
    print(IMU1_z_onXZ[0])

    # Z_on_XZ = [0, 1]
    #
    # # Get the angle between the local zs and the global Z
    # hum_y_rel2_thor_XZ = angle_between_two_2D_vecs(hum_y_in_thor_XZ, cluster_thorax_Z_on_XZ)
    #
    # return IMU1_R, IMU2_R, IMU3_R

read_data_frame_from_file(IMU_file_path)
# read_data_frame_from_file(Cluster_file_path)



#
#
# # Angular difference between IMU and cluster thorax and radius IMUS at P1, trial=CP, time = 10s (Alt Pose, self)
#
# # Cluster quats:
# cluster_thorax = np.array([0.9001377843481659, -0.1910019542404258, 0.3688189116396666, 0.131311968540742])
# cluster_radius = np.array([0.7509417812503155, 0.1361749603321717, -0.2366139310742536, -0.6012958248422512])
#
# # IMU quats
# imu_thorax = np.array([0.9678897401168001, -0.2317139377836575, 0.07483297990697342, 0.06243498323589707])
# imu_radius = np.array([0.696626743973348,0.2193969193664912,-0.3728468629700412,-0.5723297896553913])
#
#

#
#
# # Get angle between radius y-axis on thorax X-Z plane, relative to Z
# def get_y_on_XZ_rel2_Z_between_body1_and_body2(thorax_quat, radius_quat):
#
#     # Get radius CF expressed in thorax
#     thorax_R = R.from_quat(thorax_quat[[1, 2, 3, 0]])
#     radius_R = R.from_quat(radius_quat[[1, 2, 3, 0]])
#     hum_in_thor_frame = thorax_R.inv() * radius_R
#
#     # Get radius y-axis as 2D vec on thorax XZ plane
#     hum_in_thor_frame_mat = hum_in_thor_frame.as_matrix()
#     hum_y_in_thor_XZ = [hum_in_thor_frame_mat[0, 1], hum_in_thor_frame_mat[2, 1]]
#
#     # Get thorax Z-axis on thorax XZ plane
#     cluster_thorax_Z_on_XZ = [0, 1]
#
#     # Get the angle between the radius y and the thorax Z (on the thorax XZ plane)
#     hum_y_rel2_thor_XZ = angle_between_two_2D_vecs(hum_y_in_thor_XZ, cluster_thorax_Z_on_XZ)
#
#     return hum_y_rel2_thor_XZ
#
# cluster_angle = get_y_on_XZ_rel2_Z_between_body1_and_body2(cluster_thorax, cluster_radius)
# imu_angle = get_y_on_XZ_rel2_Z_between_body1_and_body2(imu_thorax, imu_radius)
#
# print(cluster_angle)
# print(imu_angle)