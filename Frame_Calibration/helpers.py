# Functions to run Frame Calibration

import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd

# A function for reading 3D angular velocity vectors from .txt TMM report file
def read_angvel_data_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    Thorax_Cluster_df = df.filter(["Cluster_Thorax_X", "Cluster_Thorax_Y", "Cluster_Thorax_Z"], axis=1)
    Thorax_IMU_df = df.filter(["IMU1_X", "IMU1_Y", "IMU1_Z"], axis=1)
    Humerus_Cluster_df = df.filter(["Cluster_Humerus_X", "Cluster_Humerus_Y", "Cluster_Humerus_Z"], axis=1)
    Humerus_IMU_df = df.filter(["IMU2_X", "IMU2_Y", "IMU2_Z"], axis=1)
    Forearm_Cluster_df = df.filter(["Cluster_Forearm_X", "Cluster_Forearm_Y", "Cluster_Forearm_Z"], axis=1)
    Forearm_IMU_df = df.filter(["IMU3_X", "IMU3_Y", "IMU3_Z"], axis=1)
    return Thorax_IMU_df, Thorax_Cluster_df, \
        Humerus_IMU_df, Humerus_Cluster_df, \
        Forearm_IMU_df, Forearm_Cluster_df

# A function for calculating the misalignment between two frames, based on vectors measured within those frames,
# which should theoretically be perfectly aligned.
def get_misalign_R_from_angvels(a, b):

    # Use scipy's 'align_vectors' to find the rotation which would best align the two sets of vectors
    rot, rssd, sens = R.align_vectors(a, b, return_sensitivity=True)
    print(f"As Quat: {np.around(rot.as_quat(),decimals=4)}")
    print(f"As Euler (yxz) {np.around(rot.as_euler('yxz', degrees=True), decimals=2)}")
    print(f"RSSD: {rssd}")
    # print(sens)

    return rot


def preprocess_angvels(IMU_ang_vels_df, Cluster_ang_vels_df, start_time, end_time, sample_rate, cut_off, delay):

    ang_vel_arr_real = IMU_ang_vels_df.to_numpy()
    ang_vel_arr_perfect = Cluster_ang_vels_df.to_numpy()

    # Trim the array based on the start and end times specified:
    # A delay is used to shift the IMU data forward to better match the ang vel vectors (to account for IMUs SFA delay)
    ang_vel_arr_real = ang_vel_arr_real[((start_time*sample_rate)+delay):((end_time*sample_rate)+delay)]
    ang_vel_arr_perfect = ang_vel_arr_perfect[(start_time*sample_rate):(end_time*sample_rate)]

    # Filter the arrays based on the magnitude of the angular velocity so we only compare vectors with larger velocity
    indices_to_delete = np.concatenate((np.where(np.linalg.norm(ang_vel_arr_real, axis=1) < cut_off),
                                        np.where(np.linalg.norm(ang_vel_arr_perfect, axis=1) < cut_off)), axis=1)

    ang_vel_arr_real = np.delete(ang_vel_arr_real, indices_to_delete, axis=0)
    ang_vel_arr_perfect = np.delete(ang_vel_arr_perfect, indices_to_delete, axis=0)

    # Normalise the angular velocity vectors so we're only comparing directions (this only affects rssd output)
    for row in range(len(ang_vel_arr_real)):
        ang_vel_arr_real[row] = ang_vel_arr_real[row]/np.linalg.norm(ang_vel_arr_real[row])
        ang_vel_arr_perfect[row] = ang_vel_arr_perfect[row]/np.linalg.norm(ang_vel_arr_perfect[row])

    return ang_vel_arr_real, ang_vel_arr_perfect

