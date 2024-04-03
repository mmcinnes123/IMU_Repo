
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np

# Angular difference between IMU and cluster thorax and radius IMUS at P1, trial=CP, time = 10s (Alt Pose, self)

# Cluster quats:
cluster_thorax = np.array([0.9001377843481659, -0.1910019542404258, 0.3688189116396666, 0.131311968540742])
cluster_radius = np.array([0.7509417812503155, 0.1361749603321717, -0.2366139310742536, -0.6012958248422512])

# IMU quats
imu_thorax = np.array([0.9678897401168001, -0.2317139377836575, 0.07483297990697342, 0.06243498323589707])
imu_radius = np.array([0.696626743973348,0.2193969193664912,-0.3728468629700412,-0.5723297896553913])


def angle_between_two_2D_vecs(vec1, vec2):
    angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
    if vec1[1] < 0:
        angle = -angle
    return angle


# Get angle between radius y-axis on thorax X-Z plane, relative to Z
def get_y_on_XZ_rel2_Z_between_body1_and_body2(thorax_quat, radius_quat):

    # Get radius CF expressed in thorax
    thorax_R = R.from_quat(thorax_quat[[1, 2, 3, 0]])
    radius_R = R.from_quat(radius_quat[[1, 2, 3, 0]])
    hum_in_thor_frame = thorax_R.inv() * radius_R

    # Get radius y-axis as 2D vec on thorax XZ plane
    hum_in_thor_frame_mat = hum_in_thor_frame.as_matrix()
    hum_y_in_thor_XZ = [hum_in_thor_frame_mat[0, 1], hum_in_thor_frame_mat[2, 1]]

    # Get thorax Z-axis on thorax XZ plane
    cluster_thorax_Z_on_XZ = [0, 1]

    # Get the angle between the radius y and the thorax Z (on the thorax XZ plane)
    hum_y_rel2_thor_XZ = angle_between_two_2D_vecs(hum_y_in_thor_XZ, cluster_thorax_Z_on_XZ)

    return hum_y_rel2_thor_XZ

cluster_angle = get_y_on_XZ_rel2_Z_between_body1_and_body2(cluster_thorax, cluster_radius)
imu_angle = get_y_on_XZ_rel2_Z_between_body1_and_body2(imu_thorax, imu_radius)

print(cluster_angle)
print(imu_angle)