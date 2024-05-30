
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd

# Test code to find JA axes based on angular velocities


# First, can we use OpenSim code to find elbow flexion axis relative to humerus?...
# model_file = 'das3.osim'
# my_model = osim.Model(model_file)
# hu_joint = my_model.getJointSet().get('hu')
# hu_joint_dc = osim.CustomJoint.safeDownCast(hu_joint)
# transform_axis = hu_joint_dc.getSpatialTransform().getTransformAxis(0) # adduction
# # print(transform_axis.dump())
# print(dir(transform_axis))


""" MODEL EL AXIS """

# Manually calculate elbow flexion axis vector in humerus frame

# XYZ orientation offset of hu parent frame, relative to humerus frame:
hu_parent_rel2_hum = [0, 0, 0.32318]
hu_parent_rel2_hum_R = R.from_euler('XYZ', hu_parent_rel2_hum, degrees=False)

# XYZ vector of hu rotation axis (EL_x), relative to parent frame
EL_axis_rel2_hu_parent = [0.969, -0.247, 0]

# XYZ vector of hu rotation axis, relative to humerus
EL_axis_rel2_humerus = hu_parent_rel2_hum_R.apply(EL_axis_rel2_hu_parent)
# print(EL_axis_rel2_humerus)

# Get angle between EL axis and humerus x axis
angle_rel2_humerus_x = np.arctan(EL_axis_rel2_humerus[1]/EL_axis_rel2_humerus[0])
# print(angle_rel2_humerus_x*180/np.pi)


""" HUMERUS IMU EL AXIS """

# From TMM, the EL axis expressed in the humerus IMU frame is
EL_axis_rel2_humerus_IMU = np.array([0.216061, -0.0914449, 0.972088])



""" GETTING THE INITIAL GUESS AT VIRTUAL OFFSET FROM POSE """

pose_based_offset = R.from_euler('XYZ', np.array([-2.5005517615173591, 1.3183366521868178, -0.57104405787610624]))

# # Now express our elbow axis vector in this new, intermidiate frame
# EL_axis_rel2_int_virtual_frame = pose_based_offset.apply(EL_axis_rel2_humerus, inverse=True)

y_comp_of_pose_based_offset = pose_based_offset.as_matrix()[:, 1]
x_comp_of_pose_based_offset = pose_based_offset.as_matrix()[:, 0]
z_comp_of_pose_based_offset = pose_based_offset.as_matrix()[:, 2]

""" FINDING THE ROTATIONAL DIFFERENCE """

# The model's elbow flexion axis, expressed in the humerus body frame
a1 = EL_axis_rel2_humerus
print("a1:", a1)

# The y-component of the pose-based offset virtual IMU frame
a2 = y_comp_of_pose_based_offset
print("a2:", a2)

a3 = x_comp_of_pose_based_offset
print("a3:", a3)

a4 = z_comp_of_pose_based_offset

# The elbow flexion axis in the humerus IMU frame, measured from angular velocity
b1 = EL_axis_rel2_humerus_IMU
print("b1:", b1)

# The unit y vector of the new virtual IMU frame, chosen to be optimally aligned with the y-component of the pose-based result
b2 = [0, 1, 0]
print("b2:", b2)

b3 = [1, 0, 0]
b4 = [0, 0, 1]

a = [a1, a2, a3, a4]
b = [b1, b2, b3, b4]

rot, rssd = R.align_vectors(a, b, weights=[10000, 1, 1, 1])

print(rot.as_matrix())
print(rot.as_euler('ZXY', degrees=True))
print(rot.magnitude()*180/np.pi)

# Calculate the angle between the resultant z-axis and the elbow flexion axis, expressed in the original humerus frame
vec1 = EL_axis_rel2_humerus
vec2 = rot.as_matrix()[:, 2]
angle = np.arccos(np.dot(vec1, vec2)) * 180/np.pi
print(angle)