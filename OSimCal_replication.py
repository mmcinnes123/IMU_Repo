# This script is a replication of OpenSim's calibration algorithm, which has been checked works against real results

import numpy as np
from scipy.spatial.transform import Rotation as R
from quat_functions import *

# Read in quaternion which describes orientation of base IMU in IMU ground frame
thorax_IMU_scipy = R.from_quat(np.array([0.01787000685397091, 0.07901603030628819, -0.03784601451569021, 0.9959943820097346]))
radius_IMU_scipy = R.from_quat(np.array([-0.5240230372905286, -0.4999650355785131, -0.5416990385483884, -0.4266110303584951]))

# Calculate the heading offset
thorax_IMU_chosen_axis = (thorax_IMU_scipy.as_matrix()[:, 0])  # Calculate the 3d vector of the base IMU's 'chosen' axis - in this case, "x"
thorax_x_axis_in_ground = np.array([1, 0, 0])   # Define the x axis of the model's base segment (improvement on this would be forward axis, not x)
heading_offset = np.arccos(np.dot(thorax_x_axis_in_ground, thorax_IMU_chosen_axis)/
                           (np.linalg.norm(thorax_x_axis_in_ground)*np.linalg.norm(thorax_IMU_chosen_axis)))    # Calculate the angle between IMU axis and base segment axis
if thorax_IMU_chosen_axis[2] < 0:   # Calculate the sign of the rotation
    heading_offset = -heading_offset
print("Heading offset is: " + str(heading_offset*180/np.pi))


# Apply the heading offset to your IMU quaternion
heading_offset_scipy = R.from_euler('y', heading_offset)    # Create a heading offset scipy rotation
rotated_thorax_IMU_scipy = heading_offset_scipy * thorax_IMU_scipy

# Calculate the initial rotational offset between thorax IMU and thorax in default pose
thorax_ori_at_default = R.from_quat(np.array([0, 0, 0, 1])) # Define the orientation of the thorax body during default pose
IMU_offset = thorax_ori_at_default.inv() * rotated_thorax_IMU_scipy

# Take negative euler values to save into model
IMU_offset_saved_in_model = IMU_offset.as_euler('XYZ')
print(IMU_offset.as_matrix())
print(IMU_offset_saved_in_model)
