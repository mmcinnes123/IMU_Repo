import numpy as np
from scipy.spatial.transform import Rotation as R
from quat_functions import *
import opensim as osim
from functions import *

""" SETTINGS """
model_file = "das3.osim"
base_IMU_axis_label = 'x'  # The axis of the base IMU which is physically aligned with the x-axis of the model's base segment



# Paste in experimental IMU orientations at time of calibration pose
thorax_IMU_ori = R.from_quat(
    np.array([0.01787000685397091, 0.07901603030628819, -0.03784601451569021, 0.9959943820097346]))
humerus_IMU_ori = R.from_quat(
    np.array([0.09537702302148852, -0.6124921478393913, -0.03382600816470299, 0.7839721892301341]))
radius_IMU_ori = R.from_quat(
    np.array([0.5416990385483884, -0.4266110303584951, -0.5240230372905286, 0.4999650355785131]))
# TODO: Read in .sto file automatically - just read it in as a read_csv file...

osim.Logger.setLevelString("Off")

""" Find model body orientations in ground during default pose """

# Create the model and the bodies
model = osim.Model(model_file)
thorax = model.getBodySet().get('thorax')
humerus = model.getBodySet().get('humerus_r')
radius = model.getBodySet().get('radius_r')

for i in range(model.getCoordinateSet().getSize()):
    model.getCoordinateSet().get(i).set_locked(False)

# Create a state based on the model's default state
default_state = model.initSystem()

# Get the orientation of each body in the given state
thorax_ori = get_scipyR_of_body_in_ground(thorax, default_state)
humerus_ori = get_scipyR_of_body_in_ground(humerus, default_state)
radius_ori = get_scipyR_of_body_in_ground(radius, default_state)

""" Apply heading offset """

# Calculate the heading offset
if base_IMU_axis_label == 'x':
    base_IMU_axis = (thorax_IMU_ori.as_matrix()[:, 0])  # The x-axis of the IMU in ground frame
else:
    print("Need to add code if axis is different from 'x'")
    quit()

base_body_axis = thorax_ori.as_matrix()[:, 0]  # The x-axis of the base body in ground frame

# Calculate the angle between IMU axis and base segment axis
heading_offset = np.arccos(np.dot(base_body_axis, base_IMU_axis) /
                           (np.linalg.norm(base_body_axis) * np.linalg.norm(base_IMU_axis)))

# Check the sign of the heading offset (if the z-component of IMU x-axis is negative, rotation is negative)
if base_IMU_axis[2] < 0:  # Calculate the sign of the rotation
    heading_offset = -heading_offset
print("Heading offset is: " + str(round(heading_offset * 180 / np.pi, 2)))
print("(i.e. IMU heading is rotated " + str(round(-heading_offset * 180 / np.pi, 2))
      + " degrees around the vertical axis, relative to the model's default heading.")

# Apply the heading offset to the IMU orientations
heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
thorax_IMU_ori_rotated = heading_offset_ori * thorax_IMU_ori
humerus_IMU_ori_rotated = heading_offset_ori * humerus_IMU_ori
radius_IMU_ori_rotated = heading_offset_ori * radius_IMU_ori

# # Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
# write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, tag="Movements")
# APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_Movements.csv", output_file_name=results_dir + r"\APDM_Movements.sto")
# TODO: Fix above so you can visualise the rotated quats


""" Define different functions for finding IMU offset """


# This function calculates an IMU offset (to create a virtual IMU in the model) based on
# the orientation of the associated body (when the model is in default pose)
# and on the orientation of the experimental IMU at the time of calibration pose
# This function is a combination of manual alignment and pose-based alignment
def get_IMU_offset_combined(IMU_ori, body_ori):

    # Find the IMU's x-axis, expressed in the body's local coordinate frame
    IMU_x_inGround = IMU_ori.as_matrix()[:, 0]  # Get the IMU's x-axis in the ground frame
    IMU_x_inBody = body_ori.apply(IMU_x_inGround,
                                  inverse=True)  # Use the body's orientation in the ground frame to transform the vector

    # Set the virtual IMUs y-axis to be inline with body's y-axis
    virtual_IMU_y_axis = [0, 1, 0]

    # Calculate the virtual IMUs z-axis based on the cross-product between virtual_IMU_y_axis,
    # and the experimental IMUs x-axis (expressed in local body frame)
    virtual_IMU_z_axis = np.cross(IMU_x_inBody, virtual_IMU_y_axis)

    # Calculate the final virtual IMUs z-axis based on the cross product of the other two
    virtual_IMU_x_axis = np.cross(virtual_IMU_y_axis, virtual_IMU_z_axis)

    # Built the virtual IMU CF from the vectors
    virtual_IMU_mat = np.array([virtual_IMU_x_axis, virtual_IMU_y_axis, virtual_IMU_z_axis]).T

    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(virtual_IMU_mat), 4)) + ")")

    return R.from_matrix(virtual_IMU_mat)



# This function calculates the IMU offset based purely on the intial pose
# - this is replicating OpenSim's built-in calibration tool
def get_IMU_offset_pose_based(IMU_ori, body_ori):

    IMU_offset = body_ori.inv() * IMU_ori

    return IMU_offset



# This function calculates the IMU offset required which is equivalent to relying on 'manual alignment'
# The only reason we need to apply an offset (and not just have 0 offset) is because the IMU axis names 'xyz' don't
# match the names of the body axes, so are only rotated in multiples of 90degrees
def get_IMU_offset_manual(which_body):

    if which_body == "Thorax":
        IMU_offset = R.from_euler('XYZ', [0, 0, 0], degrees=True)

    elif which_body == "Humerus":
        IMU_offset = R.from_euler('Y', [-90], degrees=True)

    elif which_body == "Radius":
        IMU_offset = R.from_euler('Y', [180], degrees=True)

    return IMU_offset



""" Find the IMU offset for each body """

thorax_IMU_offset = get_IMU_offset_combined(thorax_IMU_ori_rotated, thorax_ori)
humerus_IMU_offset = get_IMU_offset_combined(humerus_IMU_ori_rotated, humerus_ori)
radius_IMU_offset = get_IMU_offset_combined(radius_IMU_ori_rotated, radius_ori)

print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))
print(thorax_IMU_offset.as_matrix())
print(humerus_IMU_offset.as_matrix())
print(radius_IMU_offset.as_matrix())

thorax_IMU_offset = get_IMU_offset_pose_based(thorax_IMU_ori_rotated, thorax_ori)
humerus_IMU_offset = get_IMU_offset_pose_based(humerus_IMU_ori_rotated, humerus_ori)
radius_IMU_offset = get_IMU_offset_pose_based(radius_IMU_ori_rotated, radius_ori)

print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))
print(thorax_IMU_offset.as_matrix())
print(humerus_IMU_offset.as_matrix())
print(radius_IMU_offset.as_matrix())

thorax_IMU_offset = get_IMU_offset_manual(which_body="Thorax")
humerus_IMU_offset = get_IMU_offset_manual(which_body="Humerus")
radius_IMU_offset = get_IMU_offset_manual(which_body="Radius")

print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))

# See this link for adding IMUs to model: https://simtk.org/api_docs/opensim/api_docs/exampleIMUTracking_answers_8py-example.html
