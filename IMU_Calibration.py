import numpy as np
from scipy.spatial.transform import Rotation as R
from quat_functions import *
import opensim as osim
from functions import *
from IMU_IK_functions import *

""" SETTINGS """
model_file = "das3.osim"
base_IMU_axis_label = 'x'  # The axis of the base IMU which is physically aligned with the x-axis of the model's base segment
results_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_01_22\Shoulder_cal_compar"


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
osim.Logger.setLevelString("Off")
thorax_ori = get_scipyR_of_body_in_ground(thorax, default_state)
humerus_ori = get_scipyR_of_body_in_ground(humerus, default_state)
radius_ori = get_scipyR_of_body_in_ground(radius, default_state)

""" Apply heading offset """

# Calculate the heading offset
if base_IMU_axis_label == 'x':
    base_IMU_axis = (thorax_IMU_ori.as_matrix()[:, 0])  # The x-axis of the IMU in ground frame
else:
    print("Error: Need to add code if axis is different from 'x'")
    quit()

base_body_axis = thorax_ori.as_matrix()[:, 0]  # The x-axis of the base body in ground frame

# Calculate the angle between IMU axis and base segment axis
heading_offset = np.arccos(np.dot(base_body_axis, base_IMU_axis) /
                           (np.linalg.norm(base_body_axis) * np.linalg.norm(base_IMU_axis)))

# Update the sign of the heading offset
if base_IMU_axis[2] < 0:  # Calculate the sign of the rotation (if the z-component of IMU x-axis is negative, rotation is negative)
    heading_offset = -heading_offset
print("Heading offset is: " + str(round(heading_offset * 180 / np.pi, 2)))
print("(i.e. IMU heading is rotated " + str(round(-heading_offset * 180 / np.pi, 2))
      + " degrees around the vertical axis, relative to the model's default heading.")

# Apply the heading offset to the IMU orientations
heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
thorax_IMU_ori_rotated = heading_offset_ori * thorax_IMU_ori
humerus_IMU_ori_rotated = heading_offset_ori * humerus_IMU_ori
radius_IMU_ori_rotated = heading_offset_ori * radius_IMU_ori

# # # Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
# df1 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(thorax_IMU_ori_rotated))
# df2 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(humerus_IMU_ori_rotated))
# df3 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(radius_IMU_ori_rotated))
# template_file="APDM_template_4S.csv"
# APDM_settings_file = "APDMDataConverter_Settings.xml"
# write_to_APDM(df1, df2, df3, df3, template_file, results_dir, tag="RotatedCalibration")
# APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_RotatedCalibration.csv", output_file_name=results_dir + r"\APDM_RotatedCalibration.sto")



""" Define different functions for finding IMU offset """


""" ARCHIVED FUNCTIONS"""

# The functions below work on the same prinicple as get_IMU_cal_POSE_and_MANUAL_Y, but get there slightly
# differently (define IMU rel to body, not body rel to IMU, or they define different axes first...)

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


# This is an alternative method for get_IMU_offset_combined(), but gets the offset from a single rotation about y
# It gets roughly the same results but with a 1degree difference in eulers
# I think this one is more precise because we don't create an non-orthogonal matrix
def get_IMU_offset_combined_alt(IMU_ori, body_ori):

    # Find the IMU's x-axis, expressed in the body's local coordinate frame
    IMU_x_inGround = IMU_ori.as_matrix()[:, 0]  # Get the IMU's x-axis in the ground frame
    IMU_x_inBody = body_ori.apply(IMU_x_inGround,
                                  inverse=True)  # Use the body's orientation in the ground frame to transform the vector

    # Set the virtual IMUs y-axis to be inline with body's y-axis
    body_z_axis = [0, 0, 1]
    body_y_axis = [0, 1, 0]
    virtual_IMU_y_axis = body_y_axis  # which is equal to body's y-axis

    # Calculate the virtual IMUs z-axis based on the cross-product between virtual_IMU_y_axis,
    # and the experimental IMUs x-axis (expressed in local body frame)
    virtual_IMU_z_axis = np.cross(IMU_x_inBody, virtual_IMU_y_axis)


    angle_between_zs = np.arccos(np.dot(body_z_axis, virtual_IMU_z_axis) /
                           (np.linalg.norm(body_z_axis) * np.linalg.norm(virtual_IMU_z_axis)))

    print(angle_between_zs*180/np.pi)

    offset_from_euls = R.from_euler('XYZ', [0, angle_between_zs, 0])

    print(offset_from_euls.as_euler('XYZ'))


    # return R.from_matrix(virtual_IMU_mat)


# This is an alternative method for get_IMU_offset_combined(), but is worded as if we're defining the
# humerus CF in the IMU's frame, instead of the other way around
def get_IMU_offset_combined_alt2(IMU_ori, body_ori):

    # Find the IMU's x-axis, expressed in the body's local coordinate frame
    IMU_x_inGround = IMU_ori.as_matrix()[:, 0]  # Get the IMU's x-axis in the ground frame
    IMU_y_inGround = IMU_ori.as_matrix()[:, 1]  # Get the IMU's y-axis in the ground frame
    IMU_z_inGround = IMU_ori.as_matrix()[:, 2]  # Get the IMU's z-axis in the ground frame
    IMU_x_inBody = body_ori.apply(IMU_x_inGround,
                                  inverse=True)  # Use the body's orientation in the ground frame to transform the vector

    body_x_inGround = body_ori.as_matrix()[:,0] # Get the body's x-axis in the ground frame
    body_y_inGround = body_ori.as_matrix()[:,1] # Get the body's y-axis in the ground frame
    body_z_inGround = body_ori.as_matrix()[:,2] # Get the body's z-axis in the ground frame
    body_x_inIMU = IMU_ori.apply(body_x_inGround, inverse=True)
    body_y_inIMU = IMU_ori.apply(body_y_inGround, inverse=True)
    body_z_inIMU = IMU_ori.apply(body_z_inGround, inverse=True)

    # Define the axis of the IMU in the IMU's frame
    IMU_x_inIMU = [1, 0, 0]
    IMU_y_inIMU = [0, 1, 0]
    IMU_z_inIMU = [0, 0, 1]

    # Equate the body's y-axis as aligned with the IMU y-axis, in the IMU frame (manual-alignment)
    new_body_y_inIMU = IMU_y_inIMU

    # Define the other two body axes based on the body's pose (pose-based alignment)
    new_body_x_inIMU = np.cross(new_body_y_inIMU, body_z_inIMU)
    new_body_z_inIMU = np.cross(new_body_x_inIMU, new_body_y_inIMU)

    # Build the new body CF from the axes, in the IMU frame
    new_body_mat = np.array([new_body_x_inIMU, new_body_y_inIMU, new_body_z_inIMU]).T

    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(new_body_mat), 4)) + ")")

    # Define the IMU offset from the model body as the inverse of this matrix
    virtual_IMU = R.from_matrix(new_body_mat).inv()

    return virtual_IMU


# This is an alternative method for get_IMU_offset_combined_alt2(), except humerus z is defined as
# cross product of y and x, not humerus x defined as product of y and z.
def get_IMU_offset_combined_alt3(IMU_ori, body_ori):

    # Find the IMU's x-axis, expressed in the body's local coordinate frame
    IMU_x_inGround = IMU_ori.as_matrix()[:, 0]  # Get the IMU's x-axis in the ground frame
    IMU_y_inGround = IMU_ori.as_matrix()[:, 1]  # Get the IMU's y-axis in the ground frame
    IMU_z_inGround = IMU_ori.as_matrix()[:, 2]  # Get the IMU's z-axis in the ground frame
    IMU_x_inBody = body_ori.apply(IMU_x_inGround,
                                  inverse=True)  # Use the body's orientation in the ground frame to transform the vector

    body_x_inGround = body_ori.as_matrix()[:,0] # Get the body's x-axis in the ground frame
    body_y_inGround = body_ori.as_matrix()[:,1] # Get the body's y-axis in the ground frame
    body_z_inGround = body_ori.as_matrix()[:,2] # Get the body's z-axis in the ground frame
    body_x_inIMU = IMU_ori.apply(body_x_inGround, inverse=True)
    body_y_inIMU = IMU_ori.apply(body_y_inGround, inverse=True)
    body_z_inIMU = IMU_ori.apply(body_z_inGround, inverse=True)

    # Define the axis of the IMU in the IMU's frame
    IMU_x_inIMU = [1, 0, 0]
    IMU_y_inIMU = [0, 1, 0]
    IMU_z_inIMU = [0, 0, 1]

    # Equate the body's y-axis as aligned with the IMU y-axis, in the IMU frame (manual-alignment)
    new_body_y_inIMU = IMU_y_inIMU

    # Define the other two body axes based on the body's pose (pose-based alignment)
    new_body_z_inIMU = np.cross(body_x_inIMU, new_body_y_inIMU)
    new_body_x_inIMU = np.cross(new_body_y_inIMU, new_body_z_inIMU)

    # Build the new body CF from the axes, in the IMU frame
    new_body_mat = np.array([new_body_x_inIMU, new_body_y_inIMU, new_body_z_inIMU]).T

    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(new_body_mat), 4)) + ")")

    # Define the IMU offset from the model body as the inverse of this matrix
    virtual_IMU = R.from_matrix(new_body_mat).inv()

    return virtual_IMU



# This function calculates an IMU offset (to create a virtual IMU in the model) based on
# the orientation of the associated body (when the model is in default pose)
# and on the orientation of the experimental IMU at the time of calibration pose
# This function is a combination of manual alignment and pose-based alignment
def get_IMU_offset_combined_new(IMU_ori, body_ori):

    # Find the IMU's x-axis, expressed in the body's local coordinate frame
    IMU_x_inGround = IMU_ori.as_matrix()[:, 0]  # Get the IMU's x-axis in the ground frame
    IMU_y_inGround = IMU_ori.as_matrix()[:, 1]  # Get the IMU's y-axis in the ground frame
    IMU_z_inGround = IMU_ori.as_matrix()[:, 2]  # Get the IMU's z-axis in the ground frame
    IMU_x_inBody = body_ori.apply(IMU_x_inGround, inverse=True)  # Use the body's orientation in the ground frame to transform the vector
    IMU_y_inBody = body_ori.apply(IMU_y_inGround, inverse=True)  # Use the body's orientation in the ground frame to transform the vector
    IMU_z_inBody = body_ori.apply(IMU_z_inGround, inverse=True)  # Use the body's orientation in the ground frame to transform the vector

    # Define the axes of the body, in the body frame
    body_x_axis = [1, 0, 0]
    body_y_axis = [0, 1, 0]
    body_z_axis = [0, 0, 1]

    # Define the virtual IMU's x-axis as perpendicular to the plane formed by body's y-axis and the experimental IMU's z-axis
    virtual_IMU_x_axis = np.cross(body_y_axis, IMU_z_inBody)

    virtual_IMU_z_axis = IMU_z_inBody

    virtual_IMU_y_axis = np.cross(virtual_IMU_z_axis, virtual_IMU_x_axis)


    # Build the virtual IMU CF from the vectors
    virtual_IMU_mat = np.array([virtual_IMU_x_axis, virtual_IMU_y_axis, virtual_IMU_z_axis]).T

    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(virtual_IMU_mat), 4)) + ")")

    return R.from_matrix(virtual_IMU_mat)


""" Find the IMU offset for each body """


thorax_IMU_offset = get_IMU_offset_combined(thorax_IMU_ori_rotated, thorax_ori)
humerus_IMU_offset = get_IMU_offset_combined(humerus_IMU_ori_rotated, humerus_ori)
radius_IMU_offset = get_IMU_offset_combined(radius_IMU_ori_rotated, radius_ori)

print("Combined offsets")
print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())


thorax_IMU_offset = get_IMU_offset_combined_alt2(thorax_IMU_ori_rotated, thorax_ori)
humerus_IMU_offset = get_IMU_offset_combined_alt2(humerus_IMU_ori_rotated, humerus_ori)
radius_IMU_offset = get_IMU_offset_combined_alt2(radius_IMU_ori_rotated, radius_ori)

print("Combined offsets alt 2")
print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())

thorax_IMU_offset = get_IMU_offset_combined_alt3(thorax_IMU_ori_rotated, thorax_ori)
humerus_IMU_offset = get_IMU_offset_combined_alt3(humerus_IMU_ori_rotated, humerus_ori)
radius_IMU_offset = get_IMU_offset_combined_alt3(radius_IMU_ori_rotated, radius_ori)

print("Combined offsets alt 2")
print(thorax_IMU_offset.as_euler('XYZ'))
print(humerus_IMU_offset.as_euler('XYZ'))
print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())




#
# thorax_IMU_offset = get_IMU_offset_pose_based(thorax_IMU_ori_rotated, thorax_ori)
# humerus_IMU_offset = get_IMU_offset_pose_based(humerus_IMU_ori_rotated, humerus_ori)
# radius_IMU_offset = get_IMU_offset_pose_based(radius_IMU_ori_rotated, radius_ori)
#
# print("OpenSim (pose-based) offsets")
# print(thorax_IMU_offset.as_euler('XYZ'))
# print(humerus_IMU_offset.as_euler('XYZ'))
# print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())
#
# thorax_IMU_offset = get_IMU_offset_manual(which_body="Thorax")
# humerus_IMU_offset = get_IMU_offset_manual(which_body="Humerus")
# radius_IMU_offset = get_IMU_offset_manual(which_body="Radius")
#
# print("Manual offsets")
# print(thorax_IMU_offset.as_euler('XYZ'))
# print(humerus_IMU_offset.as_euler('XYZ'))
# print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())


# New combined method
# thorax_IMU_offset = get_IMU_offset_combined_new(thorax_IMU_ori_rotated, thorax_ori)
# humerus_IMU_offset = get_IMU_offset_combined_new(humerus_IMU_ori_rotated, humerus_ori)
# radius_IMU_offset = get_IMU_offset_combined_new(radius_IMU_ori_rotated, radius_ori)

print("Combined offsets")
# print(thorax_IMU_offset.as_euler('XYZ'))
# print(humerus_IMU_offset.as_euler('XYZ'))
# print(radius_IMU_offset.as_euler('XYZ'))
# print(thorax_IMU_offset.as_matrix())
# print(humerus_IMU_offset.as_matrix())
# print(radius_IMU_offset.as_matrix())



# See this link for adding IMUs to model: https://simtk.org/api_docs/opensim/api_docs/exampleIMUTracking_answers_8py-example.html

