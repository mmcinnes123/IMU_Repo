from scipy.spatial.transform import Rotation as R
from functions import *
from IMU_IK_functions import APDM_2_sto_Converter


""" Preliminary functions for getting heading offset, rotated IMU orientations, and reading/writing data"""

def read_sto_quaternion_file(IMU_orientations_file, pose_time):

    # Read sto file
    with open(IMU_orientations_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")

    # Filter the data frame based to extract only the values at time = pose_time
    index = df['time'].index[np.isclose(df['time'].loc[:], pose_time, atol=0.001)].to_list()
    df = df.iloc[index[0]]

    # Create scipy orientations from the dataframe
    thorax_IMU_ori_np = np.fromstring(df.loc['thorax_imu'], sep=",")
    thorax_IMU_ori = R.from_quat([thorax_IMU_ori_np[1], thorax_IMU_ori_np[2], thorax_IMU_ori_np[3], thorax_IMU_ori_np[0]])
    humerus_IMU_ori_np = np.fromstring(df.loc['humerus_r_imu'], sep=",")
    humerus_IMU_ori = R.from_quat([humerus_IMU_ori_np[1], humerus_IMU_ori_np[2], humerus_IMU_ori_np[3], humerus_IMU_ori_np[0]])
    radius_IMU_ori_np = np.fromstring(df.loc['radius_r_imu'], sep=",")
    radius_IMU_ori = R.from_quat([radius_IMU_ori_np[1], radius_IMU_ori_np[2], radius_IMU_ori_np[3], radius_IMU_ori_np[0]])

    return thorax_IMU_ori, humerus_IMU_ori, radius_IMU_ori


def get_model_body_oris_during_default_pose(model_file):

    """ Get model body orientations in ground during default pose """

    # Create the model and the bodies
    model = osim.Model(model_file)
    thorax = model.getBodySet().get('thorax')
    humerus = model.getBodySet().get('humerus_r')
    radius = model.getBodySet().get('radius_r')

    # Unlock any locked coordinates to allow model to realise any position
    for i in range(model.getCoordinateSet().getSize()):
        model.getCoordinateSet().get(i).set_locked(False)

    # Create a state based on the model's default state
    default_state = model.initSystem()

    # Get the orientation of each body in the given state
    thorax_ori = get_scipyR_of_body_in_ground(thorax, default_state)
    humerus_ori = get_scipyR_of_body_in_ground(humerus, default_state)
    radius_ori = get_scipyR_of_body_in_ground(radius, default_state)

    return thorax_ori, humerus_ori, radius_ori


def get_heading_offset(base_body_ori, base_IMU_ori, base_IMU_axis_label):

    # Calculate the heading offset
    if base_IMU_axis_label == 'x':
        base_IMU_axis = (base_IMU_ori.as_matrix()[:, 0])  # The x-axis of the IMU in ground frame
    else:
        print("Error: Need to add code if axis is different from 'x'")
        quit()

    base_body_axis = base_body_ori.as_matrix()[:, 0]  # The x-axis of the base body in ground frame

    # Calculate the angle between IMU axis and base segment axis
    heading_offset = np.arccos(np.dot(base_body_axis, base_IMU_axis) /
                               (np.linalg.norm(base_body_axis) * np.linalg.norm(base_IMU_axis)))

    # Update the sign of the heading offset
    if base_IMU_axis[2] < 0:  # Calculate the sign of the rotation (if the z-component of IMU x-axis is negative, rotation is negative)
        heading_offset = -heading_offset
    print("Heading offset is: " + str(round(heading_offset * 180 / np.pi, 2)))
    print("(i.e. IMU heading is rotated " + str(round(-heading_offset * 180 / np.pi, 2))
          + " degrees around the vertical axis, relative to the model's default heading.")

    return heading_offset


def write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated, humerus_IMU_ori_rotated, radius_IMU_ori_rotated, results_dir):

    # # Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
    df1 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(thorax_IMU_ori_rotated))
    df2 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(humerus_IMU_ori_rotated))
    df3 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(radius_IMU_ori_rotated))
    template_file="APDM_template_4S.csv"
    APDM_settings_file = "APDMDataConverter_Settings.xml"
    write_to_APDM(df1, df2, df3, df3, template_file, results_dir, tag="RotatedCalibration")
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_RotatedCalibration.csv",
                         output_file_name=results_dir + r"\APDM_RotatedCalibration.sto")



""" Functions which apply different methods of calibration """


# This method uses the default pose of the model to calculate and initial orientation offset between body and IMU.
# It replicates the built-in OpenSim calibration
def get_IMU_cal_POSE_BASED(IMU_ori, body_ori):

    # Find the body frame, expressed in IMU frame
    body_inIMU = IMU_ori.inv() * body_ori

    # Express the virtual IMU frame in the model's body frame
    virtual_IMU = body_inIMU.inv()

    return virtual_IMU



# This function calculates the IMU offset required which is equivalent to relying on 'manual alignment'
# The only reason we need to apply an offset (and not just have 0 offset) is because the IMU axis names 'xyz' don't
# match the names of the body axes, so are only rotated in multiples of 90degrees
def get_IMU_cal_MANUAL(which_body):

    if which_body == "Thorax":
        virtual_IMU = R.from_euler('XYZ', [180, 0, 0], degrees=True)

    elif which_body == "Humerus":
        virtual_IMU = R.from_euler('XYZ', [180, 90, 0], degrees=True)

    elif which_body == "Radius":
        virtual_IMU = R.from_euler('XYZ', [0, 0, 180], degrees=True)

    else:
        print("Which_body input wasn't 'Thorax', 'Humerus', or 'Radius'")

    return virtual_IMU



# This function calculates an IMU offset defined by the pose of the body,
# but with a correction which defines the long axis of the body to be aligned with the long axis of the IMU.
# It uses the cross product of the new y-axis, and pose-based z axis to define the new x (and then z) axes.
def get_IMU_cal_POSE_and_MANUAL_Y(IMU_ori, body_ori):

    # Get pose-based offset:
    pose_based_body_inIMU = IMU_ori.inv() * body_ori   # Find the body frame, expressed in IMU frame
    pose_based_body_z = pose_based_body_inIMU.as_matrix()[:, 2]

    # Redefine body's long axis as aligned with IMU's long-axis
    body_y = [0, -1, 0]

    # Body x and z axis must be redefined to create an orthogonal CF
    body_x = np.cross(body_y, pose_based_body_z)    # use cross_prod(y, z) to define x-axis
    body_z = np.cross(body_x, body_y)   # then cross_prod(x, y) to define z-axis

    # Create an orthogonal CF from these vectors
    body_matrix = np.array([body_x, body_y, body_z]).T
    body_in_IMU = R.from_matrix(body_matrix)

    # Check how non-orthogonal the matrix was
    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(body_matrix), 4)) + ")")

    # Express the virtual IMU frame in the model's body frame
    virtual_IMU = body_in_IMU.inv()

    return virtual_IMU



# This funtion calculates an IMU offset for the humerus, where the long axis of the humerus is defined by the long axis
# of the IMU, and the z-axis of the humerus is defined by the long axis of the forearm IMU
def get_humerus_IMU_cal_MANUAL_Ys(humerus_IMU_ori, radius_IMU_ori):

    # Define the y axis of the humerus to be aligned with the negative y axis of the humerus IMU
    body_y = [0, -1, 0]

    # Get the orientation of the radius IMU, expressed in the humerus IMU frame
    radius_IMU_inHumIMU = humerus_IMU_ori.inv() * radius_IMU_ori

    # Get the direction of the radius IMUs y-axis
    rad_y_inHumIMU = radius_IMU_inHumIMU.as_matrix()[:, 1]

    # Define the int/ext rotation of the humerus using the radius IMU y-axis
    body_x = np.cross(rad_y_inHumIMU, body_y)   # humerus x-axis is cross_prod of humerus y-axis and radius IMU y-axis
    body_z = np.cross(body_x, body_y)   # remaining z-axis is cross_prod of x and y

    # Create an orthogonal CF from these vectors
    body_matrix = np.array([body_x, body_y, body_z]).T
    body_in_IMU = R.from_matrix(body_matrix)

    # Check how non-orthogonal the matrix was
    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(body_matrix), 4)) + ")")

    # Express the virtual IMU frame in the model's body frame
    virtual_IMU = body_in_IMU.inv()

    return virtual_IMU



""" Functions to calculate and apply IMU offset depending on the method defined """

# This function applies one of the calibration method functions above, depending on the method name specified
def apply_chosen_method(which_body, IMU_ori_rotated, body_ori, second_IMU_ori_rotated, method_name):
    if method_name == "get_IMU_cal_POSE_BASED":
        virtual_IMU = get_IMU_cal_POSE_BASED(IMU_ori_rotated, body_ori)
    elif method_name == "get_IMU_cal_MANUAL":
        virtual_IMU = get_IMU_cal_MANUAL(which_body)
    elif method_name == "get_IMU_cal_POSE_and_MANUAL_Y":
        virtual_IMU = get_IMU_cal_POSE_and_MANUAL_Y(IMU_ori_rotated, body_ori)
    elif method_name == "get_humerus_IMU_cal_MANUAL_Ys":
        virtual_IMU = get_humerus_IMU_cal_MANUAL_Ys(IMU_ori_rotated, second_IMU_ori_rotated)
    else:
        print("Method not defined")

    print(which_body + " virtual IMU eulers: " + str(virtual_IMU.as_euler('XYZ')))

    return virtual_IMU


# A function which calls several other functions to calculate a virtual IMU offset for the thorax, humerus and radius
# Within this function, the type of calibration method for each IMU can be specified
def get_IMU_offset(pose_time, IMU_orientations_file, model_file, results_dir, base_IMU_axis_label):

    """ Get IMU orientations at pose time """

    thorax_IMU_ori, humerus_IMU_ori, radius_IMU_ori = read_sto_quaternion_file(IMU_orientations_file, pose_time)


    """ Get model body orientations in ground during default pose """

    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(model_file)

    """ Get heading offset between IMU heading and model heading """

    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori, base_IMU_axis_label)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated = heading_offset_ori * thorax_IMU_ori
    humerus_IMU_ori_rotated = heading_offset_ori * humerus_IMU_ori
    radius_IMU_ori_rotated = heading_offset_ori * radius_IMU_ori

    # Write the rotated IMU orientations to sto file for visualisation
    write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated, humerus_IMU_ori_rotated, radius_IMU_ori_rotated, results_dir)

    """ Get the IMU offset """

    # Options:
    # Pose-only (OpenSim): get_IMU_cal_POSE_BASED
    # Manual alignment: get_IMU_cal_MANUAL
    # Combined: Pose-based, but then correct with manual Y: get_IMU_cal_POSE_and_MANUAL_Y
    # Manual: Humerus-specific, using humerus IMU y-axis and radius IMU y-axis: get_humerus_IMU_cal_MANUAL_Ys

    # Use this as a setting to decide which method you want to apply
    thorax_cal_method = "get_IMU_cal_POSE_BASED"
    humerus_cal_method = "get_humerus_IMU_cal_MANUAL_Ys"
    radius_cal_method = "get_IMU_cal_MANUAL"

    # Apply the chosen calibration method:
    thorax_virtual_IMU = apply_chosen_method("Thorax", thorax_IMU_ori_rotated, thorax_ori, humerus_IMU_ori_rotated, thorax_cal_method)
    humerus_virtual_IMU = apply_chosen_method("Humerus", humerus_IMU_ori_rotated, humerus_ori, radius_IMU_ori_rotated, humerus_cal_method)
    radius_virtual_IMU = apply_chosen_method("Radius", radius_IMU_ori_rotated, radius_ori, radius_IMU_ori_rotated, radius_cal_method)


    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU


# A function which takes an uncalibrated model (with IMUs already associated with each body)
# and inputs an euler orientation offset which defines the virtual IMU offset relative to the bodies
def apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, results_dir):

    model = osim.Model(model_file)

    def set_IMU_transform(virtual_IMU_R, body_name, imu_name):

        # Define a new OpenSim rotation from the scipy rotation
        mat_from_scipy = virtual_IMU_R.as_matrix()
        rot = osim.Rotation(osim.Mat33(mat_from_scipy[0,0], mat_from_scipy[0,1], mat_from_scipy[0,2],
                                       mat_from_scipy[1,0], mat_from_scipy[1,1], mat_from_scipy[1,2],
                                       mat_from_scipy[2,0], mat_from_scipy[2,1], mat_from_scipy[2,2]))

        # Apply the rotation to the IMU in the model
        IMU_frame = model.getBodySet().get(body_name).getComponent(imu_name)    # Get the exsisting phyiscal offset frame of the IMU
        trans_vec = IMU_frame.getOffsetTransform().T()    # Get the existing translational offset of the IMU frame
        transform = osim.Transform(rot, osim.Vec3(trans_vec))   # Create an opensim transform from the rotation and translation
        IMU_frame.setOffsetTransform(transform)  # Update the IMU frame transform

    set_IMU_transform(thorax_virtual_IMU, body_name='thorax', imu_name='thorax_imu')
    set_IMU_transform(humerus_virtual_IMU, body_name='humerus_r', imu_name='humerus_r_imu')
    set_IMU_transform(radius_virtual_IMU, body_name='radius_r', imu_name='radius_r_imu')

    model.setName("IMU_Calibrated_das")
    model.printToXML(results_dir + r"\Calibrated_das3.osim")