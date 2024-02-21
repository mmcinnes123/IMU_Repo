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


""" Functions which define different methods of calibration """

# This method uses the default pose of the model to calculate and initial orientation offset between body and IMU.
# It replicates the built-in OpenSim calibration
def get_IMU_cal_POSE_BASED(IMU_ori, body_ori):

    # Find the body frame, expressed in IMU frame
    body_inIMU = IMU_ori.inv() * body_ori

    # Express the virtual IMU frame in the model's body frame:
    virtual_IMU = body_inIMU.inv()

    return virtual_IMU


""" Function to calculate the IMU offset depending on the method defined """

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

    thorax_virtual_IMU = get_IMU_cal_POSE_BASED(thorax_IMU_ori_rotated, thorax_ori)
    print("Thorax virtual IMU eulers: " + str(thorax_virtual_IMU.as_euler('XYZ')))

        # TODO: Add ability to quickly choose which calibration method I want to apply to which IMU
    humerus_virtual_IMU = get_IMU_cal_POSE_BASED(humerus_IMU_ori_rotated, humerus_ori)
    radius_virtual_IMU = get_IMU_cal_POSE_BASED(radius_IMU_ori_rotated, radius_ori)

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