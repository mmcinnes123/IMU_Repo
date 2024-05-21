# Functions used to run IMU_Calibration

from constants import calibration_settings_template_file
from functions import *
from IMU_IK_functions import APDM_2_sto_Converter

import os

# Calibration settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?
template_model_file = 'das3.osim'




""" CUSTOM FUNCTIONS SPECIFIC TO EACH CALIBRATION METHOD"""


# Function to apply ALL_MANUAL method
def get_IMU_offsets_ALL_MANUAL():

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_MANUAL('Thorax')
    humerus_virtual_IMU = get_IMU_cal_MANUAL('Humerus')
    radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU


# Function to apply METHOD_1
def get_IMU_offsets_METHOD_1(subject_code, trial_name1, pose_name1, IMU_type, calibrated_model_dir):

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name1, pose_name1, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated1 = heading_offset_ori * thorax_IMU_ori1
    humerus_IMU_ori_rotated1 = heading_offset_ori * humerus_IMU_ori1
    radius_IMU_ori_rotated1 = heading_offset_ori * radius_IMU_ori1

    # Write the rotated IMU orientations to sto file for visualisation
    write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated1, humerus_IMU_ori_rotated1, radius_IMU_ori_rotated1, calibrated_model_dir)

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_POSE_BASED(thorax_IMU_ori_rotated1, thorax_ori)
    humerus_virtual_IMU = get_IMU_cal_hum_method_2(humerus_IMU_ori_rotated1, radius_IMU_ori_rotated1)
    radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU


# Function to apply METHOD_2
def get_IMU_offsets_METHOD_2(subject_code, trial_name1, pose_name1, IMU_type, calibrated_model_dir):

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name1, pose_name1, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated1 = heading_offset_ori * thorax_IMU_ori1
    humerus_IMU_ori_rotated1 = heading_offset_ori * humerus_IMU_ori1
    radius_IMU_ori_rotated1 = heading_offset_ori * radius_IMU_ori1

    # Write the rotated IMU orientations to sto file for visualisation
    write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated1, humerus_IMU_ori_rotated1, radius_IMU_ori_rotated1, calibrated_model_dir)

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_POSE_BASED(thorax_IMU_ori_rotated1, thorax_ori)
    humerus_virtual_IMU = get_IMU_cal_hum_method_3(humerus_IMU_ori_rotated1, radius_IMU_ori_rotated1, humerus_ori)
    radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU


# Function to apply METHOD_3
def get_IMU_offsets_METHOD_3(subject_code, trial_name1, trial_name2, pose_name1, pose_name2, IMU_type, calibrated_model_dir):

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name1, pose_name1, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get the IMU orientation data at calibration pose time 2
    cal_oris_file_path_2 = get_cal_ori_file_path(subject_code, trial_name2, pose_name2, IMU_type)
    thorax_IMU_ori2, humerus_IMU_ori2, radius_IMU_ori2 = read_sto_quaternion_file(cal_oris_file_path_2)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated1 = heading_offset_ori * thorax_IMU_ori1
    humerus_IMU_ori_rotated1 = heading_offset_ori * humerus_IMU_ori1
    radius_IMU_ori_rotated1 = heading_offset_ori * radius_IMU_ori1
    thorax_IMU_ori_rotated2 = heading_offset_ori * thorax_IMU_ori2
    humerus_IMU_ori_rotated2 = heading_offset_ori * humerus_IMU_ori2
    radius_IMU_ori_rotated2 = heading_offset_ori * radius_IMU_ori2

    # Write the rotated IMU orientations to sto file for visualisation
    write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated1, humerus_IMU_ori_rotated1, radius_IMU_ori_rotated1, calibrated_model_dir)

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_POSE_BASED(thorax_IMU_ori_rotated1, thorax_ori)
    humerus_virtual_IMU = get_IMU_cal_hum_method_4(humerus_IMU_ori_rotated1, humerus_ori, humerus_IMU_ori_rotated2, radius_IMU_ori_rotated2)
    radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU





""" AUXILIARY FUNCTIONS USED IN CALIBRATION PROCESS """


# Get the file path for the sto file containing the IMU orientation data during the specified pose
def get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type):
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
    sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'), trial_name)  # dir for preprocess orientations files
    cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
    cal_oris_file_path = os.path.join(sto_files_dir, cal_oris_file)
    return cal_oris_file_path


# Read the IMU orientation data from an sto file and return a list of scipy Rs
def read_sto_quaternion_file(IMU_orientations_file):

    # Read sto file
    with open(IMU_orientations_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")

    # # Filter the data frame based to extract only the values at time = pose_time
    # index = df['time'].index[np.isclose(df['time'].loc[:], pose_time, atol=0.001)].to_list()
    # df = df.iloc[index[0]]
    df = df.iloc[0]

    # Create scipy orientations from the dataframe
    thorax_IMU_ori_np = np.fromstring(df.loc['thorax_imu'], sep=",")
    thorax_IMU_ori = R.from_quat([thorax_IMU_ori_np[1], thorax_IMU_ori_np[2], thorax_IMU_ori_np[3], thorax_IMU_ori_np[0]])
    humerus_IMU_ori_np = np.fromstring(df.loc['humerus_r_imu'], sep=",")
    humerus_IMU_ori = R.from_quat([humerus_IMU_ori_np[1], humerus_IMU_ori_np[2], humerus_IMU_ori_np[3], humerus_IMU_ori_np[0]])
    radius_IMU_ori_np = np.fromstring(df.loc['radius_r_imu'], sep=",")
    radius_IMU_ori = R.from_quat([radius_IMU_ori_np[1], radius_IMU_ori_np[2], radius_IMU_ori_np[3], radius_IMU_ori_np[0]])

    return thorax_IMU_ori, humerus_IMU_ori, radius_IMU_ori


# Get the orientation of each model body, relative to the ground frame, during the default pose
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


# Get the heading offset between the thorax IMU heading and the heading of the model in its default state
def get_heading_offset(base_body_ori, base_IMU_ori, base_IMU_axis_label):

    # Calculate the heading offset
    if base_IMU_axis_label == 'x':
        base_IMU_axis = (base_IMU_ori.as_matrix()[:, 0])  # The x-axis of the IMU in ground frame
    elif base_IMU_axis_label == '-x':
        base_IMU_axis = (-base_IMU_ori.as_matrix()[:, 0])  # The x-axis of the IMU in ground frame
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


# Write the rotated IMU data to file so that they can be visualised alongside the model in the default pose
def write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated, humerus_IMU_ori_rotated, radius_IMU_ori_rotated, results_dir):

    # # Write transformed IMU quaternions to .sto file (write to APDM .csv first, then convert)
    df1 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(thorax_IMU_ori_rotated))
    df2 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(humerus_IMU_ori_rotated))
    df3 = pd.DataFrame(convert_scipy_to_scalar_first_np_quat(radius_IMU_ori_rotated))
    template_file="APDM_template_4S.csv"
    APDM_settings_file = "APDMDataConverter_Settings.xml"
    write_to_APDM(df1, df2, df3, df3, template_file, results_dir, tag="APDM_RotatedCalibration")
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + r"\APDM_RotatedCalibration.csv",
                         output_file_name=results_dir + r"\APDM_RotatedCalibration.sto")


# Get/make the folder for saving the calibrated model, defined by the calibration name
def get_calibrated_model_dir(subject_code, IMU_type, calibration_name):

    # Define some file paths
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
    IMU_type_dir = os.path.join(parent_dir, IMU_type)  # working dir for each IMU type
    if os.path.exists(IMU_type_dir) == False:
        os.mkdir(IMU_type_dir)

    # Get/make the directory based on calibration name
    calibrated_model_dir = get_make_model_dir(IMU_type_dir, calibration_name)

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(calibrated_model_dir + r'\calibration.log')

    return calibrated_model_dir


# Get/make the calibrated model folder
def get_make_model_dir(IMU_type_dir, calibration_name):
    calibrated_models_dir = os.path.join(IMU_type_dir, 'Calibrated_Models')
    if os.path.exists(calibrated_models_dir) == False:
        os.mkdir(calibrated_models_dir)
    calibrated_model_dir = os.path.join(calibrated_models_dir, calibration_name)
    if os.path.exists(calibrated_model_dir) == False:
        os.mkdir(calibrated_model_dir)
    return calibrated_model_dir


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
    model.printToXML(results_dir + r"\Calibrated_" + model_file)




""" SENSOR-2-SEGMENT OFFSET CALCULATION FUNCTIONS (applied seperately to each body-IMU pair) """


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
        virtual_IMU = R.from_euler('XYZ', [0, 180, 0], degrees=True)

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
def get_IMU_cal_hum_method_2(humerus_IMU_ori, radius_IMU_ori):

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


# This method starts with a pose-based humerus calibration, then adjusts the calibration in the
# abduction plane using the humerus IMU long axis, then adjust int/ext using forearm IMU long axis.
def get_IMU_cal_hum_method_3(humerus_IMU_ori, radius_IMU_ori, body_ori):

    # Get pose-based offset:
    pose_based_body_inIMU = humerus_IMU_ori.inv() * body_ori   # Find the body frame, expressed in IMU frame
    pose_based_body_z = pose_based_body_inIMU.as_matrix()[:, 2]

    # Calculate the body's x-axis as cross product of IMUs y-axis and body's z-axis
    # i.e. humeral abduction is defined by IMU long axis
    int_body_x = np.cross(pose_based_body_z, [0, 1, 0])

    # Lock in the newly created body y-axis, based on this new x-axis
    body_y = np.cross(pose_based_body_z, int_body_x)
    # This new y is a combination of the abduction defined by the pose, and the flexion defined by the humerus IMU

    # Get the orientation of the radius IMU, expressed in the humerus IMU frame
    radius_IMU_inHumIMU = humerus_IMU_ori.inv() * radius_IMU_ori
    rad_y_inHumIMU = radius_IMU_inHumIMU.as_matrix()[:, 1]  # Get the direction of the radius IMUs y-axis

    # Now, define the int/ext rotation by defining new body x-axis as cross prod between new body y and the forearm y
    body_x = np.cross(rad_y_inHumIMU, body_y)

    # Lastly:
    body_z = np.cross(body_x, body_y)

    # Finish the orthogonal CF from these vectors
    body_matrix = np.array([body_x, body_y, body_z]).T
    body_in_IMU = R.from_matrix(body_matrix)

    # Check how non-orthogonal the matrix was
    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(body_matrix), 4)) + ")")

    # Express the virtual IMU frame in the model's body frame
    virtual_IMU = body_in_IMU.inv()

    return virtual_IMU


# This method builds on get_IMU_cal_hum_method_3(), but uses a different pose (in 90deg forward flexion, with elbow bent)
# to calibrate the int/ext rotation.
# This method starts with a pose-based humerus calibration, then adjusts the calibration in the
# abduction plane using the humerus IMU long axis, then adjust int/ext using forearm IMU long axis, but with body ori
# data from a different pose/moment in time.
def get_IMU_cal_hum_method_4(humerus_IMU_ori_at_t1, humerus_ori_at_t1,
                             humerus_IMU_ori_at_t2, radius_IMU_ori_at_t2):

    # Get pose-based offset, based on time1, when arm is at side:
    pose_based_body_inIMU = humerus_IMU_ori_at_t1.inv() * humerus_ori_at_t1   # Find the body frame, expressed in IMU frame
    pose_based_body_z = pose_based_body_inIMU.as_matrix()[:, 2]

    # Calculate the body's x-axis as cross product of IMUs y-axis and body's z-axis
    # i.e. humeral abduction is defined by IMU long axis
    int_body_x = np.cross(pose_based_body_z, [0, 1, 0])

    # Lock in the newly created body y-axis, based on this new x-axis
    body_y = np.cross(pose_based_body_z, int_body_x)
    # This new y is a combination of the abduction defined by the pose, and the flexion defined by the humerus IMU

    # Get the orientation of the radius IMU, expressed in the humerus IMU frame, at time 2
    radius_IMU_inHumIMU = humerus_IMU_ori_at_t2.inv() * radius_IMU_ori_at_t2
    rad_y_inHumIMU = radius_IMU_inHumIMU.as_matrix()[:, 1]  # Get the direction of the radius IMUs y-axis

    # Now, define the int/ext rotation by defining new body x-axis as cross prod between new body y and the forearm y
    body_x = np.cross(rad_y_inHumIMU, body_y)

    # Lastly:
    body_z = np.cross(body_x, body_y)

    # Finish the orthogonal CF from these vectors
    body_matrix = np.array([body_x, body_y, body_z]).T
    body_in_IMU = R.from_matrix(body_matrix)

    # Check how non-orthogonal the matrix was
    print("Non-orthogonality has been accounted for (determinant was: " + str(
        round(np.linalg.det(body_matrix), 4)) + ")")

    # Express the virtual IMU frame in the model's body frame
    virtual_IMU = body_in_IMU.inv()

    return virtual_IMU



""" BUILT IN OPENSIM CALIBRATION """


# Use OpenSim's built-in calibration method
def osim_calibrate_model(cal_oris_file_path, calibrated_model_dir, model_file):

    # Instantiate an IMUPlacer object
    imuPlacer = osim.IMUPlacer(calibration_settings_template_file)

    # Set properties for the IMUPlacer
    imuPlacer.set_model_file(model_file)
    imuPlacer.set_orientation_file_for_calibration(cal_oris_file_path)
    imuPlacer.set_sensor_to_opensim_rotations(sensor_to_opensim_rotations)
    imuPlacer.set_base_imu_label(baseIMUName)
    imuPlacer.set_base_heading_axis(baseIMUHeading)

    # Update settings file
    imuPlacer.printToXML(calibrated_model_dir + "\\" + calibration_settings_template_file)

    # Run the IMUPlacer
    visualize_calibration = False
    imuPlacer.run(visualize_calibration)

    # Get the model with the calibrated IMU
    model = imuPlacer.getCalibratedModel()

    # Print the calibrated model to file.
    model.printToXML(calibrated_model_dir + r'\Calibrated_' + model_file)






""" ARCHIVED FUNCTIONS """


# OLD SYSTEM OF APPLYING EACH METHOD
def custom_calibrate_model(calibration_name, cal_oris_file_path, cal_oris2_file_path, calibrated_model_dir, model_file):

    # Get the dict which specifies which type of calibration should be applied to each body/IMU pair
    cal_method_dict = get_cal_method_dict(calibration_name)

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU = \
        get_IMU_offset(cal_method_dict, cal_oris_file_path, cal_oris2_file_path, model_file, calibrated_model_dir,
                       baseIMUHeading)

    # Using the IMU offsets calculated above, update the virtual IMUs in the model to create a calibrated model
    apply_cal_to_model(thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU, model_file, calibrated_model_dir)

def get_cal_method_dict(calibration_name):
    """ DEFINING CUSTOM CALIBRATION METHODS """
    # Calibration Method Options:
    # Pose-only (OpenSim): get_IMU_cal_POSE_BASED
    # Manual alignment: get_IMU_cal_MANUAL
    # Combined: Pose-based, but then correct with manual Y: get_IMU_cal_POSE_and_MANUAL_Y
    # Manual: Humerus-specific, using humerus IMU y-axis and radius IMU y-axis: get_IMU_cal_hum_method_2
    # Humerus method 3: pose defines flexion, humerus IMU defines adb, forearm IMU defines int/ext: get_IMU_cal_hum_method_3
    # humerus method 4: based on humerus method 3, but uses alternative pose to get radius IMU projection on humerus IMU

    if calibration_name == 'ALL_MANUAL':
        thorax_method = 'get_IMU_cal_MANUAL'
        humerus_method = 'get_IMU_cal_MANUAL'
        radius_method = 'get_IMU_cal_MANUAL'

    elif calibration_name == 'METHOD_1_Alt_self':
        thorax_method = 'get_IMU_cal_POSE_BASED'
        humerus_method = 'get_IMU_cal_hum_method_2'
        radius_method = 'get_IMU_cal_MANUAL'

    elif calibration_name == 'METHOD_2_Alt_self':
        thorax_method = 'get_IMU_cal_POSE_BASED'
        humerus_method = 'get_IMU_cal_hum_method_3'
        radius_method = 'get_IMU_cal_MANUAL'

    elif calibration_name == 'METHOD_3':
        thorax_method = 'get_IMU_cal_POSE_BASED'
        humerus_method = 'get_IMU_cal_hum_method_4'
        radius_method = 'get_IMU_cal_MANUAL'

    # Set the cal_method_dict for each body based on the method chosen above
    cal_method_dict = {'Thorax': thorax_method,
                       'Humerus': humerus_method,
                       'Radius': radius_method}

    return cal_method_dict

# This function applies one of the calibration method functions above, depending on the method name specified
def apply_chosen_method(which_body, IMU_ori_rotated, body_ori, second_IMU_ori_rotated, method_name, humerus_IMU_ori2_rotated, radius_IMU_ori2_rotated):
    if method_name == "get_IMU_cal_POSE_BASED":
        virtual_IMU = get_IMU_cal_POSE_BASED(IMU_ori_rotated, body_ori)
    elif method_name == "get_IMU_cal_MANUAL":
        virtual_IMU = get_IMU_cal_MANUAL(which_body)
    elif method_name == "get_IMU_cal_POSE_and_MANUAL_Y":
        virtual_IMU = get_IMU_cal_POSE_and_MANUAL_Y(IMU_ori_rotated, body_ori)
    elif method_name == "get_IMU_cal_hum_method_2":
        virtual_IMU = get_IMU_cal_hum_method_2(IMU_ori_rotated, second_IMU_ori_rotated)
    elif method_name == "get_IMU_cal_hum_method_3":
        virtual_IMU = get_IMU_cal_hum_method_3(IMU_ori_rotated, second_IMU_ori_rotated, body_ori)
    elif method_name == "get_IMU_cal_hum_method_4":
        virtual_IMU = get_IMU_cal_hum_method_4(IMU_ori_rotated, body_ori, humerus_IMU_ori2_rotated, radius_IMU_ori2_rotated)
    else:
        print("Method not defined")

    print(which_body + " virtual IMU eulers: " + str(virtual_IMU.as_euler('XYZ')))

    return virtual_IMU

# A function which calls several other functions to calculate a virtual IMU offset for the thorax, humerus and radius
# Within this function, the type of calibration method for each IMU can be specified
def get_IMU_offset(cal_method_dict, calibration_orientations_file, cal_oris2_file, model_file, results_dir, base_IMU_axis_label):

    """ Get IMU orientations at pose time """

    thorax_IMU_ori, humerus_IMU_ori, radius_IMU_ori = read_sto_quaternion_file(calibration_orientations_file)

    thorax_IMU_ori2, humerus_IMU_ori2, radius_IMU_ori2 = read_sto_quaternion_file(cal_oris2_file)


    """ Get model body orientations in ground during default pose """

    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(model_file)

    """ Get heading offset between IMU heading and model heading """

    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori, base_IMU_axis_label)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated = heading_offset_ori * thorax_IMU_ori
    humerus_IMU_ori_rotated = heading_offset_ori * humerus_IMU_ori
    radius_IMU_ori_rotated = heading_offset_ori * radius_IMU_ori
    thorax_IMU_ori2_rotated = heading_offset_ori * thorax_IMU_ori2
    humerus_IMU_ori2_rotated = heading_offset_ori * humerus_IMU_ori2
    radius_IMU_ori2_rotated = heading_offset_ori * radius_IMU_ori2

    # Write the rotated IMU orientations to sto file for visualisation
    write_rotated_IMU_oris_to_file(thorax_IMU_ori_rotated, humerus_IMU_ori_rotated, radius_IMU_ori_rotated, results_dir)

    """ Get the IMU offset """

    # Get the chosen settings to decide which method you want to apply
    thorax_cal_method = cal_method_dict['Thorax']
    humerus_cal_method = cal_method_dict['Humerus']
    radius_cal_method = cal_method_dict['Radius']

    # Apply the chosen calibration method:
    thorax_virtual_IMU = apply_chosen_method("Thorax", thorax_IMU_ori_rotated, thorax_ori, humerus_IMU_ori_rotated, thorax_cal_method, humerus_IMU_ori2, radius_IMU_ori2)
    humerus_virtual_IMU = apply_chosen_method("Humerus", humerus_IMU_ori_rotated, humerus_ori, radius_IMU_ori_rotated, humerus_cal_method, humerus_IMU_ori2_rotated, radius_IMU_ori2_rotated)
    radius_virtual_IMU = apply_chosen_method("Radius", radius_IMU_ori_rotated, radius_ori, radius_IMU_ori_rotated, radius_cal_method, humerus_IMU_ori2, radius_IMU_ori2)


    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU



# def get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type):
#     parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
#     sto_files_dir = os.path.join(os.path.join(parent_dir, 'Preprocessed_Data'), trial_name)  # dir for preprocess orientations files
#     cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
#     cal_oris_file_path = os.path.join(sto_files_dir, cal_oris_file)
#     return cal_oris_file_path