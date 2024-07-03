# Functions used to run IMU_Calibration

from constants import calibration_settings_template_file
from constants import sample_rate
from constants import template_model_file
from helpers_preprocess import APDM_2_sto_Converter
from TwoDoF_Axis_Est.helpers_2DoF import get_J1_J2_from_opt

from os.path import join
from os import makedirs
import opensim as osim
import numpy as np
import pandas as pd
import qmt
from scipy.spatial.transform import Rotation as R

# Calibration settings
sensor_to_opensim_rotations = osim.Vec3(0, 0, 0)
baseIMUName = 'thorax_imu'
baseIMUHeading = '-x'  # Which axis of the thorax IMU points in same direction as the model's thorax x-axis?




""" CUSTOM FUNCTIONS SPECIFIC TO EACH CALIBRATION METHOD"""


# Function to apply ALL_MANUAL method
def get_IMU_offsets_ALL_MANUAL():

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_MANUAL('Thorax')
    humerus_virtual_IMU = get_IMU_cal_MANUAL('Humerus')
    radius_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU


# Function to apply METHOD_1
def get_IMU_offsets_METHOD_1(subject_code, pose_name, IMU_type, calibrated_model_dir):

    trial_name = 'CP'

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file, pose_name)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading, debug=False)

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
def get_IMU_offsets_METHOD_2(subject_code, IMU_type, pose_name, calibrated_model_dir):

    trial_name = 'CP'

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file, pose_name)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading, debug=False)

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
def get_IMU_offsets_METHOD_3(subject_code, IMU_type, calibrated_model_dir):

    pose_name1 = 'Alt_self'
    pose_name2 = 'Alt2_self'
    trial_name1 = 'CP'  # Specify which trial to use for the first calibration pose
    if subject_code in ['P1', 'P2', 'P3']:  # Specify which trial to use for the second calibration pose
        trial_name2 = 'JA_Slow'
    else:
        trial_name2 = 'CP'

    # Get the IMU orientation data at calibration pose time 1
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, trial_name1, pose_name1, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get the IMU orientation data at calibration pose time 2
    cal_oris_file_path_2 = get_cal_ori_file_path(subject_code, trial_name2, pose_name2, IMU_type)
    thorax_IMU_ori2, humerus_IMU_ori2, radius_IMU_ori2 = read_sto_quaternion_file(cal_oris_file_path_2)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file, pose_name1)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading, debug=False)

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


# Function to apply METHOD_4
def get_IMU_offsets_METHOD_4a(subject_code, IMU_type):

    pose_name = 'Alt_self'
    pose_trial_name = 'CP'
    opt_trial_name = 'JA_Slow'
    opt_method = 'rot_noDelta'

    # Get the IMU orientation data at calibration pose time
    cal_oris_file_path_1 = get_cal_ori_file_path(subject_code, pose_trial_name, pose_name, IMU_type)
    thorax_IMU_ori1, humerus_IMU_ori1, radius_IMU_ori1 = read_sto_quaternion_file(cal_oris_file_path_1)

    # Get model body orientations in ground during default pose
    thorax_ori, humerus_ori, radius_ori = get_model_body_oris_during_default_pose(template_model_file, pose_name)

    # Get heading offset between IMU heading and model heading
    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori1, baseIMUHeading, debug=False)

    # Apply the heading offset to the IMU orientations
    heading_offset_ori = R.from_euler('y', heading_offset)  # Create a heading offset scipy rotation
    thorax_IMU_ori_rotated1 = heading_offset_ori * thorax_IMU_ori1
    humerus_IMU_ori_rotated1 = heading_offset_ori * humerus_IMU_ori1
    radius_IMU_ori_rotated1 = heading_offset_ori * radius_IMU_ori1

    """ FINDING FE AND PS FROM OPTIMISATION RESULT """

    # Get the dict with the timings for FE and PS events
    subject_event_dict = get_event_dict_from_file(subject_code)

    # Get the estimated FE and PS axes from the optimisation
    opt_FE_axis_in_humerus_IMU, opt_PS_axis_in_radius_IMU, opt_results = get_J1_J2_from_opt(subject_code, IMU_type, opt_trial_name,
                                                     opt_method, subject_event_dict, sample_rate, debug=False)

    # Get the body-IMU offset for each body, based on the custom methods specified in cal_method_dict
    thorax_virtual_IMU = get_IMU_cal_POSE_BASED(thorax_IMU_ori_rotated1, thorax_ori)
    humerus_virtual_IMU = get_IMU_cal_hum_method_5(opt_FE_axis_in_humerus_IMU, humerus_IMU_ori_rotated1, humerus_ori, debug=False)
    radius_virtual_IMU = get_IMU_cal_rad_method_1(opt_PS_axis_in_radius_IMU, debug=False)

    return thorax_virtual_IMU, humerus_virtual_IMU, radius_virtual_IMU



""" AUXILIARY FUNCTIONS USED IN CALIBRATION PROCESS """


# Get the file path for the sto file containing the IMU orientation data during the specified pose
def get_cal_ori_file_path(subject_code, trial_name, pose_name, IMU_type):
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject
    sto_files_dir = join(join(parent_dir, 'Preprocessed_Data'), trial_name)  # dir for preprocess orientations files
    cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
    cal_oris_file_path = join(sto_files_dir, cal_oris_file)
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
def get_model_body_oris_during_default_pose(model_file, pose_name):

    """ Make sure model is in correct pose """

    # Set the template model pose
    set_default_model_pose(model_file, pose_name)

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

def get_scipyR_of_body_in_ground(body, state):
    Rot = body.getTransformInGround(state).R()
    quat = Rot.convertRotationToQuaternion()
    scipyR = R.from_quat([quat.get(1), quat.get(2), quat.get(3), quat.get(0)])
    return scipyR

# Get the heading offset between the thorax IMU heading and the heading of the model in its default state
def get_heading_offset(base_body_ori, base_IMU_ori, base_IMU_axis_label, debug):

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

    if debug:
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


# Function which takes an array of scipy Rs (orientation) and converts it to a numpy array of quats (scalar first)
def convert_scipy_to_scalar_first_np_quat(scipy):
    arr = np.array([[scipy.as_quat()[3], scipy.as_quat()[0], scipy.as_quat()[1], scipy.as_quat()[2]]])
    return arr


# Function to write dataframes containing quats into an APDM template csv file
def write_to_APDM(df_1, df_2, df_3, df_4, template_file, output_dir, tag):

    # Make columns of zeros
    N = len(df_1)
    zeros_25_df = pd.DataFrame(np.zeros((N, 25)))
    zeros_11_df = pd.DataFrame(np.zeros((N, 11)))
    zeros_2_df = pd.DataFrame(np.zeros((N, 2)))

    # Make a dataframe with zeros columns inbetween the data_out
    IMU_and_zeros_df = pd.concat([zeros_25_df, df_1, zeros_11_df, df_2, zeros_11_df, df_3, zeros_11_df, df_4, zeros_2_df], axis=1)

    # Read in the APDM template and save as an array
    with open(template_file, 'r') as file:
        template_df = pd.read_csv(file, header=0)
        template_array = template_df.to_numpy()

    # Concatenate the IMU_and_zeros and the APDM template headings
    IMU_and_zeros_array = IMU_and_zeros_df.to_numpy()
    new_array = np.concatenate((template_array, IMU_and_zeros_array), axis=0)
    new_df = pd.DataFrame(new_array)

    # Add the new dataframe into the template
    new_df.to_csv(output_dir + "\\" + tag + ".csv", mode='w', index=False, header=False, encoding='utf-8', na_rep='nan')


# Get/make the folder for saving the calibrated model, defined by the calibration name
def get_calibrated_model_dir(subject_code, IMU_type, calibration_name):

    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code  # parent dir for the subject

    IMU_type_dir = join(parent_dir, IMU_type)  # dir for each IMU type
    makedirs(IMU_type_dir, exist_ok=True)

    calibrated_models_dir = join(IMU_type_dir, 'Calibrated_Models')     # dir for calibrated models
    makedirs(calibrated_models_dir, exist_ok=True)

    calibrated_model_dir = join(calibrated_models_dir, calibration_name)   # dir for calibrated model
    makedirs(calibrated_model_dir, exist_ok=True)

    # Create opensim logger file
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(calibrated_model_dir + r'\calibration.log')

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

def set_default_model_pose(model_file, pose):

    if pose in ['Alt_self', 'Alt_asst']:
        EL_X_new = 90  # Specify the default elbow flexion angle in degrees
    elif pose in ['N_self', 'N_asst']:
        EL_X_new = 0    # Specify the default elbow flexion angle in degrees
    else:
        EL_X_new = None
        print('Pose name not specified correctly')
        quit()

    osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files
    model = osim.Model(model_file)
    model.getCoordinateSet().get('EL_x').setDefaultValue(EL_X_new * np.pi / 180)
    model.printToXML(model_file)

    print(f'\nIMU das3.osim default elbow angle has been updated to {EL_X_new} degrees.')

def get_event_dict_from_file(subject_code):

    event_files_folder = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\SubjectEventFiles'
    event_file_name = subject_code + '_event_dict.txt'
    event_file = join(event_files_folder, event_file_name)

    file_obj = open(event_file, 'r')
    event_dict_str = file_obj.read()
    file_obj.close()
    event_dict = eval(event_dict_str)

    return event_dict


""" SENSOR-2-SEGMENT OFFSET CALCULATION FUNCTIONS (applied seperately to each body-IMU pair) """


# This method uses the default pose of the model to calculate an initial orientation offset between body and IMU.
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


# Function to define the humerus IMU offset based on an estimated elbow flexion axis
# Note: this is based on the fixed carry angle defined in the model
def get_IMU_cal_hum_method_5(FE_axis_in_humerus_IMU, humerus_IMU_ori_rotated1, humerus_ori, debug):

    """ GET MODEL EF AXIS IN HUMERUS FRAME """

    # Based on how the hu joint is defined in the model, the XYZ euler ori offset of the parent frame,
    # relative to humerus frame is:
    hu_parent_rel2_hum_R = R.from_euler('XYZ', [0, 0, 0.32318], degrees=False)

    # Based on how the hu joint is defined in the model, relative to the hu joint parent frame,
    # the vector of hu rotation axis (EL_x) is:
    FE_axis_rel2_hu_parent = [0.969, -0.247, 0]

    # Get the vector of hu rotation axis, relative to the humerus frame
    FE_axis_in_humerus = hu_parent_rel2_hum_R.apply(FE_axis_rel2_hu_parent)

    """ GET THE POSE-BASED IMU OFFSET TO CONSTRAIN RESULTS """

    # Get the body-IMU offset for each body, based on the pose-based method (mirroring OpenSims built-in calibration)
    pose_based_virtual_IMU = get_IMU_cal_POSE_BASED(humerus_IMU_ori_rotated1, humerus_ori)

    # Get the individual axes of the pose-based virtual IMU frame
    y_comp_of_pose_based_offset = pose_based_virtual_IMU.as_matrix()[:, 1]
    x_comp_of_pose_based_offset = pose_based_virtual_IMU.as_matrix()[:, 0]
    z_comp_of_pose_based_offset = pose_based_virtual_IMU.as_matrix()[:, 2]

    """ FIND OPTIMAL VIRTUAL IMU OFFSET BASED ON THE INPUTS """

    # We are trying to find a rotational offset between two frames, A - the model's humerus, and B - the humerus IMU
    # The scipy align_vectors() function finds a rotational offset between two frames which optimally aligns two sets of
    # vectors defined in those frames: a, and b.
    # The largest weight is given to the first pair of vectors, because we want to strictly enforce that the estimated
    # elbow flexion axis is aligned with the model elbow flexion axis.
    # The other pairs of vectors are included to constrain the undefined DoF which would be present if we only used the
    # elbow flexion axis vectors. These pairs try to align the humerus IMU frame with the initial estimate of virtual
    # IMU frame from the pose-based calibration

    # Specify the first pairs of vectors which should be aligned, with the highest weighting
    a1 = FE_axis_in_humerus
    b1 = FE_axis_in_humerus_IMU
    w1 = 10000

    # Specify the other pairs of vectors, using the initial guess at the IMU offset based on pose
    a2, a3, a4 = x_comp_of_pose_based_offset , \
        y_comp_of_pose_based_offset, \
        z_comp_of_pose_based_offset    # i.e. the axis of the pose-based virtual IMU frame
    b2, b3, b4 = [1, 0, 0], [0, 1, 0], [0, 0, 1]     # i.e. the x, y, z axis of the IMU frame
    w2, w3, w4 = 1, 1, 1        # These are weighted much lower because we want to prioritise the flexion axis estimation

    # Compile the arrays
    a = [a1, a2, a3, a4]
    b = [b1, b2, b3, b4]
    w = [w1, w2, w3, w4]

    # Run the align_vectors() optimisation
    rot, rssd = R.align_vectors(a, b, weights=w)
    virtual_IMU = rot

    # Alternative function
    # alt_virtual_IMU = qmt.quatFromVectorObservations(b, a, weights=w, debug=False, plot=True)
    # print('Alternative virtual IMU offset:', alt_virtual_IMU)

    if debug:
        print("The estimated FE axis in the humerus IMU frame is:", FE_axis_in_humerus_IMU)
        print("The model's EF axis in the humerus frame is: ", FE_axis_in_humerus)
        print("The initial estimate of virtual IMU offset from pose-based calibration is: \n", pose_based_virtual_IMU.as_matrix())
        print("The optimal virtual IMU offset is: \n", rot.as_matrix())

    return virtual_IMU


# Function to define the humerus IMU offset based on an estimated elbow flexion axis
# Note: this is based on the fixed carry angle defined in the model
def get_IMU_cal_rad_method_1(PS_axis_in_radius_IMU, debug):

    """ GET MODEL PS AXIS IN RADIUS FRAME """

    # PS_axis of the ur joint is defined relative to the parent/child frames, where the child frame = radius body frame
    PS_axis_in_radius = [0.182, 0.98227, -0.044946]

    """ GET THE POSE-BASED IMU OFFSET TO CONSTRAIN RESULTS """

    # Get the IMU offset defined by a manual alignment, to refine results of the optimisation estimation
    manual_virtual_IMU = get_IMU_cal_MANUAL('Radius')

    # Get the individual axes of the manual virtual IMU frame
    x_comp_of_manual_offset = manual_virtual_IMU.as_matrix()[:, 0]
    y_comp_of_manual_offset = manual_virtual_IMU.as_matrix()[:, 1]
    z_comp_of_manual_offset = manual_virtual_IMU.as_matrix()[:, 2]

    """ FIND OPTIMAL VIRTUAL IMU OFFSET BASED ON THE INPUTS """

    # We are trying to find a rotational offset between two frames, A - the model's radius, and B - the radius IMU
    # The scipy align_vectors() function finds a rotational offset between two frames which optimally aligns two sets of
    # vectors defined in those frames: a, and b.
    # The largest weight is given to the first pair of vectors, because we want to strictly enforce that the estimated
    # PS axis is aligned with the model PS axis.
    # The other pairs of vectors are included to constrain the undefined DoF which would be present if we only used the
    # PS axis vectors. These pairs try to align the radius IMU frame with the initial estimate of virtual
    # IMU frame from the manual calibration

    # Specify the first pairs of vectors which should be aligned, with the highest weighting
    a1 = PS_axis_in_radius
    b1 = PS_axis_in_radius_IMU
    w1 = 10000

    # Specify the other pairs of vectors, using the initial guess at the IMU offset based on manual alignment
    a2, a3, a4 = x_comp_of_manual_offset, \
        y_comp_of_manual_offset, \
        z_comp_of_manual_offset  # i.e. the axis of the pose-based virtual IMU frame
    b2, b3, b4 = [1, 0, 0], [0, 1, 0], [0, 0, 1]  # i.e. the x, y, z axis of the IMU frame
    w2, w3, w4 = 1, 1, 1  # These are weighted much lower because we want to prioritise the flexion axis estimation

    # Compile the arrays
    a = [a1, a2, a3, a4]
    b = [b1, b2, b3, b4]
    w = [w1, w2, w3, w4]

    # Run the align_vectors() optimisation
    rot, rssd = R.align_vectors(a, b, weights=w)
    virtual_IMU = rot

    # Alternative function
    # alt_virtual_IMU = qmt.quatFromVectorObservations(b, a, weights=w, debug=False, plot=True)
    # print('Alternative virtual IMU offset:', alt_virtual_IMU)

    if debug:
        print("The estimated PS axis in the forearm IMU frame is:", PS_axis_in_radius_IMU)
        print("The model's PS axis in the radius frame is: ", PS_axis_in_radius)
        print("The initial estimate of virtual IMU offset from manual alignment is: \n", manual_virtual_IMU.as_matrix())
        print("The optimal virtual IMU offset is: \n", rot.as_matrix())

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

    heading_offset = get_heading_offset(thorax_ori, thorax_IMU_ori, base_IMU_axis_label, debug=False)

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
#     sto_files_dir = join(join(parent_dir, 'Preprocessed_Data'), trial_name)  # dir for preprocess orientations files
#     cal_oris_file = IMU_type + '_Quats_' + pose_name + '.sto'
#     cal_oris_file_path = join(sto_files_dir, cal_oris_file)
#     return cal_oris_file_path