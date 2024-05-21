### A mix of funtions for use in 1_Preprocess.py and Compare.py


import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import pandas as pd
from quat_functions import *
import opensim as osim
from scipy.spatial.transform import Slerp
from scipy.signal import find_peaks
from scipy.stats import pearsonr
from scipy import signal
from IMU_IK_functions import APDM_2_sto_Converter


""" FUNCTIONS FOR READING IN DATA"""


# Read all data_out in from specified input file
def read_data_frame_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)
    return IMU1_df, IMU2_df, IMU3_df


# Trim the data_out frames based on start and end time
def trim_df(df, start_time, end_time, sample_rate):
    first_index = int(start_time*sample_rate)
    last_index = int(end_time*sample_rate)
    index_range = list(range(first_index, last_index))
    df_new = df.iloc[index_range, :]
    df_new_new = df_new.reset_index(drop=True)
    return df_new_new


def extract_cal_row(df, cal_time, sample_rate):
    first_index = int(cal_time*sample_rate)
    last_index = first_index + 1
    index_range = list(range(first_index, last_index))
    df_new = df.iloc[index_range, :]
    df_new_new = df_new.reset_index(drop=True)
    return df_new_new


# Interpolate all the data_out and return how many missing data_out points there were.
def interpolate_df(df):
    df = df.interpolate(limit=50)
    nan_count = df.isna().sum().sum()
    return df, nan_count


# Transform IMU data_out into Y-up convention (and apply transpose so orientations are in global frame, not local)
# Used when MM settings are: North: -X, Up: +Z, SubjectFacing: -X
def intial_IMU_transform(IMU_df):
    header = IMU_df.columns
    # Create the rotation matrix to transform the IMU orientations from Delsys global CF to OptiTrack global CF
    rot_matrix = [[-1, 0, 0], [0, 0, 1], [0, 1, 0]]
    # Turn the rotation matrix into a quaternion (note, scipy quats are scalar LAST)
    rot_matrix_asR = R.from_matrix(rot_matrix)
    rot_matrix_asquat = rot_matrix_asR.as_quat()
    rot_quat = [rot_matrix_asquat[3], rot_matrix_asquat[0], rot_matrix_asquat[1], rot_matrix_asquat[2]]
    # For every row in IMU data_out, take the transpose, then multiply by the rotation quaternion
    N = len(IMU_df)
    transformed_quats = np.zeros((N, 4))
    for row in range(N):
        quat_i = np.array([IMU_df.values[row, 0], -IMU_df.values[row, 1], -IMU_df.values[row, 2], -IMU_df.values[row, 3]])
        transformed_quats[row] = quat_mul(rot_quat, quat_i)
    transformed_quats_df = pd.DataFrame(transformed_quats, columns=header)
    return transformed_quats_df


# Transform IMU data_out into Y-up convention (and apply transpose so orientations are in global frame, not local)
# Used when MM settings are: North: +X, Up: +Y, SubjectFacing: -Z
def intial_IMU_transform_alt(IMU_df):
    header = IMU_df.columns
    # Create the rotation matrix to transform the IMU orientations from Delsys global CF to OptiTrack global CF
    global_rot_scipy = R.from_matrix(np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]))
    global_rot_quat = [global_rot_scipy.as_quat()[3], global_rot_scipy.as_quat()[0], global_rot_scipy.as_quat()[1], global_rot_scipy.as_quat()[2]] # Switch from scalal last format
    # Create a rotation matrix which switches the local axes to match the Delsys namign convention
    local_rot_scipy = R.from_euler('YXZ', [180, 90, 0], degrees=True)
    local_rot_quat = [local_rot_scipy.as_quat()[3], local_rot_scipy.as_quat()[0], local_rot_scipy.as_quat()[1], local_rot_scipy.as_quat()[2]]
    # For every row in IMU data_out, take the transpose, then multiply by the global and local rotation quaternions
    N = len(IMU_df)
    transformed_quats = np.zeros((N, 4))
    for row in range(N):
        quat_i = np.array([IMU_df.values[row, 0], -IMU_df.values[row, 1], -IMU_df.values[row, 2], -IMU_df.values[row, 3]])
        transformed_quats_int = quat_mul(global_rot_quat, quat_i)   # Pre-multiply by the global rotation
        transformed_quats[row] = quat_mul(transformed_quats_int, local_rot_quat)    # Post-multiply by the local rotation
    transformed_quats_df = pd.DataFrame(transformed_quats, columns=header)
    return transformed_quats_df


# Write new data_out to APDM file template
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



""" FUNCTIONS FOR ANALYSIS"""

# Apply offset to calculate body segment orientations based on IMU_offset
def find_segment_quats(segment_imu_offset, IMU_quats):

    offset_R = R.from_matrix(segment_imu_offset)
    offset_R_quat = offset_R.as_quat()
    offset_quat = np.array([offset_R_quat[3], offset_R_quat[0], offset_R_quat[1], offset_R_quat[2]])

    N = len(IMU_quats)
    segment_quats = np.zeros((N, 4))
    for row in range(N):
        imu_quat_i = np.array([IMU_quats.values[row, 0], IMU_quats.values[row,1], IMU_quats.values[row,2], IMU_quats.values[row,3]])
        segment_quats[row] = quat_mul(imu_quat_i, quat_conj(offset_quat))
    segment_quats_df = pd.DataFrame(segment_quats)

    return segment_quats_df



def get_body_quat(state, body):
    Rot = body.getTransformInGround(state).R()
    quat = Rot.convertRotationToQuaternion()
    output_quat = np.array([quat.get(0), quat.get(1), quat.get(2), quat.get(3)])
    return output_quat


def get_vec_between_bodies(state, body1, body2):

    Rot = body2.findTransformBetween(state, body1).R()  # Finds rotation between two bodies
    quat = Rot.convertRotationToQuaternion()
    scipyR = R.from_quat([quat.get(1), quat.get(2), quat.get(3), quat.get(0)])
    mat = scipyR.as_matrix()
    mat_x_X = mat[0,0]  # This is the component of the humerus x-axis which points in the thorax X direction
    mat_x_Z = mat[2,0]  # This is the component of the humerus x-axis which points in the thorax Z direction
    mat_z_X = mat[0,2]  # This is the component of the humerus z-axis which points in the thorax X direction
    mat_z_Z = mat[2,2]  # This is the component of the humerus z-axis which points in the thorax Z direction
    mat_y_X = mat[0,1]  # This is the component of the humerus y-axis which points in the thorax X direction
    mat_y_Z = mat[2,1]  # This is the component of the humerus y-axis which points in the thorax Z direction
    # Get angle of local x-axis projected on the thorax XZ plane (relative to thorax X)
    angle_x = np.arctan(mat_x_Z/mat_x_X)*180/np.pi
    # Get angle of local z-axis projected on the thorax XZ plane (relative to thorax Z)
    angle_z = -np.arctan(mat_z_X/mat_z_Z)*180/np.pi
    # Get angle of local y-axis projected on the thorax XZ plane (relative to thorax X)
    angle_y = -np.arctan(mat_y_Z/mat_y_X)*180/np.pi


    return angle_x, angle_z, angle_y


def get_JA_euls_from_quats(body1_quats, body2_quats, eul_seq):

    n_rows = len(body1_quats)
    eul_1_arr = np.zeros((n_rows))
    eul_2_arr = np.zeros((n_rows))
    eul_3_arr = np.zeros((n_rows))
    for row in range(n_rows):
        joint_Rot = quat_mul(quat_conj(body1_quats[row]), body2_quats[row])  # Calculate joint Rot quat
        joint_scipyR = R.from_quat([joint_Rot[1], joint_Rot[2], joint_Rot[3], joint_Rot[0]])    # In scalar last format
        joint_eul = joint_scipyR.as_euler(eul_seq, degrees=True)    # Get euler angles
        eul_1_arr[row], eul_2_arr[row], eul_3_arr[row] = joint_eul[0], joint_eul[1], joint_eul[2]

    return eul_1_arr, eul_2_arr, eul_3_arr


def get_eulers_between_two_bodies(state, body1, body2, eul_seq):
    Rot = body2.findTransformBetween(state, body1).R()  # Finds rotation between two bodies
    quat = Rot.convertRotationToQuaternion()
    scipyR = R.from_quat([quat.get(1), quat.get(2), quat.get(3), quat.get(0)])
    # Get euler angles
    eul = scipyR.as_euler(eul_seq, degrees=True)

    return eul[0], eul[1], eul[2]


def get_scipyR_of_body_in_ground(body, state):
    Rot = body.getTransformInGround(state).R()
    quat = Rot.convertRotationToQuaternion()
    scipyR = R.from_quat([quat.get(1), quat.get(2), quat.get(3), quat.get(0)])
    return scipyR

def get_joint_angles_from_states(states_file, model_file, start_time, end_time):

    # Create a time series table from the states file
    state_table = osim.TimeSeriesTable(states_file)
    # Trim the table based on time period of interest
    state_table.trim(start_time, end_time)

    # Create the model and the bodies
    model = osim.Model(model_file)
    thorax = model.getBodySet().get('thorax')
    humerus_r = model.getBodySet().get('humerus_r')

    # Unlock any locked coordinates in model
    for coord in ['TH_x','TH_y','TH_z','TH_x_trans','TH_y_trans','TH_z_trans',
                  'SC_x','SC_y','SC_z','AC_x','AC_y','AC_z','GH_y','GH_z','GH_yy','EL_x','PS_y']:
        model.getCoordinateSet().get(coord).set_locked(False)

    # Get the states info from the states file
    stateTrajectory = osim.StatesTrajectory.createFromStatesTable(model, state_table)
    n_rows = stateTrajectory.getSize()

    # Initiate the system so that the model can actively realise positions based on states
    model.initSystem()

    # Get the relative orientation of the two bodies from the states info
    HT1_arr = np.zeros((n_rows))
    HT2_arr = np.zeros((n_rows))
    HT3_arr = np.zeros((n_rows))
    HT_IER_arr_x = np.zeros((n_rows))
    HT_IER_arr_z = np.zeros((n_rows))
    HT_IER_arr_y = np.zeros((n_rows))
    for row in range(n_rows):
        state = stateTrajectory.get(row)
        model.realizePosition(state)
        HT1_arr[row], HT2_arr[row], HT3_arr[row] = get_eulers_between_two_bodies(state, thorax, humerus_r, 'YZY')
        # Get vector projection based int/ext rot
        HT_IER_arr_x[row], HT_IER_arr_z[row], HT_IER_arr_y[row] = get_vec_between_bodies(state, thorax, humerus_r)

    return HT1_arr, HT2_arr, HT3_arr, HT_IER_arr_x, HT1_arr, HT2_arr, HT3_arr, HT_IER_arr_x, HT_IER_arr_z, HT_IER_arr_y


# Define a function for extracting body orientations from the states table
def extract_body_quats(states_table, model_file, results_dir, tag):

    # Create the model and the bodies
    model = osim.Model(model_file)
    thorax = model.getBodySet().get('thorax')
    humerus = model.getBodySet().get('humerus_r')
    radius = model.getBodySet().get('radius_r')

    # Unlock any locked coordinates in model
    for coord in ['TH_x','TH_y','TH_z','TH_x_trans','TH_y_trans','TH_z_trans',
                  'SC_x','SC_y','SC_z','AC_x','AC_y','AC_z','GH_y','GH_z','GH_yy','EL_x','PS_y']:
        model.getCoordinateSet().get(coord).set_locked(False)

    print("Getting states info from states file...")

    # Get the states info from the states file (this is the step which is computationally slow)
    stateTrajectory = osim.StatesTrajectory.createFromStatesTable(model, states_table)
    n_rows = stateTrajectory.getSize()
    print('Made stateTrajectory from states file.')

    # Initiate the system so that the model can actively realise positions based on states
    model.initSystem()

    # Get the orientation of each body of interest
    thorax_quats = np.zeros((n_rows, 4))
    humerus_quats = np.zeros((n_rows, 4))
    radius_quats = np.zeros((n_rows, 4))
    for row in range(n_rows):
        state = stateTrajectory.get(row)
        model.realizePosition(state)
        thorax_quats[row] = get_body_quat(state, thorax)
        humerus_quats[row] = get_body_quat(state, humerus)
        radius_quats[row] = get_body_quat(state, radius)

    # Write all body quats to a csv file
    thorax_quats_df = pd.DataFrame({"Thorax_Q0": thorax_quats[:,0],"Thorax_Q1": thorax_quats[:,1], "Thorax_Q2": thorax_quats[:,2], "Thorax_Q3": thorax_quats[:,3]})
    humerus_quats_df = pd.DataFrame({"Humerus_Q0": humerus_quats[:,0],"Humerus_Q1": humerus_quats[:,1], "Humerus_Q2": humerus_quats[:,2],"Humerus_Q3": humerus_quats[:,3]})
    radius_quats_df = pd.DataFrame({"Radius_Q0": radius_quats[:,0],"Radius_Q1": radius_quats[:,1], "Radius_Q2": radius_quats[:,2],"Radius_Q3": radius_quats[:,3]})
    time_df = pd.DataFrame({"Time": np.asarray(states_table.getIndependentColumn())[:]})

    all_quats_df = pd.concat([time_df, thorax_quats_df, humerus_quats_df, radius_quats_df], axis=1)

    print("Writing " + tag + " orientations file to csv...")

    all_quats_df.to_csv(results_dir + "\\" + tag + "_quats.csv", mode='w', encoding='utf-8', na_rep='nan')


def read_in_quats(start_time, end_time, file_name, trim_bool):
    with open(file_name, 'r') as file:
        df = pd.read_csv(file, header=0)
    # Trim dataframe
    if trim_bool == True:
        df = df.loc[(df['Time'] >= start_time) & (df['Time'] <= end_time)]
    # Extract separate dfs for each body
    thorax_quats = df.filter(["Thorax_Q0", "Thorax_Q1", "Thorax_Q2", "Thorax_Q3"], axis=1)
    humerus_quats = df.filter(["Humerus_Q0", "Humerus_Q1", "Humerus_Q2", "Humerus_Q3"], axis=1)
    radius_quats = df.filter(["Radius_Q0", "Radius_Q1", "Radius_Q2", "Radius_Q3"], axis=1)
    thorax_quats_np = thorax_quats.to_numpy()
    humerus_quats_np = humerus_quats.to_numpy()
    radius_quats_np = radius_quats.to_numpy()

    return thorax_quats_np, humerus_quats_np, radius_quats_np


# A function for calculating the average heading offset between an array of two bodies or IMUs, relative to a global frame
def find_heading_offset(OMC_thorax_quats, IMU_thorax_quats):
    def find_heading_of_thorax(thorax_quats, row):
        thorax_scipy_R = R.from_quat([thorax_quats[row,1], thorax_quats[row,2], thorax_quats[row,3], thorax_quats[row,0]])
        thorax_rot_mat = thorax_scipy_R.as_matrix()
        # Calculating angle of thorax z-axis on the glboal XZ plane
        mat_z_Z = thorax_rot_mat[2,2]
        mat_z_X = thorax_rot_mat[0,2]
        angle_z = np.arctan(mat_z_X / mat_z_Z)
        return angle_z

    heading_offset_arr = np.zeros((len(OMC_thorax_quats)))
    for row in range(len(OMC_thorax_quats)):
        OMC_thorax_offset_i = find_heading_of_thorax(OMC_thorax_quats, row)
        IMU_thorax_offset_i = find_heading_of_thorax(IMU_thorax_quats, row)
        heading_offset_arr[row] = OMC_thorax_offset_i - IMU_thorax_offset_i

    angle_z = np.mean(heading_offset_arr)

    return angle_z


# Define functions for finding vector projected joint angles for the HT joint
def get_vec_angles_from_two_CFs(CF1, CF2):

    def angle_between_two_2D_vecs(vec1, vec2):
        angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
        if vec1[1] < 0:
            angle = -angle
        return angle

    n_rows = len(CF1)
    x_rel2_X_on_XY = np.zeros((n_rows))
    x_rel2_X_on_XZ = np.zeros((n_rows))
    z_rel2_Z_on_ZY = np.zeros((n_rows))
    y_rel2_Y_on_XY = np.zeros((n_rows))

    for row in range(n_rows):
        joint_rot = quat_mul(quat_conj(CF1[row]), CF2[row])  # Calculate joint rotation quaternion
        joint_scipyR = R.from_quat([joint_rot[1], joint_rot[2], joint_rot[3], joint_rot[0]])  # In scalar last format
        joint_mat = joint_scipyR.as_matrix()    # Calculate the joint rotation matrix
        # Extract the vector components of CF2 axes relative to CF1 axes
        mat_x_X = joint_mat[0,0]    # This is the component of the CF2 x axis in the CF1 X direction
        mat_x_Y = joint_mat[1,0]    # This is the component of the CF2 x axis in the CF1 Y direction
        mat_x_Z = joint_mat[2,0]    # This is the component of the CF2 x axis in the CF1 Z direction
        mat_y_X = joint_mat[0,1]    # This is the component of the CF2 y axis in the CF1 X direction
        mat_y_Y = joint_mat[1,1]    # This is the component of the CF2 y axis in the CF1 Y direction
        mat_y_Z = joint_mat[2,1]    # This is the component of the CF2 y axis in the CF1 Z direction
        mat_z_X = joint_mat[0,2]    # This is the component of the CF2 z axis in the CF1 X direction
        mat_z_Y = joint_mat[1,2]    # This is the component of the CF2 z axis in the CF1 Y direction
        mat_z_Z = joint_mat[2,2]    # This is the component of the CF2 z axis in the CF1 Z direction
        # Make these values up into vectors projected on certain planes
        vec_x_on_XY = [mat_x_X, mat_x_Y]
        X_on_XY = [1, 0]
        vec_x_on_XZ = [mat_x_X, mat_x_Z]
        X_on_XZ = [1, 0]
        vec_z_on_ZY = [mat_z_Z, mat_z_Y]
        Z_on_ZY = [1, 0]
        vec_y_on_XY = [mat_y_X, mat_y_Y]
        Y_on_XY = [0, 1]

        # Calculate the angle of certain CF2 vectors on certain CF1 planes
        if np.linalg.norm(vec_x_on_XY) > 0.5:
            x_rel2_X_on_XY[row] = angle_between_two_2D_vecs(vec_x_on_XY, X_on_XY)
        else:
            # If the magnitude of the vector gets close to 1, its normal to plane and so angle is unstable
            x_rel2_X_on_XY[row] = np.nan

        if np.linalg.norm(vec_x_on_XZ) > 0.5:
            x_rel2_X_on_XZ[row] = angle_between_two_2D_vecs(vec_x_on_XZ, X_on_XZ)
        else:
            # If the magnitude of the vector gets close to 1, its normal to plane and so angle is unstable
            x_rel2_X_on_XZ[row] = np.nan

        if np.linalg.norm(vec_z_on_ZY) > 0.5:
            z_rel2_Z_on_ZY[row] = angle_between_two_2D_vecs(vec_z_on_ZY, Z_on_ZY)
        else:
            z_rel2_Z_on_ZY[row] = np.nan

        if np.linalg.norm(vec_y_on_XY) > 0.5:
            y_rel2_Y_on_XY[row] = angle_between_two_2D_vecs(vec_y_on_XY, Y_on_XY)
        else:
            y_rel2_Y_on_XY[row] = np.nan

    # Assign to clinically relevant joint angles
    abduction_all = y_rel2_Y_on_XY
    flexion_all = -z_rel2_Z_on_ZY
    rotation_elbow_down_all = x_rel2_X_on_XZ
    rotation_elbow_up_all = -z_rel2_Z_on_ZY

    return abduction_all, flexion_all, rotation_elbow_down_all, rotation_elbow_up_all


def trim_vec_prof_angles(abduction_all, flexion_all, rotation_elbow_down_all, rotation_elbow_up_all,
                         Eul_angle1_all, Eul_angle2_all):

    # Name the three euler angles to be used as references
    plane_of_elevation = Eul_angle1_all
    elevation = Eul_angle2_all

    # Discount values of the projected vector to avoid singularities and only focus on angles on interest

    # Remove rotation values if abduction or flexion is above 45 degrees
    rotation_elbow_down_keep_conditions = (elevation < 45)
    rotation_elbow_down = np.where(rotation_elbow_down_keep_conditions, rotation_elbow_down_all, np.nan)

    # Remove abduction values whenever elevation is above 45 AND plane of elevation is outwith -45 to 45
    abduction_discard_conditions1 = (elevation > 45) & (plane_of_elevation > 45)
    abduction_discard_conditions2 = (elevation > 45) & (plane_of_elevation < -45)
    abduction_int = np.where(abduction_discard_conditions1, np.nan, abduction_all)
    abduction = np.where(abduction_discard_conditions2, np.nan, abduction_int)

    # Remove flexion values whenever elevation is above 45deg AND plane of elevation is within -45 to 45
    flexion_discard_conditions1 = (elevation > 45) & (plane_of_elevation < 45) & (plane_of_elevation > -45)
    flexion_int_1 = np.where(flexion_discard_conditions1, np.nan, flexion_all)
    # Also Remove flexion values whenever int/ext rotation is outwith -45 to 45
    flexion_discard_conditions2 = (rotation_elbow_down > 45) & (elevation < 45)
    flexion_discard_conditions3 = (rotation_elbow_down < -45) & (elevation < 45)
    flexion_int_2 = np.where(flexion_discard_conditions2, np.nan, flexion_int_1)
    flexion = np.where(flexion_discard_conditions3, np.nan, flexion_int_2)

    # Remove these rotation values if we're not in abduction - i.e. only keep if elevation is over 45 and plane of elevation is within -45 to 45
    rotation_elbow_up_keep_condition1 = (elevation > 45) & (plane_of_elevation < 45) & (plane_of_elevation > -45)
    rotation_elbow_up = np.where(rotation_elbow_up_keep_condition1, rotation_elbow_up_all, np.nan)

    return abduction, flexion, rotation_elbow_down, rotation_elbow_up


def find_RMSE_of_error_array(error_arr):
    if len(error_arr) != 0:
        RMSE = (sum(np.square(error_arr)) / len(error_arr)) ** 0.5
    else:
        RMSE = 0
    return RMSE


def find_max_in_error_array(error_arr):
    if len(error_arr) != 0:
        max = np.amax(error_arr)
    else:
        max = 0
    return max



def convert_scipy_to_scalar_first_np_quat(scipy):
    arr = np.array([[scipy.as_quat()[3], scipy.as_quat()[0], scipy.as_quat()[1], scipy.as_quat()[2]]])
    return arr



""" Plotting Functions"""


def plot_compare_any_JAs(OMC_angle, IMU_angle, time, start_time, end_time,
                         figure_results_dir, joint_name):

    label = joint_name.replace('_', ' ').title()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    # Calculate cross-correlation lag and shift IMU data
    lag = get_cross_cor_lag(OMC_angle, IMU_angle)
    if lag < 0:
        IMU_angle = IMU_angle[-lag:]    # Remove first n values from IMU data
        OMC_angle = OMC_angle[:lag]     # Remove last n values from OMC data
        time = time[:lag]               # Remove last n values from time array
    print(f" Applied a shift of {lag} to {joint_name} IMU data")

    # Get the peaks and troughs
    OMC_peaks, OMC_peak_inds = get_peaks_or_troughs(OMC_angle, peak_or_trough='peak', data_type='OMC')
    IMU_peaks, IMU_peak_inds = get_peaks_or_troughs(IMU_angle, peak_or_trough='peak', data_type='IMU')
    OMC_troughs, OMC_trough_inds = get_peaks_or_troughs(OMC_angle, peak_or_trough='trough', data_type='OMC')
    IMU_troughs, IMU_trough_inds = get_peaks_or_troughs(IMU_angle, peak_or_trough='trough', data_type='IMU')

    # Get the mean peak/trough error
    if joint_name not in ('thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation'):
        plot_peaks = True
        mean_peak_error = get_peak_and_trough_errors(OMC_peaks, IMU_peaks)
        mean_trough_error = get_peak_and_trough_errors(OMC_troughs, IMU_troughs)
        # Check we have enough peaks/troughs, then calculate mean
        if any(len(var) < 4 for var in [OMC_peaks, IMU_peaks, OMC_troughs, IMU_troughs]):
            print(f"WARNING: No/not enough peaks found for {joint_name} (less than 4)")

    # Don't bother plotting/getting peak values for thorax angles
    else:
        plot_peaks = False
        mean_peak_error = 0     # Sub in value for plotting
        mean_trough_error = 0   # Sub in value for plotting


    # Get error metrics
    error_angle1 = abs(OMC_angle - IMU_angle)       # Calculate error array
    R = get_pearsonr(OMC_angle, IMU_angle)          # Calculate Pearson correlation coefficient
    RMSE_angle1 = get_RMSE(error_angle1)            # Calculate RMSE
    max_error_angle1 = np.nanmax(error_angle1)      # Calculate max error


    # Create figure
    fig, axs = plt.subplots(2, 1, figsize=(16,8), height_ratios=[7,3])


    """ Plot joint angles """

    # Axs 0 settings
    axs[0].set_title(label)
    axs[0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
    axs[0].legend(['OMC', 'IMU'])
    axs[0].grid(color="lightgrey")

    # Plot the joint angles
    axs[0].plot(time, OMC_angle)
    axs[0].plot(time, IMU_angle)

    # Plot the peaks and troughs
    if plot_peaks == True:
        axs[0].plot(time[OMC_trough_inds], OMC_troughs, "x", c='blue')
        axs[0].plot(time[OMC_peak_inds], OMC_peaks, "x", c='blue')
        axs[0].plot(time[IMU_trough_inds], IMU_troughs, "x", c='orange')
        axs[0].plot(time[IMU_peak_inds], IMU_peaks, "x", c='orange')

    # Annotate with mean peak/trough error
    y_min, y_max = axs[0].get_ylim()
    y_mid = (y_min + y_max) / 2
    if plot_peaks == True:
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid+10,
                    "Mean peak\nerror = " + str(round(mean_peak_error,1)) + " deg")
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid-10,
                    "Mean trough\nerror = " + str(round(mean_trough_error,1)) + " deg")


    """ Plot error graphs """

    # Axs 1 settings
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*max_error_angle1])))
    axs[1].grid(color="lightgrey")

    # Plot to time-series error
    axs[1].scatter(time, error_angle1, s=0.4)

    # Plot RMSE error lines and text
    axs[1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_name + "_angles.png")
    plt.close()

    return RMSE_angle1, R, mean_peak_error, mean_trough_error




# Define a function to plot IMU vs OMC, with extra plot of errors to see distribution, using OpenSim coords
def plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     figure_results_dir, labelA, labelB, joint_of_interest):

    if joint_of_interest == "Thorax":
        ref1 = "TH_x"
        ref2 = "TH_z"
        ref3 = "TH_y"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "(Change in) Trunk Rotation"

    elif joint_of_interest == "Elbow":
        ref1 = "EL_x"
        ref2 = "PS_y"
        ref3 = "PS_y"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    # Extract coordinates from states table
    OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()
    OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()
    OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()
    IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()
    IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()
    IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()

    # Update trunk rotation angle to be the change in direction based on initial direction
    if joint_of_interest == "Thorax":
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    # Calculate Pearson correlation coefficient
    R_1 = pearsonr(OMC_angle1, IMU_angle1)[0]
    R_2 = pearsonr(OMC_angle2, IMU_angle2)[0]
    R_3 = pearsonr(OMC_angle3, IMU_angle3)[0]

    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)
    error_angle2 = abs(OMC_angle2 - IMU_angle2)
    error_angle3 = abs(OMC_angle3 - IMU_angle3)

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1)) / len(error_angle1)) ** 0.5
    RMSE_angle2 = (sum(np.square(error_angle2)) / len(error_angle2)) ** 0.5
    RMSE_angle3 = (sum(np.square(error_angle3)) / len(error_angle3)) ** 0.5
    max_error_angle1 = np.amax(error_angle1)
    max_error_angle2 = np.amax(error_angle2)
    max_error_angle3 = np.amax(error_angle3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    axs[0,0].plot(time, OMC_angle1)
    axs[0,0].plot(time, IMU_angle1)

    axs[1,0].plot(time, OMC_angle2)
    axs[1,0].plot(time, IMU_angle2)

    axs[2,0].plot(time, OMC_angle3)
    axs[2,0].plot(time, IMU_angle3)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)

    for i in range(0, 3):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([labelA, labelB])
        axs[i,0].grid(color="lightgrey")

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1, s=0.4)
    axs[1,1].scatter(time, error_angle2, s=0.4)
    axs[2,1].scatter(time, error_angle3, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE*1.1):
            text_placement = RMSE*1.1
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 3):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + "_angles.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3


# Define a function to plot IMU vs OMC for the shoulder joint euler anlges
def plot_compare_JAs_shoulder_eulers(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                                     time, start_time, end_time, figure_results_dir, labelA, labelB):

    label1 = "Plane of Elevation (Y)"
    label2 = "Elevation (Z)"
    label3 = "Internal/External Rotation (Y)"

    OMC_angle1_all, OMC_angle2, OMC_angle3_all = get_JA_euls_from_quats(thorax_OMC, humerus_OMC, eul_seq="YZY")
    IMU_angle1_all, IMU_angle2, IMU_angle3_all = get_JA_euls_from_quats(thorax_IMU, humerus_IMU, eul_seq="YZY")

    # Discount 1st and 3rd euler if 2nd euler (elevation) is below threshold
    elevation_threshold = 45
    OMC_angle1 = np.where(OMC_angle2 > elevation_threshold, OMC_angle1_all, np.nan)
    OMC_angle3 = np.where(OMC_angle2 > elevation_threshold, OMC_angle3_all, np.nan)
    IMU_angle1 = np.where(OMC_angle2 > elevation_threshold, IMU_angle1_all, np.nan)
    IMU_angle3 = np.where(OMC_angle2 > elevation_threshold, IMU_angle3_all, np.nan)


    # Remove rows from both arrays where there's a nan in either
    OMC_angle1_for_R_calc = np.delete(OMC_angle1, np.union1d(np.where(np.isnan(OMC_angle1)), np.where(np.isnan(IMU_angle1))))
    IMU_angle1_for_R_calc = np.delete(IMU_angle1, np.union1d(np.where(np.isnan(OMC_angle1)), np.where(np.isnan(IMU_angle1))))
    OMC_angle2_for_R_calc = np.delete(OMC_angle2, np.union1d(np.where(np.isnan(OMC_angle2)), np.where(np.isnan(IMU_angle2))))
    IMU_angle2_for_R_calc = np.delete(IMU_angle2, np.union1d(np.where(np.isnan(OMC_angle2)), np.where(np.isnan(IMU_angle2))))
    OMC_angle3_for_R_calc = np.delete(OMC_angle3, np.union1d(np.where(np.isnan(OMC_angle3)), np.where(np.isnan(IMU_angle3))))
    IMU_angle3_for_R_calc = np.delete(IMU_angle3, np.union1d(np.where(np.isnan(OMC_angle3)), np.where(np.isnan(IMU_angle3))))


    # Calculate Pearson correlation coefficient
    # Check if there is at least some data to compare
    if len(np.union1d(np.where(~np.isnan(OMC_angle1)), np.where(~np.isnan(IMU_angle1)))) == 0:
        R_1 = 0
    else:
        R_1 = pearsonr(OMC_angle1_for_R_calc, IMU_angle1_for_R_calc)[0]
    if len(np.union1d(np.where(~np.isnan(OMC_angle2)), np.where(~np.isnan(IMU_angle2)))) == 0:
        R_2 = 0
    else:
        R_2 = pearsonr(OMC_angle2_for_R_calc, IMU_angle2_for_R_calc)[0]
    if len(np.union1d(np.where(~np.isnan(OMC_angle3)), np.where(~np.isnan(IMU_angle3)))) == 0:
        R_3 = 0
    else:
        R_3 = pearsonr(OMC_angle3_for_R_calc, IMU_angle3_for_R_calc)[0]


    # Calculate error arrays
    error_angle1_with_nans = abs(OMC_angle1 - IMU_angle1)
    error_angle2_with_nans = abs(OMC_angle2 - IMU_angle2)
    error_angle3_with_nans = abs(OMC_angle3 - IMU_angle3)
    # Remove any rows where there's nan values
    error_angle1 = error_angle1_with_nans[~np.isnan(error_angle1_with_nans)]
    error_angle2 = error_angle2_with_nans[~np.isnan(error_angle2_with_nans)]
    error_angle3 = error_angle3_with_nans[~np.isnan(error_angle3_with_nans)]

    # Calculate RMSE
    RMSE_angle1 = find_RMSE_of_error_array(error_angle1)
    RMSE_angle2 = find_RMSE_of_error_array(error_angle2)
    RMSE_angle3 = find_RMSE_of_error_array(error_angle3)
    max_error_angle1 = find_max_in_error_array(error_angle1)
    max_error_angle2 = find_max_in_error_array(error_angle2)
    max_error_angle3 = find_max_in_error_array(error_angle3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    axs[0,0].plot(time, OMC_angle1)
    axs[0,0].plot(time, IMU_angle1)
    axs[0,0].plot(time, OMC_angle1_all, linestyle='dotted', c='lightgrey')
    axs[0,0].plot(time, IMU_angle1_all, linestyle='dotted', c='lightgrey')

    axs[1,0].plot(time, OMC_angle2)
    axs[1,0].plot(time, IMU_angle2)

    axs[2,0].plot(time, OMC_angle3)
    axs[2,0].plot(time, IMU_angle3)
    axs[2,0].plot(time, OMC_angle3_all, linestyle='dotted', c='lightgrey')
    axs[2,0].plot(time, IMU_angle3_all, linestyle='dotted', c='lightgrey')

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)

    for i in range(0, 3):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([labelA, labelB])
        axs[i,0].grid(color="lightgrey")

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1_with_nans, s=0.4)
    axs[1,1].scatter(time, error_angle2_with_nans, s=0.4)
    axs[2,1].scatter(time, error_angle3_with_nans, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE * 1.1):
            text_placement = RMSE * 1.1
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 3):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])), xlim=(start_time, end_time))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + r"\HT_Eulers.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3


# Define a function to plot IMU vs OMC model body orientation errors (single angle quaternion difference)
def plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                           heading_offset, time, start_time, end_time, figure_results_dir):

    label1 = "Thorax Orientation Error" + " (heading offset applied: " + str(round(heading_offset*180/np.pi,1)) + "deg)"
    label2 = "Humerus Orientation Error"
    label3 = "Radius Orientation Error"

    # Apply heading offset to all IMU frames
    heading_offset_R = R.from_euler('y', [heading_offset])

    thorax_IMU_R = R.from_quat(thorax_IMU[:, [1, 2, 3, 0]])
    humerus_IMU_R = R.from_quat(humerus_IMU[:, [1, 2, 3, 0]])
    radius_IMU_R = R.from_quat(radius_IMU[:, [1, 2, 3, 0]])
    thorax_OMC_R = R.from_quat(thorax_OMC[:, [1, 2, 3, 0]])
    humerus_OMC_R = R.from_quat(humerus_OMC[:, [1, 2, 3, 0]])
    radius_OMC_R = R.from_quat(radius_OMC[:, [1, 2, 3, 0]])

    thorax_IMU_rotated = thorax_IMU_R*heading_offset_R
    humerus_IMU_rotated = humerus_IMU_R*heading_offset_R
    radius_IMU_rotated = radius_IMU_R*heading_offset_R

    def find_single_angle_diff_between_two_CFs(body1, body2):
        diff = body1.inv()*body2
        angle = diff.magnitude()*180/np.pi
        return angle

    thorax_ori_error = find_single_angle_diff_between_two_CFs(thorax_OMC_R, thorax_IMU_rotated)
    humerus_ori_error = find_single_angle_diff_between_two_CFs(humerus_OMC_R, humerus_IMU_rotated)
    radius_ori_error = find_single_angle_diff_between_two_CFs(radius_OMC_R, radius_IMU_rotated)


    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(thorax_ori_error)) / len(thorax_ori_error)) ** 0.5
    RMSE_angle2 = (sum(np.square(humerus_ori_error)) / len(humerus_ori_error)) ** 0.5
    RMSE_angle3 = (sum(np.square(radius_ori_error)) / len(radius_ori_error)) ** 0.5
    max_error_angle1 = np.amax(thorax_ori_error)
    max_error_angle2 = np.amax(humerus_ori_error)
    max_error_angle3 = np.amax(radius_ori_error)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(14,9))

    # Plot error graphs

    axs[0].scatter(time, thorax_ori_error, s=0.4)
    axs[1].scatter(time, humerus_ori_error, s=0.4)
    axs[2].scatter(time, radius_ori_error, s=0.4)

    axs[0].set_title(label1)
    axs[1].set_title(label2)
    axs[2].set_title(label3)

    # Plot RMSE error lines and text
    axs[0].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE + 3):
            text_placement = RMSE*1.1
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    y_lim_list = np.array([max_error_angle1, max_error_angle2, max_error_angle3])
    for i in range(0, 3):
        axs[i].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0, 1.1*y_lim_list[i]))
        axs[i].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + r"\Body_Orientation_Diff.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3


# Define a function to plot IMU vs OMC HT angles, based on projected vector directions
def plot_vector_HT_angles(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                          time, start_time, end_time, figure_results_dir, labelA, labelB):

    label1 = "Abduction (y_rel2_Y_on_XY)"
    label2 = "Flexion (z_rel2_Z_on_ZY)"
    label3 = "Rotation - Elbow Down (x_rel2_X_on_XZ)"
    label4 = "Rotation - Elbow Up (z_rel2_Z_on_ZY)"

    # Calculate HT Euler angles (to be used as a reference)
    OMC_angle1_all, OMC_angle2_all, OMC_angle3_all = get_JA_euls_from_quats(thorax_OMC, humerus_OMC, eul_seq="YZY")
    IMU_angle1_all, IMU_angle2_all, IMU_angle3_all = get_JA_euls_from_quats(thorax_IMU, humerus_IMU, eul_seq="YZY")

    # Calculate the projected vector angles based on the body orientations of thorax and humerus
    abduction_all_OMC, flexion_all_OMC, rotation_elbow_down_all_OMC, rotation_elbow_up_all_OMC = \
        get_vec_angles_from_two_CFs(thorax_OMC, humerus_OMC)
    abduction_all_IMU, flexion_all_IMU, rotation_elbow_down_all_IMU, rotation_elbow_up_all_IMU = \
        get_vec_angles_from_two_CFs(thorax_IMU, humerus_IMU)

    # Trim the arrays above based on criteria to avoid singularities and only focus on angles of interest
    abduction_OMC, flexion_OMC, rotation_elbow_down_OMC, rotation_elbow_up_OMC = \
        trim_vec_prof_angles(abduction_all_OMC, flexion_all_OMC, rotation_elbow_down_all_OMC, rotation_elbow_up_all_OMC,
                             OMC_angle1_all, OMC_angle2_all)
    abduction_IMU, flexion_IMU, rotation_elbow_down_IMU, rotation_elbow_up_IMU = \
        trim_vec_prof_angles(abduction_all_IMU, flexion_all_IMU, rotation_elbow_down_all_IMU, rotation_elbow_up_all_IMU,
                             IMU_angle1_all, IMU_angle2_all)

    # Remove rows from both arrays where there's a nan in either
    abduction_OMC_for_R_calc = np.delete(abduction_OMC, np.union1d(np.where(np.isnan(abduction_OMC)), np.where(np.isnan(abduction_IMU))))
    abduction_IMU_for_R_calc = np.delete(abduction_IMU, np.union1d(np.where(np.isnan(abduction_OMC)), np.where(np.isnan(abduction_IMU))))
    flexion_OMC_for_R_calc = np.delete(flexion_OMC, np.union1d(np.where(np.isnan(flexion_OMC)), np.where(np.isnan(flexion_IMU))))
    flexion_IMU_for_R_calc = np.delete(flexion_IMU, np.union1d(np.where(np.isnan(flexion_OMC)), np.where(np.isnan(flexion_IMU))))
    rotation_elbow_down_OMC_for_R_calc = np.delete(rotation_elbow_down_OMC, np.union1d(np.where(np.isnan(rotation_elbow_down_OMC)), np.where(np.isnan(rotation_elbow_down_IMU))))
    rotation_elbow_down_IMU_for_R_calc = np.delete(rotation_elbow_down_IMU, np.union1d(np.where(np.isnan(rotation_elbow_down_OMC)), np.where(np.isnan(rotation_elbow_down_IMU))))
    rotation_elbow_up_OMC_for_R_calc = np.delete(rotation_elbow_up_OMC, np.union1d(np.where(np.isnan(rotation_elbow_up_OMC)), np.where(np.isnan(rotation_elbow_up_IMU))))
    rotation_elbow_up_IMU_for_R_calc = np.delete(rotation_elbow_up_IMU, np.union1d(np.where(np.isnan(rotation_elbow_up_OMC)), np.where(np.isnan(rotation_elbow_up_IMU))))


    # Calculate Pearson correlation coefficient
    # Check if there is at least some data to compare
    if len(np.union1d(np.where(~np.isnan(abduction_OMC)), np.where(~np.isnan(abduction_IMU)))) == 0:
        R_1 = 0
    else:
        R_1 = pearsonr(abduction_OMC_for_R_calc, abduction_IMU_for_R_calc)[0]
    if len(np.union1d(np.where(~np.isnan(flexion_OMC)), np.where(~np.isnan(flexion_IMU)))) == 0:
        R_2 = 0
    else:
        R_2 = pearsonr(flexion_OMC_for_R_calc, flexion_IMU_for_R_calc)[0]
    if len(np.union1d(np.where(~np.isnan(rotation_elbow_down_OMC)), np.where(~np.isnan(rotation_elbow_down_IMU)))) == 0:
        R_3 = 0
    else:
        R_3 = pearsonr(rotation_elbow_down_OMC_for_R_calc, rotation_elbow_down_IMU_for_R_calc)[0]
    if len(np.union1d(np.where(~np.isnan(rotation_elbow_up_OMC)), np.where(~np.isnan(rotation_elbow_up_IMU)))) == 0:
        R_4 = 0
    else:
        R_4 = pearsonr(rotation_elbow_up_OMC_for_R_calc, rotation_elbow_up_IMU_for_R_calc)[0]

    # Calculate error arrays
    error_angle1_with_nans = abs(abduction_OMC - abduction_IMU)
    error_angle2_with_nans = abs(flexion_OMC - flexion_IMU)
    error_angle3_with_nans = abs(rotation_elbow_down_OMC - rotation_elbow_down_IMU)
    error_angle4_with_nans = abs(rotation_elbow_up_OMC - rotation_elbow_up_IMU)

    # Remove any rows where there's nan values
    error_angle1 = error_angle1_with_nans[~np.isnan(error_angle1_with_nans)]
    error_angle2 = error_angle2_with_nans[~np.isnan(error_angle2_with_nans)]
    error_angle3 = error_angle3_with_nans[~np.isnan(error_angle3_with_nans)]
    error_angle4 = error_angle4_with_nans[~np.isnan(error_angle4_with_nans)]

    # Calculate RMSE
    RMSE_angle1 = find_RMSE_of_error_array(error_angle1)
    RMSE_angle2 = find_RMSE_of_error_array(error_angle2)
    RMSE_angle3 = find_RMSE_of_error_array(error_angle3)
    RMSE_angle4 = find_RMSE_of_error_array(error_angle4)
    max_error_angle1 = find_max_in_error_array(error_angle1)
    max_error_angle2 = find_max_in_error_array(error_angle2)
    max_error_angle3 = find_max_in_error_array(error_angle3)
    max_error_angle4 = find_max_in_error_array(error_angle4)

    # Create figure with three subplots
    fig, axs = plt.subplots(4, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    line1, = axs[0,0].plot(time, abduction_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[0,0].plot(time, abduction_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[0,0].plot(time, abduction_OMC)
    line4, = axs[0,0].plot(time, abduction_IMU)

    line1, = axs[1,0].plot(time, flexion_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[1,0].plot(time, flexion_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[1,0].plot(time, flexion_OMC)
    line4, = axs[1,0].plot(time, flexion_IMU)

    line1, = axs[2,0].plot(time, rotation_elbow_down_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[2,0].plot(time, rotation_elbow_down_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[2,0].plot(time, rotation_elbow_down_OMC)
    line4, = axs[2,0].plot(time, rotation_elbow_down_IMU)

    line1, = axs[3,0].plot(time, rotation_elbow_up_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[3,0].plot(time, rotation_elbow_up_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[3,0].plot(time, rotation_elbow_up_OMC)
    line4, = axs[3,0].plot(time, rotation_elbow_up_IMU)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)
    axs[3,0].set_title(label4)

    for i in range(0, 4):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([line3, line4], [labelA, labelB])
        axs[i,0].grid(color="lightgrey")

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1_with_nans, s=0.4)
    axs[1,1].scatter(time, error_angle2_with_nans, s=0.4)
    axs[2,1].scatter(time, error_angle3_with_nans, s=0.4)
    axs[3,1].scatter(time, error_angle4_with_nans, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")
    axs[3,1].axhline(y=RMSE_angle4, linewidth=2, c="red")
    axs[3,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle4, "RMSE = " + str(round(RMSE_angle4,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE + 3):
            text_placement = RMSE + 3
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    y_max_line_placement_4 = y_max_line_placement(max_error_angle4)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")
    axs[3,1].axhline(y=y_max_line_placement_4, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    y_max_text_placement_4 = y_max_text_placement(max_error_angle4, RMSE_angle4)
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")
    axs[3,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_4, "Max = " + str(round(max_error_angle4,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 4):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])), xlim=(start_time, end_time))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + r"\HT_Vectors.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3


# Define a function to plot the error between real IMUs and cluster orientations, representing 'perfect' IMUs
def plot_compare_real_vs_perfect(IMU1_diff, IMU2_diff, IMU3_diff, figure_results_dir):

    label1 = "Thorax IMU orientation error"
    label2 = "Humerus IMU orientation error"
    label3 = "Forearm IMU orientation error"

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(IMU1_diff[~np.isnan(IMU1_diff)])) / len(IMU1_diff[~np.isnan(IMU1_diff)])) ** 0.5
    RMSE_angle2 = (sum(np.square(IMU2_diff[~np.isnan(IMU2_diff)])) / len(IMU2_diff[~np.isnan(IMU2_diff)])) ** 0.5
    RMSE_angle3 = (sum(np.square(IMU3_diff[~np.isnan(IMU3_diff)])) / len(IMU3_diff[~np.isnan(IMU3_diff)])) ** 0.5
    max_error_angle1 = np.nanmax(IMU1_diff)
    max_error_angle2 = np.nanmax(IMU2_diff)
    max_error_angle3 = np.nanmax(IMU3_diff)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(14, 9))

    # Plot error graphs
    time1 = np.array(range(len(IMU1_diff)))*0.01
    time2 = np.array(range(len(IMU2_diff)))*0.01
    time3 = np.array(range(len(IMU3_diff)))*0.01
    start_time1 = time1[0]
    end_time1 = time1[-1]
    start_time2 = time2[0]
    end_time2 = time2[-1]
    start_time3 = time3[0]
    end_time3 = time3[-1]

    axs[0].scatter(time1, IMU1_diff, s=0.4)
    axs[1].scatter(time2, IMU2_diff, s=0.4)
    axs[2].scatter(time3, IMU3_diff, s=0.4)

    axs[0].set_title(label1)
    axs[1].set_title(label2)
    axs[2].set_title(label3)

    # Plot RMSE error lines and text
    axs[0].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0].text(time1[-1] + 0.1 * (end_time1 - start_time1), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1, 1)) + " deg")
    axs[1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1].text(time2[-1] + 0.1 * (end_time2 - start_time2), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2, 1)) + " deg")
    axs[2].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2].text(time3[-1] + 0.1 * (end_time3 - start_time3), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3, 1)) + " deg")


    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement


    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE + 3):
            text_placement = RMSE * 1.1
        else:
            text_placement = max_error
        return text_placement


    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0].text(time1[-1] + 0.1 * (end_time1 - start_time1), y_max_text_placement_1,
                "Max = " + str(round(max_error_angle1, 1)) + " deg")
    axs[1].text(time2[-1] + 0.1 * (end_time2 - start_time2), y_max_text_placement_2,
                "Max = " + str(round(max_error_angle2, 1)) + " deg")
    axs[2].text(time3[-1] + 0.1 * (end_time3 - start_time3), y_max_text_placement_3,
                "Max = " + str(round(max_error_angle3, 1)) + " deg")

    # Set a shared x axis
    y_lim_list = np.array([max_error_angle1, max_error_angle2, max_error_angle3])
    start_time_list = [start_time1, start_time2, start_time3]
    end_time_list = [end_time1, end_time2, end_time3]
    for i in range(0, 3):
        axs[i].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0, 1.1 * y_lim_list[i]), xlim=(start_time_list[i], end_time_list[i]))
        axs[i].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + r"\IMU_Orientation_Diff.png")

    return RMSE_angle1, RMSE_angle2, RMSE_angle3




# Define a function to plot the error between real IMUs and cluster orientations, representing 'perfect' IMUs
def plot_compare_real_vs_perfect_eulers(IMU_euls, OMC_euls, figure_results_dir, file_name):

    label1 = "Euler 1"
    label2 = "Euler 2"
    label3 = "Euler 3"

    IMU1_eul = IMU_euls[:,0]
    IMU2_eul = IMU_euls[:,1]
    IMU3_eul = IMU_euls[:,2]
    OMC1_eul = OMC_euls[:,0]
    OMC2_eul = OMC_euls[:,1]
    OMC3_eul = OMC_euls[:,2]

    IMU1_diff = IMU1_eul - OMC1_eul
    IMU2_diff = IMU2_eul - OMC2_eul
    IMU3_diff = IMU3_eul - OMC3_eul

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(IMU1_diff[~np.isnan(IMU1_diff)])) / len(IMU1_diff[~np.isnan(IMU1_diff)])) ** 0.5
    RMSE_angle2 = (sum(np.square(IMU2_diff[~np.isnan(IMU2_diff)])) / len(IMU2_diff[~np.isnan(IMU2_diff)])) ** 0.5
    RMSE_angle3 = (sum(np.square(IMU3_diff[~np.isnan(IMU3_diff)])) / len(IMU3_diff[~np.isnan(IMU3_diff)])) ** 0.5
    max_error_angle1 = np.nanmax(IMU1_diff)
    max_error_angle2 = np.nanmax(IMU2_diff)
    max_error_angle3 = np.nanmax(IMU3_diff)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(14, 9))

    # Plot error graphs
    time1 = np.array(range(len(IMU1_eul)))*0.01
    time2 = np.array(range(len(IMU2_eul)))*0.01
    time3 = np.array(range(len(IMU3_eul)))*0.01
    start_time1 = time1[0]
    end_time1 = time1[-1]
    start_time2 = time2[0]
    end_time2 = time2[-1]
    start_time3 = time3[0]
    end_time3 = time3[-1]

    axs[0].scatter(time1, IMU1_eul, s=0.4)
    axs[1].scatter(time2, IMU2_eul, s=0.4)
    axs[2].scatter(time3, IMU3_eul, s=0.4)
    axs[0].scatter(time1, OMC1_eul, s=0.4)
    axs[1].scatter(time2, OMC2_eul, s=0.4)
    axs[2].scatter(time3, OMC3_eul, s=0.4)

    axs[0].set_title(label1)
    axs[1].set_title(label2)
    axs[2].set_title(label3)

    # Plot RMSE error lines and text
    axs[0].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0].text(time1[-1] + 0.1 * (end_time1 - start_time1), IMU1_diff[0], "RMSE = " + str(round(RMSE_angle1, 1)) + " deg")
    axs[1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1].text(time2[-1] + 0.1 * (end_time2 - start_time2), IMU2_diff[0], "RMSE = " + str(round(RMSE_angle2, 1)) + " deg")
    axs[2].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2].text(time3[-1] + 0.1 * (end_time3 - start_time3), IMU3_diff[0], "RMSE = " + str(round(RMSE_angle3, 1)) + " deg")

    #
    # # Functions to define placement of max error annotation
    # def y_max_line_placement(max_error):
    #     if max_error > 40:
    #         line_placement = 40
    #     else:
    #         line_placement = max_error
    #     return line_placement
    #
    #
    # def y_max_text_placement(max_error, RMSE):
    #     if max_error > 40:
    #         text_placement = 40
    #     elif max_error < (RMSE + 3):
    #         text_placement = RMSE * 1.1
    #     else:
    #         text_placement = max_error
    #     return text_placement

    #
    # # Plot max error lines
    # y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    # y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    # y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    # axs[0].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    # axs[1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    # axs[2].axhline(y=y_max_line_placement_3, linewidth=1, c="red")
    #
    # # Plot max error text
    # y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    # y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    # y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    # axs[0].text(time1[-1] + 0.1 * (end_time1 - start_time1), y_max_text_placement_1,
    #             "Max = " + str(round(max_error_angle1, 1)) + " deg")
    # axs[1].text(time2[-1] + 0.1 * (end_time2 - start_time2), y_max_text_placement_2,
    #             "Max = " + str(round(max_error_angle2, 1)) + " deg")
    # axs[2].text(time3[-1] + 0.1 * (end_time3 - start_time3), y_max_text_placement_3,
    #             "Max = " + str(round(max_error_angle3, 1)) + " deg")

    # # Set a shared x axis
    # y_max_list = np.array([np.nanmax(IMU1_eul), np.nanmax(IMU2_eul), np.nanmax(IMU3_eul)])
    # y_min_list = np.array([np.nanmin(IMU1_eul), np.nanmin(IMU2_eul), np.nanmin(IMU3_eul)])
    # start_time_list = [start_time1, start_time2, start_time3]
    # end_time_list = [end_time1, end_time2, end_time3]
    # for i in range(0, 3):
    #     axs[i].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0, 1.1 * y_lim_list[i]), xlim=(start_time_list[i], end_time_list[i]))
    #     axs[i].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    print(f"Writing figure to results: {file_name}")
    fig.savefig(figure_results_dir + "\\" + file_name + "_IMU_Euler_Diff.png")

    return RMSE_angle1, RMSE_angle2, RMSE_angle3




# Use SLERP to fill gaps in quaternion data (awkward because scipy.spatial.transform.Rotation can't handle nans,
# so we can't just make a sequence of scipy Rs from an array of quaternions if it has any nans in)
def fill_nan_gaps_in_quat_df(IMU_df, max_gap):
    row = 0
    while row < len(IMU_df):

        if IMU_df.iloc[row].isna().any() == True:  # If there are any nan values

            # Get closest previous non-nan rot
            prev_rot_row = row - 1
            prev_rot = IMU_df.iloc[prev_rot_row, [1, 2, 3, 0]].to_numpy()

            # Get the next non-nan rot
            inner_row = row + 1
            while inner_row < len(IMU_df):  # Iterate through next rows until you find next non-nan, until end of df
                if IMU_df.iloc[inner_row].notna().any():
                    break
                inner_row += 1

            if inner_row == len(IMU_df):  # If we have reached the end of the dataframe and there's non more non-nans
                for fill_row in range(row, inner_row):
                    IMU_df.iloc[fill_row] = IMU_df.iloc[
                        prev_rot_row]  # Fill in all the last nans with the previous non-nan rot

            else:
                next_rot_row = inner_row  # This is the next available non-nan row
                next_rot = IMU_df.iloc[next_rot_row, [1, 2, 3,
                                                      0]].to_numpy()  # This is the next available quat to use for the interpolation

                # Turn the two quats into scipy Rs
                prev_rot_R = R.from_quat(prev_rot)
                next_rot_R = R.from_quat(next_rot)

                # Create sequence of the two scipy Rs, ready to be used in the slerp
                key_rots = []
                key_rots.append(prev_rot_R)
                key_rots.append(next_rot_R)
                key_rots_R = R.concatenate(key_rots)  # The start and finish Rs

                # Create the inputs for the slerp
                key_times = [prev_rot_row, next_rot_row]  # The start and finish times corresponding to the Rs
                new_times = list(range(row, next_rot_row))  # The rows where we need to create new Rs
                slerp = Slerp(key_times, key_rots_R)  # Create the Slerp
                interp_rots = slerp(new_times)  # Create the new Rs from the previous and next known non-nan Rs

                # Turn the Rs back into quaternions to update the original dataframe
                interp_quats = interp_rots.as_quat()

                # Add the new quats into the dataframe where previously there were nans
                for fill_row in range(row, next_rot_row):
                    IMU_df.iloc[fill_row] = interp_quats[(fill_row - row), [3, 0, 1, 2]]

            if inner_row - row > max_gap:
                print(f"There was a gap in the data larger than {max_gap} samples ({inner_row-row} samples), "
                      f"which has been filled with SLERP.")

            # Update row to the next non-nan row so the while loop continues looking for the next gap
            row = inner_row  # This is row with next non-nan rot, or it's = len(IMU_df), so while loop will break

        else:
            row += 1

    return IMU_df


def run_analyze_tool(analyze_settings_template_file, results_dir, model_file_path, mot_file_path, start_time, end_time):

    analyze_Tool = osim.AnalyzeTool(analyze_settings_template_file)
    analyze_Tool.updAnalysisSet().cloneAndAppend(osim.BodyKinematics())
    analyze_Tool.setModelFilename(model_file_path)
    analyze_Tool.setName("analyze")
    analyze_Tool.setCoordinatesFileName(mot_file_path)
    analyze_Tool.setStartTime(start_time)
    analyze_Tool.setFinalTime(end_time)
    analyze_Tool.setResultsDir(results_dir)
    print('Running Analyze Tool...')
    analyze_Tool.run()
    print('Analyze Tool run finished.')


def get_body_quats_from_analysis_sto(analysis_sto_path, start_time, end_time):

    # Read in the analysis .sto file with pos and ori data for each model body
    analysis_table = osim.TimeSeriesTable(analysis_sto_path)   # Read in new states

    # Trim based on start and end times
    analysis_table.trim(start_time, end_time)

    # Create an array of the XYZ Eulers used to define the body oris
    thorax_Ox = analysis_table.getDependentColumn('thorax_Ox').to_numpy()
    thorax_Oy = analysis_table.getDependentColumn('thorax_Oy').to_numpy()
    thorax_Oz = analysis_table.getDependentColumn('thorax_Oz').to_numpy()
    humerus_Ox = analysis_table.getDependentColumn('humerus_r_Ox').to_numpy()
    humerus_Oy = analysis_table.getDependentColumn('humerus_r_Oy').to_numpy()
    humerus_Oz = analysis_table.getDependentColumn('humerus_r_Oz').to_numpy()
    radius_Ox = analysis_table.getDependentColumn('radius_r_Ox').to_numpy()
    radius_Oy = analysis_table.getDependentColumn('radius_r_Oy').to_numpy()
    radius_Oz = analysis_table.getDependentColumn('radius_r_Oz').to_numpy()
    thorax_eulers = np.stack((thorax_Ox, thorax_Oy, thorax_Oz), axis=1)
    humerus_eulers = np.stack((humerus_Ox, humerus_Oy, humerus_Oz), axis=1)
    radius_eulers = np.stack((radius_Ox, radius_Oy, radius_Oz), axis=1)

    # Create an array of scipy Rotations
    thorax_R = R.from_euler('XYZ', thorax_eulers, degrees=True)
    humerus_R = R.from_euler('XYZ', humerus_eulers, degrees=True)
    radius_R = R.from_euler('XYZ', radius_eulers, degrees=True)

    thorax_quats = thorax_R.as_quat()[:,[1, 2, 3, 0]]
    humerus_quats = humerus_R.as_quat()[:,[1, 2, 3, 0]]
    radius_quats = radius_R.as_quat()[:,[1, 2, 3, 0]]

    return thorax_quats, humerus_quats, radius_quats


# Function used in IK_compare to trim data tables to same length
def trim_tables_if_diff_lengths(n, OMC_table, IMU_table):

    if n > 0:  # If OMC data is longer than IMU data
        for i in range(n):
            OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))  # Remove n rows from the OMC table
        print(f'Removed last {n} rows from OMC data')

    if n < 0:  # If IMU data is longer than OMC data
        for i in range(n):
            IMU_table.removeRowAtIndex((IMU_table.getNumRows() - 1))  # Remove n rows from the IMU table
        print(f'Removed last {n} rows from IMU data')

    return OMC_table, IMU_table

# Function used in IK_compare to trim data tables to same length
def trim_body_ori_data_to_same_length(n, thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC):

    # Remove n rows from the body ori data (trim to same length for easier plotting)
    for i in range(n):
        thorax_OMC = np.delete(thorax_OMC, [-1], 0)
        humerus_OMC = np.delete(humerus_OMC, [-1], 0)
        radius_OMC = np.delete(radius_OMC, [-1], 0)
        thorax_IMU = np.delete(thorax_IMU, [-1], 0)
        humerus_IMU = np.delete(humerus_IMU, [-1], 0)
        radius_IMU = np.delete(radius_IMU, [-1], 0)

    return thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC


# Function to calculate RMSE from an error array, dealing with any nans
def get_RMSE(error_arr):
    error_arr = error_arr[~np.isnan(error_arr)]
    RMSE = (sum(np.square(error_arr)) / len(error_arr)) ** 0.5
    return RMSE

# Function to calculate Pearson Corr Coeff, from two series, dealing with any nans
def get_pearsonr(OMC_angle, IMU_angle):
    # Remove any nans values present
    OMC_angle_no_nans = np.delete(OMC_angle, np.union1d(np.where(np.isnan(OMC_angle)), np.where(np.isnan(IMU_angle))))
    IMU_angle_no_nans = np.delete(IMU_angle, np.union1d(np.where(np.isnan(OMC_angle)), np.where(np.isnan(IMU_angle))))
    R = pearsonr(OMC_angle_no_nans, IMU_angle_no_nans)[0]
    return R

# Function to calculate the cross-correlation lag between two series, accounting for nans
def get_cross_cor_lag(x, y):

    # Remove elements from both arrays where either array is a nan
    x_nonans = np.delete(x, np.union1d(np.where(np.isnan(x)), np.where(np.isnan(y))))
    y_nonans = np.delete(y, np.union1d(np.where(np.isnan(x)), np.where(np.isnan(y))))

    # Run the cross correlation
    correlation = signal.correlate(x_nonans, y_nonans, mode="full")
    lags = signal.correlation_lags(x_nonans.size, y_nonans.size, mode="full")
    lag = lags[np.argmax(correlation)]  # Get the lag value at the index where correlation is largest

    # # Diagnosis/investigation code:
    # print(lags)
    # print(correlation)
    # # Get middle results around where lag = 0
    # indices_within_range = [index for index, lag in enumerate(lags) if -20 <= lag <= 20]
    # print(lags[indices_within_range])
    # print(correlation[indices_within_range])
    # fig, axs = plt.subplots(1, 1, figsize=(14,9))
    # axs.plot(lags, correlation)
    # axs.plot(x_nonans)
    # axs.plot(y_nonans)
    # plt.show()

    return lag


# Function for getting the peaks or troughs from an array
def get_peaks_or_troughs(angle_arr, peak_or_trough, data_type):

    prominence = 50

    if peak_or_trough == 'peak':
        x = angle_arr
    elif peak_or_trough == 'trough':
        x = -angle_arr
    else:
        print('peak/trough not written correctly')

    peak_inds, _ = find_peaks(x, prominence=prominence, wlen=1000)
    peaks = angle_arr[peak_inds]

    # For diagnostic
    print(f'For {data_type}, looking at {peak_or_trough}s:')
    for i in range(len(peaks)):
        print(f"At index = {peak_inds[i]}, angle = {peaks[i]}")
    plt.plot(angle_arr)
    plt.plot(peak_inds, peaks, 'x')
    plt.show()

    return peaks, peak_inds


# Function for getting the average error between OMC values and IMU values
def get_peak_and_trough_errors(OMC_peaks, IMU_peaks):

    if len(OMC_peaks) == len(IMU_peaks):  # Check the same number of peaks has been found
        errors = abs(OMC_peaks - IMU_peaks)
    else:
        print("Number of OMC peaks/troughs found did not match IMU peaks/troughs found.")
        quit()

    mean_error = np.mean(errors)

    return mean_error


# Functions to define placement of max error annotation
def y_max_line_placement(max_error):
    if max_error > 40:
        line_placement = 40
    else:
        line_placement = max_error
    return line_placement

def y_max_text_placement(max_error, RMSE):
    if max_error > 40:
        text_placement = 40
    elif max_error < (RMSE*1.1):
        text_placement = RMSE*1.1
    else:
        text_placement = max_error
    return text_placement