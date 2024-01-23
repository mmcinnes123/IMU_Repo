### Functions needed to read in the raw data_out, apply transformations, and write to APDM template file.
    # Raw data_out is in  MotionMonitor report file
    # APDM template file can be used to visualise quats in OpenSim
    # Transformations:
        # IMU data_out into Y-Up convention
        # LCF alginment - transform cluster data_out based on relative orientation to IMU at t=0, or average
from opensim import ArrayDouble

from scipy.spatial.transform import Rotation as R
import pandas as pd
from quat_functions import *
import opensim as osim


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
    rot_matrix = [[1, 0, 0], [0, 0, 1], [0, -1, 0]]
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
    new_df.to_csv(output_dir + r"\APDM_" + tag + ".csv", mode='w', index=False, header=False, encoding='utf-8', na_rep='nan')



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


def get_eulers_between_two_bodies(state, body1, body2, eul_seq):
    Rot = body2.findTransformBetween(state, body1).R()  # Finds rotation between two bodies
    quat = Rot.convertRotationToQuaternion()
    scipyR = R.from_quat([quat.get(1), quat.get(2), quat.get(3), quat.get(0)])
    # Get euler angles
    eul = scipyR.as_euler(eul_seq, degrees=True)

    return eul[0], eul[1], eul[2]

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




