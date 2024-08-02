# All the functions needed to run pose_analysis.py
import os.path

from helpers_compare import get_vec_angles_from_two_CFs

import ast
import opensim as osim
import numpy as np
from scipy.spatial.transform import Rotation as R
import pandas as pd
from os.path import join


# Function to open the .txt file which contains a dict with info on times, and pose names, for each trial
def get_trial_pose_time_dict_from_file(directory, subject_code):
    # Read the existing txt file to get the dict
    file_obj = open(join(directory, 'SubjectEventFiles', subject_code + '_event_dict.txt'), 'r')
    content = file_obj.read()
    trial_pose_time_dict = ast.literal_eval(content)
    file_obj.close()
    return trial_pose_time_dict


# Function to read an .mot file and get the relevant coordinates (joint angles) at a specific time
def get_coords_from_mot_file(mot_file, pose_time):

    assert os.path.exists(mot_file), f'{mot_file} does not exist.'

    # Read in coordinates from IK results .mot files
    print(f'Reading coordinates from {mot_file} file...')
    coords_table = osim.TimeSeriesTable(mot_file)

    # Get the coords (elbow flexion, pronation) at the specified pose time
    elbow_flexion = get_coord_at_time_t(coords_table, pose_time, key='EL_x')
    elbow_pronation = get_coord_at_time_t(coords_table, pose_time, key='PS_y')

    return elbow_flexion, elbow_pronation


# Function to get the coordinate at a specific time in the coords table
def get_coord_at_time_t(coords_table, pose_time, key):

    index = coords_table.getNearestRowIndexForTime(pose_time, restrictToTimeRange=True)
    coord_column = coords_table.getDependentColumn(key)
    coord = coord_column.row(index).to_numpy()

    return coord


# Function to get the humero-thoracic joint angles from an .sto file, using the model body oris to calculate projected vectors
def get_HT_angles_from_sto(analysis_sto_file, pose_time):

    assert os.path.exists(analysis_sto_file), f'{analysis_sto_file} does not exist.'

    # Read the sto file and get the orientation of each body at pose_time (in quats)
    thorax_quats, humerus_quats, radius_quats = get_body_quats_from_analysis_sto(analysis_sto_file, pose_time)

    # Get the projected vector angles from the body orientation data
    abduction, flexion, rotation_elbow_down, rotation_elbow_up = get_vec_angles_from_two_CFs(thorax_quats, humerus_quats)

    return abduction, flexion, rotation_elbow_down


# Function to get the orientation (in quaternions) of each model body from the sto file
def get_body_quats_from_analysis_sto(analysis_sto_path, pose_time):

    # Read in the analysis .sto file with pos and ori data for each model body
    analysis_table = osim.TimeSeriesTable(analysis_sto_path)   # Read in new states

    thorax_eulers = get_body_ori_from_table(analysis_table, pose_time, key1='thorax_Ox', key2='thorax_Oy', key3='thorax_Oz')
    humerus_eulers = get_body_ori_from_table(analysis_table, pose_time, key1='humerus_r_Ox', key2='humerus_r_Oy', key3='humerus_r_Oz')
    radius_eulers = get_body_ori_from_table(analysis_table, pose_time, key1='radius_r_Ox', key2='radius_r_Oy', key3='radius_r_Oz')

    # Create an array of scipy Rotations
    thorax_R = R.from_euler('XYZ', thorax_eulers, degrees=True)
    humerus_R = R.from_euler('XYZ', humerus_eulers, degrees=True)
    radius_R = R.from_euler('XYZ', radius_eulers, degrees=True)

    thorax_quats = thorax_R.as_quat()[:,[1, 2, 3, 0]]
    humerus_quats = humerus_R.as_quat()[:,[1, 2, 3, 0]]
    radius_quats = radius_R.as_quat()[:,[1, 2, 3, 0]]

    return thorax_quats, humerus_quats, radius_quats


# Function to get the body orientation from the data table, based on a given time
def get_body_ori_from_table(table, pose_time, key1, key2, key3):

    # Get the body ori at pose_time
    index = table.getNearestRowIndexForTime(pose_time, restrictToTimeRange=True)
    ori1_column = table.getDependentColumn(key1)
    ori1 = ori1_column.row(index).to_numpy()
    ori2_column = table.getDependentColumn(key2)
    ori2 = ori2_column.row(index).to_numpy()
    ori3_column = table.getDependentColumn(key3)
    ori3 = ori3_column.row(index).to_numpy()
    eulers = np.stack((ori1, ori2, ori3), axis=1)

    return eulers


# Function to get the JAs of the default model pose, based on the pose name
def get_default_model_pose_dict(pose_name):

    if pose_name in ['N_self', 'N_asst']:
        model_pose_dict = {'elbow_flexion': np.array([0]),
                           'elbow_pronation': np.array([90]),
                           'HT_abd': np.array([0]),
                           'HT_flex': np.array([0]),
                           'HT_rot': np.array([0])}

    elif pose_name in ['Alt_self', 'Alt_asst']:
        model_pose_dict = {'elbow_flexion': np.array([90]),
                           'elbow_pronation': np.array([90]),
                           'HT_abd': np.array([0]),
                           'HT_flex': np.array([0]),
                           'HT_rot': np.array([0])}

    elif pose_name == 'Alt2_self':
        model_pose_dict = {'elbow_flexion': np.array([90]),
                           'elbow_pronation': np.array([90]),
                           'HT_abd': np.array([0]),
                           'HT_flex': np.array([90]),
                           'HT_rot': np.array([0])}
    else:
        model_pose_dict = None
        print('Pose name does not match one listed here.')

    return model_pose_dict



def split_pose_name(pose_name):

    try:
        # Split the string and unpack into two variables
        pose_code, pose_type = pose_name.split('_')

    except ValueError:
        print("The string does not contain exactly one underscore or is malformed.")
        pose_code, pose_type = None, None

    return pose_code, pose_type




