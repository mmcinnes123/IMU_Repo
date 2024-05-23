# All the functions needed to run pose_analysis.py

from helpers_compare import get_vec_angles_from_two_CFs

import ast
import opensim as osim
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_trial_pose_time_dict_from_file(parent_dir, subject_code):
    # Read the existing txt file to get the dict
    file_obj = open(parent_dir + '\\' + subject_code + '_cal_pose_dict.txt', 'r')
    content = file_obj.read()
    trial_pose_time_dict = ast.literal_eval(content)
    file_obj.close()
    return trial_pose_time_dict


def get_coords_from_mot(mot_file):

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    coords_table = osim.TimeSeriesTable(mot_file)

    return coords_table

def get_coord_at_time_t(coords_table, pose_time, key):

    index = coords_table.getNearestRowIndexForTime(pose_time, restrictToTimeRange=True)
    coord_column = coords_table.getDependentColumn(key)
    coord = coord_column.row(index).to_numpy()

    return coord


def get_HT_JAs():
    thorax_IMU, humerus_IMU, radius_IMU = get_body_quats_from_analysis_sto(IMU_analysis_sto_path, start_time, end_time)


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

    print('Thorax quat:', thorax_quats)
    print('Humerus quat:', humerus_quats)
    return thorax_quats, humerus_quats, radius_quats




def get_HT_angles_from_sto(analysis_sto_file, pose_time):

    # Read the sto file and get the orientation of each body at pose_time (in quats)
    thorax_quats, humerus_quats, radius_quats = get_body_quats_from_analysis_sto(analysis_sto_file, pose_time)

    abduction, flexion, rotation_elbow_down, rotation_elbow_up = get_vec_angles_from_two_CFs(thorax_quats, humerus_quats)

    return abduction, flexion, rotation_elbow_down