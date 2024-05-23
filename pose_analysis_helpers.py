# All the functions needed to run pose_analysis.py

import ast
import opensim as osim

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