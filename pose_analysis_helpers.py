# All the functions needed to run pose_analysis.py

import ast

def get_trial_pose_time_dict_from_file(parent_dir, subject_code):
    # Read the existing txt file to get the dict
    file_obj = open(parent_dir + '\\' + subject_code + '_cal_pose_dict.txt', 'r')
    content = file_obj.read()
    trial_pose_time_dict = ast.literal_eval(content)
    file_obj.close()
    return trial_pose_time_dict