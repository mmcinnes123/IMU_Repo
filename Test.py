

import opensim as osim
import numpy as np
from os.path import join
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os
import shutil


# Check list of mot filse to see if weird singularity happened during IK which made GH coords be over 180deg

dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'

subject_code_list = [f'P{i}' for i in range(2, 23) if f'P{i}' not in ('P12', 'P21')]

move_to_folder = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\Old Range Dicts'

for subject_code in subject_code_list:
    subject_dir = join(dir, subject_code)
    range_dict_file_name = subject_code + '_JA_range_dict.txt'
    range_dict_file = join(subject_dir, range_dict_file_name)

    file_obj = open(range_dict_file, 'r')
    range_dict_str = file_obj.read()
    file_obj.close()
    range_dict = eval(range_dict_str)

    for joint_name in range_dict.keys():
        ind_val_1 = range_dict[joint_name][0]
        ind_val_2 = range_dict[joint_name][1]
        # Check the values are large, not time values
        if ind_val_1 > 200:
            ind_val_1_new = ind_val_1 / 100
            ind_val_2_new = ind_val_2 / 100

            new_list = [ind_val_1_new, ind_val_2_new]

            # Update the range_dict
            range_dict[joint_name] = new_list

            # Save dict to .txt
            file_obj = open(range_dict_file, 'w')
            file_obj.write(str(range_dict))
            file_obj.close()

        else:
            print('This range dict was already time values: ', range_dict_file)


