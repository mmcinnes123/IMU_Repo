

# import opensim as osim
# import numpy as np
# from os.path import join
# import pandas as pd
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as R
import os
import shutil


# subject_list = [f'P{i}' for i in range(1, 23) if f'P{i}' not in ('P12', 'P21')]    # Missing FE/PS data
#
# subject_list = ['P001', 'P002', 'P003', 'P004', 'P005', 'P006', 'P007', 'P008', 'P009', 'P010', 'P011', 'P012', 'P013',
#                 'P014', 'P015', 'P016', 'P017', 'P018', 'P019', 'P020']

subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

subject_code_remap = {'P1': 'P001',
                      'P2': 'P002',
                      'P3': 'P003',
                      'P4': 'P004',
                      'P5': 'P005',
                      'P6': 'PX6',
                      'P7': 'PX7',
                      'P8': 'P006',
                      'P9': 'P007',
                      'P10': 'P008',
                      'P11': 'P009',
                      'P12': 'PX12',
                      'P13': 'P010',
                      'P14': 'P011',
                      'P15': 'P012',
                      'P16': 'P013',
                      'P17': 'P014',
                      'P18': 'P015',
                      'P19': 'P016',
                      'P20': 'P017',
                      'P21': 'PX21',
                      'P22': 'P018',
                      'P23': 'P019',
                      'P24': 'P020'
                      }

dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'


for old_code, new_code in subject_code_remap.items():

    real_dir = os.path.join(dir, new_code, 'Real')

    alt_IK_results_dir = os.path.join(real_dir, 'Target_Pose_Abd13')

    folders_to_move = ['IMU_IK_results_METHOD_5', 'IMU_IK_results_METHOD_4a', 'IMU_IK_results_OSIM_Alt_asst', 'IMU_IK_results_OSIM_Alt_self', 'IMU_IK_results_OSIM_N_self']

    for folder in folders_to_move:

        folder_path = os.path.join(real_dir, folder)

        if os.path.exists(folder_path):

            # Move it to the alt results folder
            shutil.move(folder_path, alt_IK_results_dir)

        else:
            print(f'{folder} does not exist for subject {new_code}')
