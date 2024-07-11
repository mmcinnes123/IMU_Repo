

import opensim as osim
import numpy as np
from os.path import join
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os

# Check list of mot filse to see if weird singularity happened during IK which made GH coords be over 180deg


list_of_subjects = [f'P{i}' for i in range(1, 23) if f'P{i}' not in ('P12', 'P21')]    # Missing FE/PS data
# list_of_subjects = ['P15']    # Missing FE/PS data
calibration_name_dict = {'OSIM_Alt_self': None, 'OSIM_Alt_asst': None, 'METHOD_4a': None, 'METHOD_4b': None, 'METHOD_5': None}
# calibration_name_dict = {'METHOD_5': None}
IMU_type_list = ['Perfect', 'Real']
# IMU_type_list = ['Perfect']
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
trial_name = 'JA_Slow'

coord_limit = 180

# Get the results for each calibration method
for subject_code in list_of_subjects:

    for IMU_type in IMU_type_list:

        for calibration_name in calibration_name_dict:

            # Define the file paths
            IK_results_dir = join(directory, subject_code, IMU_type, 'IMU_IK_results_' + calibration_name, trial_name)
            IK_results_mot = join(IK_results_dir, 'IMU_IK_results.mot')

            OMC_results_dir = join(directory, subject_code, 'OMC', trial_name + '_IK_Results')
            OMC_results_mot = join(OMC_results_dir, 'OMC_IK_results.mot')

            # Read RMSEs from the CSV file
            if os.path.exists(IK_results_mot):

                # print('Reading coordinates from .mot files...')
                coords_table = osim.TimeSeriesTable(IK_results_mot)
                all_GH_coords = []
                for coord in ['GH_y', 'GH_z', 'GH_x']:
                    all_GH_coords.append(coords_table.getDependentColumn(coord).to_numpy())
                all_GH_coords = np.array(all_GH_coords)

                if np.any((all_GH_coords > coord_limit) | (all_GH_coords < -coord_limit)):
                    print(f'Values above {coord_limit} for file: {IK_results_mot}')




