

from helpers_2DoF import get_J1_J2_from_calibrated_OMC_model
from helpers_2DoF import get_J1_J2_from_opt
from helpers_2DoF import get_J1_J2_from_isolate_move
from helpers_2DoF import visulalise_3D_vec_on_IMU
from helpers_2DoF import get_event_dict_from_file

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join
import logging
import numpy as np
import pandas as pd
from tkinter.filedialog import askopenfilename, askdirectory

np.set_printoptions(suppress=True)
osim.Logger.setLevelString("Off")
logging.basicConfig(level=logging.INFO, filename="FE_axis.log", filemode="w")


"""" RUN FUNCTIONS ABOVE """

# Data to use for the optimisation
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
sample_rate = 100          # This is the sample rate of the data going into the function
trial_for_opt = 'JA_Slow'
IMU_type_for_opt_list = ['Real', 'Perfect']
opt_method_list = ['rot', 'ori', 'rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'

# IMU_type_for_opt_list = ['Perfect']
# opt_method_list = ['rot_noDelta']


# List of subjects
subject_list = [f'P{i}' for i in range(1, 23)]

# Initiate dict to store the calculated error for each subject
opt_rel2_OMC_errors = {}
all_data = pd.DataFrame()

for IMU_type_for_opt in IMU_type_for_opt_list:

    for opt_method in opt_method_list:

        logging.info(f'Using IMU type: {IMU_type_for_opt}, with data from trial: {trial_for_opt}, '
                     f'and optimisation method: {opt_method}.')

        for subject_code in subject_list:

            logging.info(f'Results for Subject {subject_code}')
            print(f'Running analysis for {subject_code}.')

            # Define some files
            parent_dir = join(directory, subject_code)
            OMC_dir = join(parent_dir, 'OMC')
            model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

            # Get the dict with the timings for FE and PS events
            subject_event_dict = get_event_dict_from_file(subject_code)

            """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
            OMC_FE, OMC_PS = get_J1_J2_from_calibrated_OMC_model(model_file, debug=False)
            logging.info(f'OMC FE axis in humerus cluster frame: {OMC_FE}')
            logging.info(f'OMC PS axis in forearm cluster frame: {OMC_PS}')

            """ FINDING FE AND PS FROM OPTIMISATION RESULT """
            opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt,
                                                             opt_method, subject_event_dict, sample_rate, debug=False)
            logging.info(f'Opt FE axis in humerus IMU frame: {opt_FE}')
            logging.info(f'Opt PS axis in forearm IMU frame: {opt_PS}')

            # Log optional outputs
            if 'delta' in opt_results:
                heading_offset = abs(opt_results['delta']*180/np.pi)
                logging.info(f'Opt heading offset: {heading_offset}')
            else:
                heading_offset = None
            if 'SD_third_DoF' in opt_results['debug']:
                SD_third_DoF = opt_results['debug']['SD_third_DoF']
                logging.info(f'Opt SD in third DoF: {SD_third_DoF}')
            else:
                SD_third_DoF = None

            # print('Cost: ', opt_results['debug']['cost'])
            # print('x: ', opt_results['debug']['x'])



            """ COMPARE """

            # Constrain the Opt FE axis to point in same direction as OMC reference
            if np.sign(opt_FE[2]) == np.sign(-OMC_FE[2]):   # Constrain based on the z-component, expected to be largest
                opt_FE = -opt_FE
            if np.sign(opt_PS[1]) == np.sign(-OMC_PS[1]):   # Constrain based on the y-component, expected to be largest
                opt_PS = -opt_PS

            FE_opt_error = qmt.angleBetween2Vecs(OMC_FE, opt_FE) * 180 / np.pi
            PS_opt_error = qmt.angleBetween2Vecs(OMC_PS, opt_PS) * 180 / np.pi
            logging.info(f'Error between opt_FE and OMC_FE (deg): {FE_opt_error}')
            logging.info(f'Error between opt_PS and OMC_PS (deg): {PS_opt_error}')

            # Log the results
            new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_for_opt],
                                    'IMUtype': [IMU_type_for_opt], 'OptMethod': [opt_method],
                                    'HeadingOffset': [heading_offset], 'FEOptError': [FE_opt_error],
                                    'PSOptError': [PS_opt_error], 'SD_third_DoF': [SD_third_DoF]})
            all_data = pd.concat([all_data, new_row], ignore_index=True)

            print(f'FE Error: {FE_opt_error}')
            print(f'PS Error: {PS_opt_error}')
            print(f'Finished analysis for {subject_code}.')

            # Visualise 3D animation of the results
            # visulalise_3D_vec_on_IMU(opt_PS, OMC_PS, None)
            # visulalise_3D_vec_on_IMU(OMC_FE, opt_FE, iso_FE)

            """ FINDING FE AND PS FROM ISOLATED JOINT MOVEMENT """
            # iso_FE, iso_PS = get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt,
            #                                              subject_time_dict, sample_rate, debug=False)
            # logging.info(f'Iso FE axis in humerus IMU frame: {iso_FE}')

""" COMPILE ALL RESULTS """

# Print all results to csv
all_data.to_csv(join(directory, 'R Analysis', 'R 2DoF Opt', 'OptResultsForR.csv'))


