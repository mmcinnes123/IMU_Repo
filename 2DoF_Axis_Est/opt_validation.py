

from helpers_2DoF import get_J1_J2_from_calibrated_OMC_model
from helpers_2DoF import get_J1_J2_from_opt
from helpers_2DoF import get_J1_J2_from_isolate_move
from helpers_2DoF import visulalise_3D_vec_on_IMU

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join
import logging
import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)

osim.Logger.setLevelString("Off")
logging.basicConfig(level=logging.INFO, filename="FE_axis.log", filemode="w")




"""" RUN FUNCTIONS ABOVE """

# Data to use for the optimisation
sample_rate = 100          # This is the sample rate of the data going into the function
trial_for_opt = 'JA_Slow'
IMU_type_for_opt = 'Perfect'
opt_method = 'rot'
time_dict = {
    'P1': {'JA_Slow': {'FE_start': 16, 'FE_end': 29, 'PS_start': 31, 'PS_end': 47}},
    'P2': {'JA_Slow': {'FE_start': 15, 'FE_end': 28, 'PS_start': 30, 'PS_end': 46}},
    'P3': {'JA_Slow': {'FE_start': 18, 'FE_end': 34, 'PS_start': 35, 'PS_end': 49}},
    'P4': {'JA_Slow': {'FE_start': 23, 'FE_end': 39, 'PS_start': 42, 'PS_end': 60}},
    'P5': {'JA_Slow': {'FE_start': 30, 'FE_end': 47, 'PS_start': 48, 'PS_end': 65}},
    'P6': {'JA_Slow': {'FE_start': 26, 'FE_end': 44, 'PS_start': 47, 'PS_end': 65}}
             }

# subject_list = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6']
subject_list = ['P1', 'P2']
# subject_list = []

logging.info(f'Using IMU type: {IMU_type_for_opt}, with data from trial: {trial_for_opt}.')

opt_rel2_OMC_errors = {}

for subject_code in subject_list:

    logging.info(f'Results for Subject {subject_code}')

    # Define some files
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_dir = join(parent_dir, 'OMC')
    model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')
    subject_time_dict = time_dict[subject_code]

    """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
    OMC_FE = get_J1_J2_from_calibrated_OMC_model(model_file, debug=False)
    logging.info(f'OMC FE axis in humerus cluster frame: {OMC_FE}')

    """ FINDING FE AND PS FROM OPTIMISATION RESULT """
    opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt,
                                                     opt_method, subject_time_dict, sample_rate, debug=False)
    heading_offset = opt_results['delta']*180/np.pi
    logging.info(f'Opt (rot) FE axis in humerus IMU frame: {opt_FE}')
    logging.info(f'Opt (rot) PS axis in forearm IMU frame: {opt_PS}')
    logging.info(f'Opt (rot) heading offset: {heading_offset}')
    # print('Cost: ', opt_results['debug']['cost'])
    # print('x: ', opt_results['debug']['x'])

    """ FINDING FE AND PS FROM ISOLATED JOINT MOVEMENT """
    # iso_FE, iso_PS = get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt,
    #                                              subject_time_dict, sample_rate, debug=False)
    # logging.info(f'Iso FE axis in humerus IMU frame: {iso_FE}')

    """ COMPARE """

    # Constrain the Opt FE axis to point in same direction as OMC reference
    if np.sign(opt_FE[2]) == np.sign(-OMC_FE[2]):
        opt_FE = -opt_FE

    opt_error = qmt.angleBetween2Vecs(OMC_FE, opt_FE) * 180/np.pi
    logging.info(f'Error between opt_FE and OMC_FE (deg): {opt_error}')

    # visulalise_3D_vec_on_IMU(opt_FE, OMC_FE, None)
    # visulalise_3D_vec_on_IMU(OMC_FE, opt_FE, iso_FE)

    opt_rel2_OMC_errors[subject_code] = opt_error


""" COMPILE ALL RESULTS """


all_errors = np.array(list(opt_rel2_OMC_errors.values()))
mean_all_errors = np.mean(all_errors)

logging.info(f'All errors: {opt_rel2_OMC_errors}.')
logging.info(f'Mean: {mean_all_errors}.')



