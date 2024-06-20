

from helpers_2DoF import get_J1_J2_from_calibrated_OMC_model
from helpers_2DoF import get_J1_J2_from_opt
from helpers_2DoF import get_J1_J2_from_isolate_move
from helpers_2DoF import visulalise_3D_vec_on_IMU

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join

import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)

osim.Logger.setLevelString("Off")





"""" RUN FUNCTIONS ABOVE """

# Data to use for the optimisation
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
sample_rate = 100          # This is the sample rate of the data going into the function


# subject_list = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6']
subject_list = ['P1']
# subject_list = []

for subject_code in subject_list:

    print('Results for Subject: ', subject_code)

    # Define some files
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_dir = join(parent_dir, 'OMC')
    model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')
    subject_time_dict = time_dict[subject_code]

    """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
    FE_in_clus = get_J1_J2_from_calibrated_OMC_model(model_file, debug=True)
    print('OMC reference FE axis in humerus cluster frame:', FE_in_clus)


    """ FINDING FE AND PS FROM OPTIMISATION RESULT """
    opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt,
                                                     opt_method, subject_time_dict, sample_rate, debug=False)
    print('FE axis estimation with rot method: ', opt_FE)
    # print('PS axis estimation with rot method: ', opt_PS)
    print('and heading offset: ', opt_results['delta']*180/np.pi)
    # print('Cost: ', opt_results['debug']['cost'])
    # print('x: ', opt_results['debug']['x'])


    """ FINDING FE AND PS FROM ISOLATED JOINT MOVEMENT """
    iso_FE, iso_PS = get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt,
                                                 subject_time_dict, sample_rate, debug=False)
    # print('FE axis estimation from isolated joint movement: ', iso_FE)


    """ COMPARE """

    # Note, optimisation based J1, J2 can point in either direction
    if np.sign(opt_FE[2]) == np.sign(-FE_in_clus[2]):
        opt_FE = -opt_FE

    opt_error = qmt.angleBetween2Vecs(FE_in_clus, opt_FE)
    print('Error between opt_FE and OMC_FE: ', opt_error*180/np.pi)

    # Compare results of optimisation with the isolated movement estimates
    # opt_iso_error = qmt.angleBetween2Vecs(iso_FE, opt_FE)
    # print('Error between opt_FE and iso_FE: ', opt_iso_error*180/np.pi)

    visulalise_3D_vec_on_IMU(opt_FE, FE_in_clus, None)
    # visulalise_3D_vec_on_IMU(FE_in_clus, opt_FE, iso_FE)




# # The optimal virtual IMU offset was calculated as:
# IMU_offset_hum = np.array([-0.03217144, 0.77309788, -0.02682034, 0.63290231])
# OMC_offset_hum = np.array([-0.05252603, 0.67153327, -0.05773951, 0.73685157])
#
# quat_change = qmt.qmult(qmt.qinv(IMU_offset_hum), OMC_offset_hum)  # Get orientation change from q0, for all t
# angles = qmt.quatAngle(quat_change, plot=False)  # Get the magnitude of the change from q0
# print('Angle difference between estimated virtual IMU offset and OMC virtual IMU offset is: ', np.rad2deg(angles))
#

