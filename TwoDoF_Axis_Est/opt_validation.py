import os.path

from helpers_2DoF import get_J1_J2_from_calibrated_OMC_model
from helpers_2DoF import get_J1_J2_from_opt
from helpers_2DoF import get_model_FE_in_hum
from helpers_2DoF import plot_FE_estimates
from helpers_2DoF import plot_PS_estimates
from helpers_2DoF import get_J1_J2_from_isolate_move
from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import get_event_dict_from_file
from joint_axis_est_2d import axisToThetaPhi


import qmt
import opensim as osim
from scipy.spatial.transform import Rotation as R
from os.path import join
import logging
import numpy as np
import pandas as pd
import plotly.graph_objects as go

np.set_printoptions(suppress=True)
osim.Logger.setLevelString("Off")
logging.basicConfig(level=logging.INFO, filename="FE_axis.log", filemode="w")


"""" RUN FUNCTIONS ABOVE """

# Data to use for the optimisation
directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
sample_rate = 100          # This is the sample rate of the data going into the function




def run_opt_validation(subject_list, IMU_type_for_opt_list, opt_method_list, JA_Slow_period_dict, ADL_period_dict):

    # Initiate dict to store the calculated error for each subject_code
    all_data = pd.DataFrame()
    alt_all_data = pd.DataFrame()
    vec_in_model_frames_data = pd.DataFrame()
    trial_dict_dict = {'JA_Slow': JA_Slow_period_dict, 'ADL': ADL_period_dict}

    for trial_for_opt in trial_dict_dict:

        # Get either the JA_Slow or ADL data period dict
        data_period_dict = trial_dict_dict[trial_for_opt]

        for data_period in data_period_dict:

            for IMU_type_for_opt in IMU_type_for_opt_list:

                for opt_method in opt_method_list:

                    logging.info(f'Using IMU type: {IMU_type_for_opt}, with data from trial: {trial_for_opt}, '
                                 f'and optimisation method: {opt_method}.')

                    for subject_code in subject_list:

                        # Get the names of the events which define the start and end time of the period of interest
                        # Correct event names for P008 who performed drink first, then kettle
                        if subject_code == 'P008' and trial_for_opt == 'ADL' and data_period == 'ADL_both':
                            event_to_start = 'drink1_start'
                            event_to_end = 'kettle1_end'
                        else:
                            event_to_start = data_period_dict[data_period][0]
                            event_to_end = data_period_dict[data_period][1]

                        # Get the dict with the timings for FE and PS events
                        subject_event_dict = get_event_dict_from_file(subject_code)

                        # Get the start and end time for which to run the optimisation
                        start_time = float(subject_event_dict[trial_for_opt][event_to_start])
                        end_time = float(subject_event_dict[trial_for_opt][event_to_end])
                        time_period = end_time - start_time

                        # Define some files
                        parent_dir = join(directory, subject_code)
                        OMC_dir = join(parent_dir, 'OMC')
                        model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

                        print(f'Running {opt_method} analysis for {subject_code}, IMU type {IMU_type_for_opt}'
                              f'\nfrom event {event_to_start} (time {start_time}) to event {event_to_end} (time {end_time}).')

                        """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
                        OMC_FE, OMC_PS, hum_clus_in_hum, rad_clus_in_rad = get_J1_J2_from_calibrated_OMC_model(model_file, debug=False)

                        """ FINDING FE AND PS FROM OPTIMISATION RESULT """
                        opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt,
                                                                         opt_method, trial_for_opt, subject_event_dict,
                                                                         event_to_start, event_to_end,
                                                                         sample_rate, debug=False)

                        # Get FE/PS estimates in body frames for visualisation
                        opt_FE_in_hum = qmt.rotate(qmt.qinv(hum_clus_in_hum), opt_FE, debug=False, plot=False)
                        opt_PS_in_rad = qmt.rotate(qmt.qinv(rad_clus_in_rad), opt_PS, debug=False, plot=False)
                        new_vec_row = pd.DataFrame({'opt_FE_in_hum': [opt_FE_in_hum], 'opt_PS_in_rad': [opt_PS_in_rad]})
                        vec_in_model_frames_data = pd.concat([vec_in_model_frames_data, new_vec_row], ignore_index=True)

                        # Log optional outputs
                        if 'delta' in opt_results:
                            heading_offset = opt_results['delta']*180/np.pi
                            abs_heading_offset = abs(heading_offset)
                        else:
                            heading_offset = None
                            abs_heading_offset = None
                        if 'SD_third_DoF' in opt_results['debug']:
                            SD_third_DoF = opt_results['debug']['SD_third_DoF']
                        else:
                            SD_third_DoF = None

                        """ COMPARE """

                        # Choose the direction of the PS axis based on the direction of the OMC result
                        if np.sign(opt_PS[1]) == np.sign(-OMC_PS[1]):  # Constrain based on the y-component, expected to be largest
                            opt_PS = -opt_PS

                        # Choose the direction of the FE axis based on the direction of the OMC result
                        if 'delta' not in opt_results:
                            if np.sign(opt_FE[2]) == np.sign(-OMC_FE[2]):   # Constrain based on the z-component, expected to be largest
                                opt_FE = -opt_FE
                        else:
                            if abs(heading_offset) < 45:
                                if np.sign(opt_FE[2]) == np.sign(-OMC_FE[2]):  # Constrain to half space in positive z direction based on opt result
                                    opt_FE = -opt_FE

                            # For more extreme cases where the heading offset is large
                            else:
                                # If the axis points in positive x direction and the heading offset is positive
                                if np.sign(opt_FE[0]) == np.sign(heading_offset):
                                    opt_FE = -opt_FE    # Change the direction of the FE axis estimate

                        # Find the single angle difference between he OMC axis estimates and the optimisation estiamtes
                        FE_opt_error = qmt.angleBetween2Vecs(OMC_FE, opt_FE) * 180 / np.pi
                        PS_opt_error = qmt.angleBetween2Vecs(OMC_PS, opt_PS) * 180 / np.pi

                        # Investigate theta phi variation
                        opt_FE_theta, opt_FE_phi = axisToThetaPhi(opt_FE, var=3)
                        OMC_FE_theta, OMC_FE_phi = axisToThetaPhi(OMC_FE, var=3)
                        FE_theta_diff = np.rad2deg(OMC_FE_theta - opt_FE_theta)
                        FE_phi_diff = np.rad2deg(OMC_FE_phi - opt_FE_phi)

                        opt_PS_theta, opt_PS_phi = axisToThetaPhi(opt_PS, var=3)
                        OMC_PS_theta, OMC_PS_phi = axisToThetaPhi(OMC_PS, var=3)
                        PS_theta_diff = np.rad2deg(abs(OMC_PS_theta - opt_PS_theta))
                        PS_phi_diff = np.rad2deg(abs(OMC_PS_phi - opt_PS_phi))

                        metric_dict = {'HeadingOffset': [heading_offset], 'AbsHeadingOffset': [abs_heading_offset], 'SD_third_DoF': [SD_third_DoF],
                                       'FEOptError': [FE_opt_error], 'FEOptError_theta': [FE_theta_diff], 'FEOptError_phi': [FE_phi_diff],
                                       'PSOptError': [PS_opt_error], 'PSOptError_theta': [PS_theta_diff], 'PSOptError_phi': [PS_phi_diff]}

                        for metric, metric_value in metric_dict.items():

                            new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_for_opt],
                                                    'DataPeriod': [data_period], 'TimePeriod': [time_period],
                                                    'IMUtype': [IMU_type_for_opt], 'OptMethod': [opt_method],
                                                    'Metric': metric, 'Metric_value': metric_value
                                                    })
                            # Log the results
                            all_data = pd.concat([all_data, new_row], ignore_index=True)

                        alt_new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_for_opt], 'DataPeriod': [data_period],
                                                    'IMUtype': [IMU_type_for_opt], 'OptMethod': [opt_method],
                                                    'HeadingOffset': [heading_offset], 'AbsHeadingOffset': [abs_heading_offset],
                                                    'SD_third_DoF': [SD_third_DoF],
                                                    'FEOptError': [FE_opt_error], 'FEOptError_theta': [FE_theta_diff], 'FEOptError_phi': [FE_phi_diff],
                                                    'PSOptError': [PS_opt_error], 'PSOptError_theta': [PS_theta_diff], 'PSOptError_phi': [PS_phi_diff]
                                                    })

                        alt_all_data = pd.concat([alt_all_data, alt_new_row], ignore_index=True)

                        # Visualise 3D animation of the results
                        # visulalise_opt_result_vec_on_IMU(OMC_PS, opt_PS, None)
                        # visulalise_opt_result_vec_on_IMU(OMC_FE, opt_FE, None)

                        # Log results
                        logging.info(f'Results for Subject {subject_code}')
                        logging.info(f'OMC FE axis in humerus cluster frame: {OMC_FE}')
                        logging.info(f'OMC PS axis in forearm cluster frame: {OMC_PS}')
                        if 'delta' in opt_results:
                            logging.info(f'Opt heading offset (deg): {heading_offset}')
                        logging.info(f'Opt SD in third DoF (deg): {SD_third_DoF}')
                        logging.info(f'Opt FE axis in humerus IMU frame: {opt_FE}')
                        logging.info(f'Opt PS axis in forearm IMU frame: {opt_PS}')
                        logging.info(f'Error between opt_FE and OMC_FE (deg): {FE_opt_error}')
                        logging.info(f'Error between opt_PS and OMC_PS (deg): {PS_opt_error}')

                        # Print results
                        if 'delta' in opt_results:
                            print('Heading offset (rad): ', opt_results['delta'])
                            print('Heading offset (deg): ', opt_results['delta'] * 180 / np.pi)
                        print('Cost: ', opt_results['debug']['cost'])
                        print('x: ', opt_results['debug']['x'])

                        print(f'FE Error: {FE_opt_error}')
                        print(f'PS Error: {PS_opt_error}')
                        print(f'Finished analysis for {subject_code}.')

                        """ FINDING FE AND PS FROM ISOLATED JOINT MOVEMENT """
                        # iso_FE, iso_PS = get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt,
                        #                                              subject_time_dict, sample_rate, debug=False)
                        # logging.info(f'Iso FE axis in humerus IMU frame: {iso_FE}')

    return all_data, alt_all_data, vec_in_model_frames_data


""" RUN ALL VALIDATION AND SAVE RESULTS TO CSV """
#
# subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
# IMU_type_for_opt_list = ['Real', 'Perfect']
# opt_method_list = ['rot', 'ori', 'rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'
# # Define a list of different periods of data to use
# JA_Slow_period_dict = {'ISO_5reps': ['FE_start', 'PS_end'],
#                        # 'ISO_4reps': ['FE2_start', 'PS5_start'],
#                        # 'ISO_3reps': ['FE3_start', 'PS4_start'],
#                        # 'ISO_2reps': ['FE4_start', 'PS3_start'],
#                        'ISO_1rep': ['FE5_start', 'PS2_start']}
# ADL_period_dict = {'ADL_both': ['kettle1_start', 'drink1_end'],
#                    'ADL_kettle': ['kettle1_start', 'kettle1_end'],
#                    'ADL_drink': ['drink1_start', 'drink1_end']}
#
# all_data, alt_all_data, vec_in_model_frames_data = run_opt_validation(subject_list, IMU_type_for_opt_list, opt_method_list, JA_Slow_period_dict, ADL_period_dict)
#
# # Print all results to csv/ update row-by-row if the file already exists
# results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\Results'
# results_file_path = join(results_dir, 'Opt_Results.csv')
# alt_results_file_path = join(results_dir, 'Alt_Opt_Results.csv')
#
# all_data.to_csv(results_file_path)
# alt_all_data.to_csv(alt_results_file_path)

""" PLOT VARIATION """

# # subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
# subject_list = ['P019']
# IMU_type_for_opt_list = ['Perfect']
# opt_method_list = ['rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'
# JA_Slow_period_dict = {'ISO_1rep': ['FE5_start', 'PS2_start']}
# ADL_period_dict = {}
#
# all_data, alt_all_data, vec_in_model_frames_data = run_opt_validation(subject_list, IMU_type_for_opt_list, opt_method_list, JA_Slow_period_dict, ADL_period_dict)
#
#
# def plot_variation_in_opt_estimates(vec_data):
#
#     # Get FE and PS in model frames
#     FE_axis_in_humerus = get_model_FE_in_hum()
#     PS_axis_in_radius = np.array([0.182, 0.98227, -0.044946])
#
#     all_FE_axes_est = vec_data['opt_FE_in_hum'].to_numpy()
#     all_PS_axes_est = vec_data['opt_PS_in_rad'].to_numpy()
#
    # plot_FE_estimates(all_FE_axes_est, FE_axis_in_humerus)
#     plot_PS_estimates(all_PS_axes_est, PS_axis_in_radius)


# plot_variation_in_opt_estimates(vec_in_model_frames_data)


""" RUN INVESTIGATION """

subject_list = ['P005']
IMU_type_for_opt_list = ['Perfect']
opt_method_list = ['rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'
# Define a list of different periods of data to use
JA_Slow_period_dict = {}
ADL_period_dict = {'ADL_both': ['kettle1_start', 'drink1_end']}

all_data, alt_all_data, vec_in_model_frames_data = run_opt_validation(subject_list, IMU_type_for_opt_list, opt_method_list, JA_Slow_period_dict, ADL_period_dict)


