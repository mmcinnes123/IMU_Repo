import os.path

from helpers_2DoF import get_J1_J2_from_calibrated_OMC_model
from helpers_2DoF import get_J1_J2_from_opt
from helpers_2DoF import get_J1_J2_from_isolate_move
from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import get_event_dict_from_file
from joint_axis_est_2d import axisToThetaPhi


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

IMU_type_for_opt_list = ['Real', 'Perfect']
# IMU_type_for_opt_list = ['Perfect']

opt_method_list = ['rot', 'ori', 'rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'
# opt_method_list = ['rot_noDelta']   # Options: 'rot', 'ori', 'rot_noDelta'

trial_dict = {'JA_Slow': ['FE_start', 'PS_end'], 'ADL': ['kettle1_start', 'drink1_end']}
# trial_dict = {'JA_Slow': ['FE_start', 'PS_end']}


# List of subjects
subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]
# subject_list = [f'P{i}' for i in range(1, 24) if f'P{i}' not in ('P12', 'P21', 'P6', 'P7')]    # Missing FE/PS data
# subject_list = ['P23']    # Missing FE/PS data

# Initiate dict to store the calculated error for each subject
opt_rel2_OMC_errors = {}
all_data = pd.DataFrame()
alt_all_data = pd.DataFrame()


for trial_for_opt in trial_dict:
    event_to_start = trial_dict[trial_for_opt][0]
    event_to_end = trial_dict[trial_for_opt][1]

    for IMU_type_for_opt in IMU_type_for_opt_list:

        for opt_method in opt_method_list:

            logging.info(f'Using IMU type: {IMU_type_for_opt}, with data from trial: {trial_for_opt}, '
                         f'and optimisation method: {opt_method}.')

            for subject_code in subject_list:

                print(f'Running analysis for {subject_code}.')

                # Define some files
                parent_dir = join(directory, subject_code)
                OMC_dir = join(parent_dir, 'OMC')
                model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

                # Get the dict with the timings for FE and PS events
                subject_event_dict = get_event_dict_from_file(subject_code)

                """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
                OMC_FE, OMC_PS = get_J1_J2_from_calibrated_OMC_model(model_file, debug=False)

                """ FINDING FE AND PS FROM OPTIMISATION RESULT """
                opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt,
                                                                 opt_method, trial_for_opt, subject_event_dict,
                                                                 event_to_start, event_to_end,
                                                                 sample_rate, debug=False)
                #
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
                                            'IMUtype': [IMU_type_for_opt], 'OptMethod': [opt_method],
                                            'Metric': metric, 'Metric_value': metric_value
                                            })
                    # Log the results
                    all_data = pd.concat([all_data, new_row], ignore_index=True)

                alt_new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_for_opt],
                                            'IMUtype': [IMU_type_for_opt], 'OptMethod': [opt_method],
                                            'HeadingOffset': [heading_offset], 'AbsHeadingOffset': [abs_heading_offset],
                                            'SD_third_DoF': [SD_third_DoF],
                                            'FEOptError': [FE_opt_error], 'FEOptError_theta': [FE_theta_diff], 'FEOptError_phi': [FE_phi_diff],
                                            'PSOptError': [PS_opt_error], 'PSOptError_theta': [PS_theta_diff], 'PSOptError_phi': [PS_phi_diff]
                                            })

                alt_all_data = pd.concat([alt_all_data, alt_new_row], ignore_index=True)

                # Visualise 3D animation of the results
                # visulalise_opt_result_vec_on_IMU(opt_PS, OMC_PS, None)
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
                # print('x: ', opt_results['debug']['x'])

                print(f'FE Error: {FE_opt_error}')
                print(f'PS Error: {PS_opt_error}')
                print(f'Finished analysis for {subject_code}.')

                """ FINDING FE AND PS FROM ISOLATED JOINT MOVEMENT """
                # iso_FE, iso_PS = get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt,
                #                                              subject_time_dict, sample_rate, debug=False)
                # logging.info(f'Iso FE axis in humerus IMU frame: {iso_FE}')


""" COMPILE ALL RESULTS """


def update_file(file_path, new_data):

    existing_df = pd.read_csv(file_path)

    # Step 3: Update the existing DataFrame with values from the new DataFrame
    for idx, new_row in new_data.iterrows():
        # Define the condition for matching rows
        condition = (
                (existing_df['Subject'] == new_row['Subject']) &
                (existing_df['Trial'] == new_row['Trial']) &
                (existing_df['IMUtype'] == new_row['IMUtype']) &
                (existing_df['OptMethod'] == new_row['OptMethod']) &
                (existing_df['Metric'] == new_row['Metric'])
        )

        # Find the index of the row to update
        index_to_update = existing_df[condition].index

        # If the index exists, update the Metric_value
        if not index_to_update.empty:
            existing_df.loc[index_to_update, 'Metric_value'] = new_row['Metric_value']

    # Step 4: Write the updated DataFrame back to the CSV file
    existing_df.to_csv(file_path, index=False)


def update_alt_file(file_path, new_data):

    existing_df = pd.read_csv(file_path)

    # Step 3: Update the existing DataFrame with values from the new DataFrame
    for idx, new_row in new_data.iterrows():
        # Define the condition for matching rows
        condition = (
                (existing_df['Subject'] == new_row['Subject']) &
                (existing_df['Trial'] == new_row['Trial']) &
                (existing_df['IMUtype'] == new_row['IMUtype']) &
                (existing_df['OptMethod'] == new_row['OptMethod'])
        )

        # Find the index of the row to update
        index_to_update = existing_df[condition].index

        # If the index exists, update the Metric_value
        if not index_to_update.empty:
            existing_df.loc[index_to_update, 'HeadingOffset'] = new_row['HeadingOffset']
            existing_df.loc[index_to_update, 'AbsHeadingOffset'] = new_row['AbsHeadingOffset']
            existing_df.loc[index_to_update, 'SD_third_DoF'] = new_row['SD_third_DoF']
            existing_df.loc[index_to_update, 'FEOptError'] = new_row['FEOptError']
            existing_df.loc[index_to_update, 'FEOptError_theta'] = new_row['FEOptError_theta']
            existing_df.loc[index_to_update, 'FEOptError_phi'] = new_row['FEOptError_phi']
            existing_df.loc[index_to_update, 'PSOptError'] = new_row['PSOptError']
            existing_df.loc[index_to_update, 'PSOptError_theta'] = new_row['PSOptError_theta']
            existing_df.loc[index_to_update, 'PSOptError_phi'] = new_row['PSOptError_phi']

    # Step 4: Write the updated DataFrame back to the CSV file
    existing_df.to_csv(file_path, index=False)


# Print all results to csv/ update row-by-row if the file already exists
results_file_path = join(directory, 'R Analysis', 'R 2DoF Opt', 'OptResultsForR.csv')
alt_results_file_path = join(directory, 'R Analysis', 'R 2DoF Opt', 'Alt_OptResultsForR.csv')

if os.path.exists(results_file_path):
    update_file(results_file_path, all_data)
else:
    all_data.to_csv(results_file_path)

if os.path.exists(alt_results_file_path):
    update_alt_file(alt_results_file_path, alt_all_data)
else:
    alt_all_data.to_csv(alt_results_file_path)


