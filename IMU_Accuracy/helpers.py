# Script containing functions to run IMU Accuracy

from functions import write_to_APDM
from functions import APDM_2_sto_Converter
from functions import find_RMSE_of_error_array

import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd
from scipy.stats.stats import pearsonr
import matplotlib.pyplot as plt




def run_IMU_Accuracy(subject_code, start_time, end_time, time_elevation_starts, trial_name):

    """ SETTINGS """

    trim_bool = True
    sample_rate = 100

    # Define some file paths
    directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
    parent_dir = os.path.join(directory, subject_code)
    raw_data_dir = os.path.join(parent_dir, 'RawData')
    IMU_type_dict = {'Real': ' - Report2 - IMU_Quats.txt', 'Perfect': ' - Report3 - Cluster_Quats.txt'}
    IMU_input_file = subject_code + '_' + trial_name + IMU_type_dict['Real']
    OMC_input_file = subject_code + '_' + trial_name + IMU_type_dict['Perfect']
    IMU_input_file_path = os.path.join(raw_data_dir, IMU_input_file)
    OMC_input_file_path = os.path.join(raw_data_dir, OMC_input_file)
    accuracy_results_dir = os.path.join(parent_dir, 'IMU_Accuracy_Results')
    if os.path.exists(accuracy_results_dir) == False:
        os.mkdir(accuracy_results_dir)
    trial_results_dir = os.path.join(accuracy_results_dir, trial_name)
    if os.path.exists(trial_results_dir) == False:
        os.mkdir(trial_results_dir)
    # Settings for writing to csv/sto
    APDM_template_file = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo', "APDM_template_4S.csv")
    APDM_settings_file = os.path.join(r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo', "APDMDataConverter_Settings.xml")

    # TODO: Include local misalignments specific to each IMU and each trial
    # Apply no local alingment for Participants where no alignment data is available
    IMU1_local_misalignment = R.from_quat([0, 0, 0, 1])
    IMU2_local_misalignment = R.from_quat([0, 0, 0, 1])
    IMU3_local_misalignment = R.from_quat([0, 0, 0, 1])


    """ GET RELATIVE ORIENTATIONS """

    # Read in IMU/Cluster orientation data from trial of interest
    IMU1, IMU2, IMU3 = get_scipyR_from_txt_file(IMU_input_file_path, trim_bool, start_time, end_time, sample_rate)
    OMC1, OMC2, OMC3 = get_scipyR_from_txt_file(OMC_input_file_path, trim_bool, start_time, end_time, sample_rate)

    # Apply local misalignment rotations to the cluster CFs to better align them with the IMU frames
    OMC1 = IMU1_local_misalignment * OMC1
    OMC2 = IMU2_local_misalignment * OMC2
    OMC3 = IMU3_local_misalignment * OMC3

    # Get the relative orientation between IMUs/Clusters
    IMU_joint12 = IMU1.inv() * IMU2
    IMU_joint23 = IMU2.inv() * IMU3
    IMU_joint13 = IMU1.inv() * IMU3
    OMC_joint12 = OMC1.inv() * OMC2
    OMC_joint23 = OMC2.inv() * OMC3
    OMC_joint13 = OMC1.inv() * OMC3

    """ WRITE ROTATED ORIENTATIONS TO STO FOR VISUALISATION IN OPENSIM """

    # For visualisation, find heading offset between thorax frames and apply to all IMUs
    heading_rotation = get_heading_rotation_using_2D_vecs(IMU1, OMC1)
    IMU1_globally_aligned = heading_rotation * IMU1
    IMU2_globally_aligned = heading_rotation * IMU2
    IMU3_globally_aligned = heading_rotation * IMU3

    # Write to .sto file for visualisation
    write_rot_quats_to_sto(IMU1_globally_aligned, IMU2_globally_aligned, IMU3_globally_aligned,
                           trial_name, APDM_template_file, APDM_settings_file, trial_results_dir, IMU_type='Real')
    write_rot_quats_to_sto(OMC1, OMC2, OMC3,
                           trial_name, APDM_template_file, APDM_settings_file, trial_results_dir, IMU_type='Perfect')

    """ ERROR METRICS """

    # Get the single-angle geodesic distance between Joint_R IMU and Joint_R OMC
    # Rotational difference between IMU and OMC joint rotation:
    joint12_rot_diff = IMU_joint12.inv() * OMC_joint12
    joint23_rot_diff = IMU_joint23.inv() * OMC_joint23
    joint13_rot_diff = IMU_joint13.inv() * OMC_joint13
    # Magnitude of that rotational difference:
    joint12_dist_error = joint12_rot_diff.magnitude() * 180 / np.pi
    joint23_dist_error = joint23_rot_diff.magnitude() * 180 / np.pi
    joint13_dist_error = joint13_rot_diff.magnitude() * 180 / np.pi
    # Plot the time-series distance error and get the RMSEs
    joint12_dist_error_RMSE, joint23_dist_error_RMSE, joint13_dist_error_RMSE = \
        plot_single_angle_diff(joint12_dist_error, joint23_dist_error, joint13_dist_error, start_time, end_time,
                               trial_results_dir)

    # Get the projected vector angles in the transverse, frontal, and sagittal planes, and compare IMU with OMC
    """
    For joint12 (thorax-humerus), projected vectors are defined as follows:
        Transverse plane: z_axis relative to Z on the XZ plane
        Frontal plane: y_axis relative to Y on the XY plane
        Sagittal plane: y_axis relative to Z on the YZ plane

    For joint23 (humerus-forearm), projected vectors are defined as follows:
        Transverse plane: y_axis relative to Z on the XZ plane
        Frontal plane: z_axis relative to Y on the ZY plane
        Sagittal plane: y_axis relative to Y on the XY plane

    For joint13 (thorax-forearm), projected vectors are defined as follows:
        Transverse plane: y_axis relative to X on the XZ plane
        Frontal plane: z_axis relative to Y on the YX plane
        Sagittal plane: y_axis relative to Z on the YZ plane
            """""
    joint12_axis_dict = {'transverse': {'local_axis': 'z', 'global_plane': 'XZ', 'global_axis': 'Z'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'XY', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'YZ', 'global_axis': 'Z'}}

    joint23_axis_dict = {'transverse': {'local_axis': 'y', 'global_plane': 'XZ', 'global_axis': 'Z'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'ZY', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'XY', 'global_axis': 'Y'}}

    joint13_axis_dict = {'transverse': {'local_axis': 'y', 'global_plane': 'XZ', 'global_axis': 'X'},
                         'frontal': {'local_axis': 'z', 'global_plane': 'YX', 'global_axis': 'Y'},
                         'sagittal': {'local_axis': 'y', 'global_plane': 'YZ', 'global_axis': 'Z'}}

    # For joint12, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plotting)
    joint12_plot_dict = get_errors_and_plot_dict(joint12_axis_dict, IMU_joint12, OMC_joint12, start_time, end_time)
    plot_vec_angles_error(joint12_plot_dict, start_time, end_time, trial_results_dir, joint_of_interest='Joint12')

    # For joint23, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plotting)
    joint_23_end_time = time_elevation_starts  # Trim so that we don't include data when arm is elevated (keeps sagittal/frontal etc definition valid)
    joint23_plot_dict = get_errors_and_plot_dict(joint23_axis_dict, IMU_joint23, OMC_joint23, start_time, joint_23_end_time)
    plot_vec_angles_error(joint23_plot_dict, start_time, joint_23_end_time, trial_results_dir,
                          joint_of_interest='Joint23')

    # For joint13, Get time-series angles and errors, and error metrics RMSE, R and max (save into one dict for plot)
    joint13_plot_dict = get_errors_and_plot_dict(joint13_axis_dict, IMU_joint13, OMC_joint13, start_time, end_time)
    plot_vec_angles_error(joint13_plot_dict, start_time, end_time, trial_results_dir, joint_of_interest='Joint13')

    """ WRITE RESULTS TO CSV """

    final_results = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_name],
                                  'Joint12_Dist_Error_RMSE': joint12_dist_error_RMSE,
                                  'Joint23_Dist_Error_RMSE': joint23_dist_error_RMSE,
                                  'Joint13_Dist_Error_RMSE': joint13_dist_error_RMSE,
                                  'Joint12_Transverse_Error_RMSE': joint12_plot_dict['transverse']['RMSE'][0],
                                  'Joint23_Transverse_Error_RMSE': joint23_plot_dict['transverse']['RMSE'][0],
                                  'Joint13_Transverse_Error_RMSE': joint13_plot_dict['transverse']['RMSE'][0],
                                  'Joint12_Frontal_Error_RMSE': joint12_plot_dict['frontal']['RMSE'][0],
                                  'Joint23_Frontal_Error_RMSE': joint23_plot_dict['frontal']['RMSE'][0],
                                  'Joint13_Frontal_Error_RMSE': joint13_plot_dict['frontal']['RMSE'][0],
                                  'Joint12_Sagittal_Error_RMSE': joint12_plot_dict['sagittal']['RMSE'][0],
                                  'Joint23_Sagittal_Error_RMSE': joint23_plot_dict['sagittal']['RMSE'][0],
                                  'Joint13_Sagittal_Error_RMSE': joint13_plot_dict['sagittal']['RMSE'][0]})

    final_results.to_csv(trial_results_dir + "\\" + subject_code + '_' + trial_name + r"_IMU_accuracy_RMSEs.csv",
                         mode='w', encoding='utf-8', na_rep='nan')

    return final_results





# Read in IMU quaternion data from TMM report .txt file
def get_scipyR_from_txt_file(input_file, trim_bool, start_time, end_time, sample_rate):

    print(input_file)
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)

    # Trim the dataframes
    if trim_bool == True:
        start_index = start_time * sample_rate
        end_index = end_time * sample_rate
        IMU1_df = IMU1_df.iloc[start_index:end_index]
        IMU2_df = IMU2_df.iloc[start_index:end_index]
        IMU3_df = IMU3_df.iloc[start_index:end_index]

    # Turn dataframes into Scipy arrays
    IMU1_R = R.from_quat(np.array((IMU1_df.loc[:, ['IMU1_Q1', 'IMU1_Q2', 'IMU1_Q3', 'IMU1_Q0']])))
    IMU2_R = R.from_quat(np.array((IMU2_df.loc[:, ['IMU2_Q1', 'IMU2_Q2', 'IMU2_Q3', 'IMU2_Q0']])))
    IMU3_R = R.from_quat(np.array((IMU3_df.loc[:, ['IMU3_Q1', 'IMU3_Q2', 'IMU3_Q3', 'IMU3_Q0']])))

    return IMU1_R, IMU2_R, IMU3_R



# Get angle between two 2D vectors
def angle_between_two_2D_vecs(vec1, vec2):
    angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
    return angle



# Define a function for getting the projected vector of interest from the joint rotation matrix
def get_proj_vec_from_joint_rot_mat(rot_mat_R, local_axis, global_plane):

    matrices = rot_mat_R.as_matrix()

    column_dict = {'x': 0, 'y': 1, 'z': 2}
    row_dict = {'X': 0, 'Y': 1, 'Z': 2}

    column = column_dict[local_axis]
    row_a = row_dict[global_plane[0]]
    row_b = row_dict[global_plane[1]]

    array_of_vecs = matrices[:, [row_a, row_b], column]

    return array_of_vecs



# Define a function for calculating the angle between the local axis, and the chosen global axis
def get_proj_vec_angle(proj_vec, global_axis, global_plane):

    if global_axis == global_plane[0]:
        global_axis_vec = [1, 0]
    else:
        global_axis_vec = [0, 1]

    array_of_angles = np.arccos(np.dot(proj_vec, global_axis_vec) /
                                (np.linalg.norm(proj_vec, axis=1) * np.linalg.norm(global_axis_vec))) * 180 / np.pi

    return array_of_angles



# Define a function for getting the error between IMU and OMC projected angles
def get_error_arrays_and_stats(IMU_proj_vecs, OMC_proj_vecs, IMU_angles, OMC_angles):

    # Filter out projected vector angles when vector is close to normal to plane (i.e. not stable projection)
    # Use the magnitude of the projected vector as a measure, since mag will fluctuate between 1 and 0 as it moves
    # from perpendicular to normal to the plane
    # Filter both IMU and OMC arrays when either are close to normal

    keep_condition = (np.linalg.norm(IMU_proj_vecs, axis=1) > 0.5) & (np.linalg.norm(OMC_proj_vecs, axis=1) > 0.5)
    IMU_angles_filtered = np.where(keep_condition, IMU_angles, np.nan)
    OMC_angles_filtered = np.where(keep_condition, OMC_angles, np.nan)

    # Use these filtered arrays to calculate error
    error_arr = abs(IMU_angles_filtered - OMC_angles_filtered)

    # Remove nans for max and RMSE calcs
    error_arr_no_nans = error_arr[~np.isnan(error_arr)]

    # Get the maximum error
    max_error = np.amax(error_arr_no_nans)

    # Calculate RMSE across the time-varying error
    RMSE = find_RMSE_of_error_array(error_arr_no_nans)

    # Calculate Pearson correlation coefficient
    IMU_angles_filtered_no_nans = IMU_angles_filtered[~np.isnan(IMU_angles_filtered)]# Remove nans to calculate pearson
    OMC_angles_filtered_no_nans = OMC_angles_filtered[~np.isnan(OMC_angles_filtered)]# Remove nans to calculate pearson
    R = pearsonr(IMU_angles_filtered_no_nans, OMC_angles_filtered_no_nans)[0]

    return RMSE, R, error_arr, max_error, IMU_angles_filtered, OMC_angles_filtered


def get_errors_and_plot_dict(axis_dict, IMU_joint_R, OMC_joint_R, error_start_time, error_end_time):

    # Trim the data so we only calculate errors for the time of interest
    start_index = error_start_time * 100
    end_index = error_end_time * 100
    IMU_joint_R = IMU_joint_R[start_index:end_index]
    OMC_joint_R = OMC_joint_R[start_index:end_index]

    # Instantiate dict of plotting info for each plane
    plotting_dict = {'transverse':
                         {'IMU_angles': [],
                          'OMC_angles': [],
                          'IMU_angles_filtered': [],
                          'OMC_angles_filtered': [],
                          'error_arr': [],
                          'RMSE': [],
                          'max_error': []},
                     'frontal':
                         {'IMU_angles': [],
                          'OMC_angles': [],
                          'IMU_angles_filtered': [],
                          'OMC_angles_filtered': [],
                          'error_arr': [],
                          'RMSE': [],
                          'max_error': []},
                     'sagittal':
                         {'IMU_angles': [],
                          'OMC_angles': [],
                          'IMU_angles_filtered': [],
                          'OMC_angles_filtered': [],
                          'error_arr': [],
                          'RMSE': [],
                          'max_error': []},
                     }

    for key in axis_dict.keys():
        local_axis = axis_dict[key]['local_axis']
        global_axis = axis_dict[key]['global_axis']
        global_plane = axis_dict[key]['global_plane']

        IMU_proj_vecs = get_proj_vec_from_joint_rot_mat(IMU_joint_R, local_axis, global_plane)
        OMC_proj_vecs = get_proj_vec_from_joint_rot_mat(OMC_joint_R, local_axis, global_plane)
        IMU_angles = get_proj_vec_angle(IMU_proj_vecs, global_axis, global_plane)
        OMC_angles = get_proj_vec_angle(OMC_proj_vecs, global_axis, global_plane)
        RMSE, R, error_arr, max_error, IMU_angles_filtered, OMC_angles_filtered = \
            get_error_arrays_and_stats(IMU_proj_vecs, OMC_proj_vecs, IMU_angles, OMC_angles)

        # Add values to dict for plotting
        for key2 in plotting_dict[key].keys():
            plotting_dict[key][key2].append(eval(key2))

    return plotting_dict



# Define a function for plotting the changing projected vector angles, and the changing error
def plot_vec_angles_error(plotting_dict, start_time, end_time, figure_results_dir, joint_of_interest):


    label1 = 'Transverse'
    label2 = 'Frontal'
    label3 = 'Sagittal'
    labelA = 'OMC'
    labelB = 'IMU'

    start_index = start_time*100
    end_index = end_time*100

    # Get the values from the dict
    IMU_angle1 = plotting_dict['transverse']['IMU_angles'][0][start_index:end_index]
    IMU_angle2 = plotting_dict['frontal']['IMU_angles'][0][start_index:end_index]
    IMU_angle3 = plotting_dict['sagittal']['IMU_angles'][0][start_index:end_index]
    IMU_angle1_filtered = plotting_dict['transverse']['IMU_angles_filtered'][0][start_index:end_index]
    IMU_angle2_filtered = plotting_dict['frontal']['IMU_angles_filtered'][0][start_index:end_index]
    IMU_angle3_filtered = plotting_dict['sagittal']['IMU_angles_filtered'][0][start_index:end_index]
    OMC_angle1 = plotting_dict['transverse']['OMC_angles'][0][start_index:end_index]
    OMC_angle2 = plotting_dict['frontal']['OMC_angles'][0][start_index:end_index]
    OMC_angle3 = plotting_dict['sagittal']['OMC_angles'][0][start_index:end_index]
    OMC_angle1_filtered = plotting_dict['transverse']['OMC_angles_filtered'][0][start_index:end_index]
    OMC_angle2_filtered = plotting_dict['frontal']['OMC_angles_filtered'][0][start_index:end_index]
    OMC_angle3_filtered = plotting_dict['sagittal']['OMC_angles_filtered'][0][start_index:end_index]
    RMSE_angle1 = plotting_dict['transverse']['RMSE'][0]
    RMSE_angle2 = plotting_dict['frontal']['RMSE'][0]
    RMSE_angle3 = plotting_dict['sagittal']['RMSE'][0]
    max_error_angle1 = plotting_dict['transverse']['max_error'][0]
    max_error_angle2 = plotting_dict['frontal']['max_error'][0]
    max_error_angle3 = plotting_dict['sagittal']['max_error'][0]
    error_angle1 = plotting_dict['transverse']['error_arr'][0][start_index:end_index]
    error_angle2 = plotting_dict['frontal']['error_arr'][0][start_index:end_index]
    error_angle3 = plotting_dict['sagittal']['error_arr'][0][start_index:end_index]

    # Create a time variable
    time = np.linspace(start_time, end_time, len(OMC_angle1))

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 2, figsize=(14,9), width_ratios=[9,1])


    # Plot joint angles
    line1, = axs[0,0].plot(time, OMC_angle1, linestyle='dotted', c='lightgrey')
    line2, = axs[0,0].plot(time, IMU_angle1, linestyle='dotted', c='lightgrey')
    line3, = axs[0,0].plot(time, OMC_angle1_filtered)
    line4, = axs[0,0].plot(time, IMU_angle1_filtered)

    line1, = axs[1,0].plot(time, OMC_angle2, linestyle='dotted', c='lightgrey')
    line2, = axs[1,0].plot(time, IMU_angle2, linestyle='dotted', c='lightgrey')
    line3, = axs[1,0].plot(time, OMC_angle2_filtered)
    line4, = axs[1,0].plot(time, IMU_angle2_filtered)

    line1, = axs[2,0].plot(time, OMC_angle3, linestyle='dotted', c='lightgrey')
    line2, = axs[2,0].plot(time, IMU_angle3, linestyle='dotted', c='lightgrey')
    line3, = axs[2,0].plot(time, OMC_angle3_filtered)
    line4, = axs[2,0].plot(time, IMU_angle3_filtered)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)

    for i in range(0, 3):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([line3, line4], [labelA, labelB])
        axs[i,0].grid(color="lightgrey")


    # Plot error graphs
    axs[0,1].scatter(time, error_angle1, s=0.4)
    axs[1,1].scatter(time, error_angle2, s=0.4)
    axs[2,1].scatter(time, error_angle3, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE*1.1):
            text_placement = RMSE*1.1
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 3):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])), xlim=(start_time, end_time))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + "_angles.png")

    plt.close()




def get_heading_rotation_using_2D_vecs(IMU_R, OMC_R):
    IMU_thorax_x_vector_on_XZ_plane = [IMU_R[0].as_matrix()[0, 0], IMU_R[0].as_matrix()[2, 0]]
    OMC_thorax_x_vector_on_XZ_plane = [OMC_R[0].as_matrix()[0, 0], OMC_R[0].as_matrix()[2, 0]]
    heading_offset = angle_between_two_2D_vecs(IMU_thorax_x_vector_on_XZ_plane, OMC_thorax_x_vector_on_XZ_plane)
    # Rotate all IMU data by this angle around
    heading_rotation = R.from_euler('y', [heading_offset], degrees=True)
    return heading_rotation


def write_rot_quats_to_sto(df1, df2, df3, trial_name, APDM_template_file, APDM_settings_file, results_folder, IMU_type):

    sto_file_tag = trial_name + '_RotatedQuats_' + IMU_type
    IMU1_df = pd.DataFrame(df1.as_quat()[:, [3, 0, 1, 2]], columns=['IMU1_Q0', 'IMU1_Q1', 'IMU1_Q2', 'IMU1_Q3'])
    IMU2_df = pd.DataFrame(df2.as_quat()[:, [3, 0, 1, 2]], columns=['IMU2_Q0', 'IMU2_Q1', 'IMU2_Q2', 'IMU2_Q3'])
    IMU3_df = pd.DataFrame(df3.as_quat()[:, [3, 0, 1, 2]], columns=['IMU3_Q0', 'IMU3_Q1', 'IMU3_Q2', 'IMU3_Q3'])
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, APDM_template_file, results_folder, sto_file_tag)
    # Write data to .sto using OpenSim APDM converter tool
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_folder + "\\" + sto_file_tag + ".csv",
                         output_file_name=results_folder + "\\" + sto_file_tag + ".sto")


def plot_single_angle_diff(error1, error2, error3, start_time, end_time, figure_results_dir):

    label1 = 'Joint12: Thorax-Humerus'
    label2 = 'Joint23: Humerus-Forearm'
    label3 = 'Joint13: Thorax-Forearm'

    # Create a time variable
    time = np.linspace(start_time, end_time, len(error1))

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error1)) / len(error1)) ** 0.5
    RMSE_angle2 = (sum(np.square(error2)) / len(error2)) ** 0.5
    RMSE_angle3 = (sum(np.square(error3)) / len(error3)) ** 0.5
    max_error_angle1 = np.amax(error1)
    max_error_angle2 = np.amax(error2)
    max_error_angle3 = np.amax(error3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(14,9))

    # Plot error graphs

    axs[0].scatter(time, error1, s=0.4)
    axs[1].scatter(time, error2, s=0.4)
    axs[2].scatter(time, error3, s=0.4)

    axs[0].set_title(label1)
    axs[1].set_title(label2)
    axs[2].set_title(label3)

    # Plot RMSE error lines and text
    axs[0].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

    # Functions to define placement of max error annotation
    def y_max_line_placement(max_error):
        if max_error > 40:
            line_placement = 40
        else:
            line_placement = max_error
        return line_placement

    def y_max_text_placement(max_error, RMSE):
        if max_error > 40:
            text_placement = 40
        elif max_error < (RMSE + 3):
            text_placement = RMSE*1.1
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    axs[0].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    y_lim_list = np.array([max_error_angle1, max_error_angle2, max_error_angle3])
    for i in range(0, 3):
        axs[i].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0, 1.1*y_lim_list[i]))
        axs[i].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + r"\Single_Angle_Diff.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3