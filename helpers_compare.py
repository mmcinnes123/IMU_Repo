# Functions used in compare.py

from quat_functions import quat_mul
from quat_functions import quat_conj
from constants import sample_rate

import os
from os.path import join
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
from scipy import signal
from scipy.signal import find_peaks
from scipy.stats import pearsonr
from scipy.spatial.transform import Rotation as R
from matplotlib.widgets import SpanSelector

def debug_print(**kwargs):
    for name, value in kwargs.items():
        print(f"{name} = {value}")

# Take an osim tables of coords and check that certain coords are within a specified sensible range
def check_sensible_GHs(file_name, table, GH_coord_limit):
    all_GH_coords = []
    for coord in ['GH_y', 'GH_z', 'GH_x']:
        all_GH_coords.append(table.getDependentColumn(coord).to_numpy())
    all_GH_coords = np.array(all_GH_coords)
    if np.any((all_GH_coords > GH_coord_limit) | (all_GH_coords < -GH_coord_limit)):
        print(f'WARNING: IMU mot file has GH values above {GH_coord_limit} for file: {file_name}')


# Turn the tables into dataframes
def convert_osim_table_to_df(table):
    time = np.asarray(table.getIndependentColumn()[:])
    df = pd.DataFrame(time, columns=['time'])
    for column_name in table.getColumnLabels():
        df[column_name] = table.getDependentColumn(column_name).to_numpy()
    return df


def get_body_quats_from_analysis_sto(analysis_sto_path, start_time, end_time):

    # Read in the analysis .sto file with pos and ori data for each model body
    analysis_table = osim.TimeSeriesTable(analysis_sto_path)   # Read in new states

    # # Trim based on start and end times
    # analysis_table.trim(start_time, end_time)

    # Create an array of the XYZ Eulers used to define the body oris
    thorax_Ox = analysis_table.getDependentColumn('thorax_Ox').to_numpy()
    thorax_Oy = analysis_table.getDependentColumn('thorax_Oy').to_numpy()
    thorax_Oz = analysis_table.getDependentColumn('thorax_Oz').to_numpy()
    humerus_Ox = analysis_table.getDependentColumn('humerus_r_Ox').to_numpy()
    humerus_Oy = analysis_table.getDependentColumn('humerus_r_Oy').to_numpy()
    humerus_Oz = analysis_table.getDependentColumn('humerus_r_Oz').to_numpy()
    radius_Ox = analysis_table.getDependentColumn('radius_r_Ox').to_numpy()
    radius_Oy = analysis_table.getDependentColumn('radius_r_Oy').to_numpy()
    radius_Oz = analysis_table.getDependentColumn('radius_r_Oz').to_numpy()
    time = np.asarray(analysis_table.getIndependentColumn()[:])
    thorax_eulers = np.stack((thorax_Ox, thorax_Oy, thorax_Oz), axis=1)
    humerus_eulers = np.stack((humerus_Ox, humerus_Oy, humerus_Oz), axis=1)
    radius_eulers = np.stack((radius_Ox, radius_Oy, radius_Oz), axis=1)

    # Create an array of scipy Rotations
    thorax_R = R.from_euler('XYZ', thorax_eulers, degrees=True)
    humerus_R = R.from_euler('XYZ', humerus_eulers, degrees=True)
    radius_R = R.from_euler('XYZ', radius_eulers, degrees=True)
    thorax_quats = thorax_R.as_quat()[:,[3, 0, 1, 2]]   # Convert from scalar-last to scalar-first format
    humerus_quats = humerus_R.as_quat()[:,[3, 0, 1, 2]]
    radius_quats = radius_R.as_quat()[:,[3, 0, 1, 2]]


    # Concatenate the arrays and turn into labelled dataframe
    time = time.reshape(-1, 1)
    combined_array = np.hstack((time, thorax_quats, humerus_quats, radius_quats))
    body_quats = pd.DataFrame(combined_array, columns=['time', 'thorax_q0', 'thorax_q1', 'thorax_q2', 'thorax_q3',
                                                       'humerus_q0', 'humerus_q1', 'humerus_q2', 'humerus_q3',
                                                       'radius_q0', 'radius_q1', 'radius_q2', 'radius_q3'])
    return body_quats


# Trim the dataframes based on start and end times
def trim_df(df, start_time, end_time):
    filtered_df = df[(np.round(df['time'], 2) > start_time) & (np.round(df['time'], 2) < end_time)]
    return filtered_df


# Get the HT projected vector angles from a dataframe of model body orientations
def get_HT_angles(body_quats_df):

    thorax_quats_arr = get_quat_arr_from_df(body_quats_df, 'thorax')
    humerus_quats_arr = get_quat_arr_from_df(body_quats_df, 'humerus')

    abd, flex, rot, rot_el_up = get_vec_angles_from_two_CFs(thorax_quats_arr, humerus_quats_arr)

    HT_angles = body_quats_df[['time']].copy()
    HT_angles['HT_abd'] = abd
    HT_angles['HT_flexion'] = flex
    HT_angles['HT_rotation'] = rot

    return HT_angles


def get_range_dict(JA_range_dict_file, IMU_angles, OMC_angles):

    if os.path.exists(JA_range_dict_file) == False:

        time = OMC_angles['time'].to_numpy()
        range_dict = {}

        for col in IMU_angles[['elbow_flexion', 'elbow_pronation', 'HT_rotation', 'HT_abd', 'HT_flexion']].columns:
            OMC_angle = OMC_angles[col].to_numpy()
            IMU_angle = IMU_angles[col].to_numpy()
            time_start, time_end = run_span_selector(OMC_angle, IMU_angle, time, joint_name=col)
            range_dict[col] = [time_start, time_end]

        # Save dict to .txt
        file_obj = open(JA_range_dict_file, 'w')
        file_obj.write(str(range_dict))
        file_obj.close()

    else:
        file_obj = open(JA_range_dict_file, 'r')
        range_dict_str = file_obj.read()
        file_obj.close()
        range_dict = eval(range_dict_str)

    return range_dict


# A function for calculating the average heading offset between an array of two bodies or IMUs, relative to a global frame
def find_heading_offset(OMC_body_quats, IMU_body_quats, report):

    # Split the dataframes into np arrays
    OMC_thorax_quats = get_quat_arr_from_df(OMC_body_quats, 'thorax')
    IMU_thorax_quats = get_quat_arr_from_df(IMU_body_quats, 'thorax')

    def find_heading_of_thorax(thorax_quats, row):
        thorax_scipy_R = R.from_quat([thorax_quats[row,1], thorax_quats[row,2], thorax_quats[row,3], thorax_quats[row,0]])
        thorax_rot_mat = thorax_scipy_R.as_matrix()
        # Calculating angle of thorax z-axis on the glboal XZ plane
        mat_z_Z = thorax_rot_mat[2,2]
        mat_z_X = thorax_rot_mat[0,2]
        angle_z = np.arctan(mat_z_X / mat_z_Z)
        return angle_z

    heading_offset_arr = np.zeros((len(OMC_thorax_quats)))
    for row in range(len(OMC_thorax_quats)):
        OMC_thorax_offset_i = find_heading_of_thorax(OMC_thorax_quats, row)
        IMU_thorax_offset_i = find_heading_of_thorax(IMU_thorax_quats, row)
        heading_offset_arr[row] = OMC_thorax_offset_i - IMU_thorax_offset_i

    angle_z = np.mean(heading_offset_arr)

    if report:
        print(f'Average heading difference between thorax IMU and OMC cluster is: {np.rad2deg(angle_z)} degrees.')

    return angle_z


def plot_compare_any_JAs(joint_name, IMU_angles, OMC_angles, start_time, end_time, figure_results_dir,
                                 range_dict):

    IMU_angle = IMU_angles[[joint_name]].to_numpy()
    OMC_angle = OMC_angles[[joint_name]].to_numpy()
    time = IMU_angles[['time']].to_numpy()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    label = joint_name.replace('_', ' ').title()

    """ Get the peaks and troughs """

    if joint_name not in ('thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation'):

        plot_peaks = True   # Don't bother calculating or plotting thorax peaks
        time_start, time_end = range_dict[joint_name][0], range_dict[joint_name][1]   # Get the indices saved in the range dict

        OMC_peaks, OMC_peak_inds, OMC_peak_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
        IMU_peaks, IMU_peak_inds, IMU_peak_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
        OMC_troughs, OMC_trough_inds, OMC_trough_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)
        IMU_troughs, IMU_trough_inds, IMU_trough_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)

        # Check we have found enough peaks/troughs
        if any(len(var) < 4 for var in [OMC_peaks, IMU_peaks, OMC_troughs, IMU_troughs]):
            print(f"WARNING: No/not enough peaks found for {joint_name} (less than 4)")

        # Get the mean peak/trough error
        mean_peak_error = get_peak_and_trough_errors(OMC_peaks, IMU_peaks, joint_name)
        mean_trough_error = get_peak_and_trough_errors(OMC_troughs, IMU_troughs, joint_name)

    else:
        plot_peaks = False
        mean_peak_error = np.nan     # Sub in value for plotting
        mean_trough_error = np.nan   # Sub in value for plotting

    """ CROSS CORRELATE """

    # Calculate cross-correlation lag
    lag = get_cross_cor_lag(OMC_angle, IMU_angle)

    # Shift IMU and OMC data if lag is within sensible range
    if -20 < lag < 0:
        IMU_angle = IMU_angle[-lag:]    # Remove first n values from IMU data
        OMC_angle = OMC_angle[:lag]     # Remove last n values from OMC data
        time = time[:lag]               # Remove last n values from time array
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => APPLIED')
    else:
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => NOT APPLIED')

    """ GET ERROR METRICS """

    error_angle1 = abs(OMC_angle - IMU_angle)       # Calculate error array
    R = get_pearsonr(OMC_angle, IMU_angle)          # Calculate Pearson correlation coefficient
    RMSE_angle1 = get_RMSE(error_angle1)            # Calculate RMSE
    max_error_angle1 = np.nanmax(error_angle1)      # Calculate max error

    """ CREATE FIGURE """

    fig, axs = plt.subplots(2, 1, figsize=(16,8), height_ratios=[7,3])

    """ Plot joint angles """

    # Axs 0 settings
    axs[0].set_title(label)
    axs[0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
    axs[0].legend(['OMC', 'IMU'])
    axs[0].grid(color="lightgrey")

    # Plot the joint angles
    axs[0].plot(time, OMC_angle)
    axs[0].plot(time, IMU_angle)

    """ Plot the peaks and troughs """

    if plot_peaks == True:
        axs[0].plot(OMC_trough_times, OMC_troughs, "x", c='blue')
        axs[0].plot(OMC_peak_times, OMC_peaks, "x", c='blue')
        axs[0].plot(IMU_trough_times, IMU_troughs, "x", c='orange')
        axs[0].plot(IMU_peak_times, IMU_peaks, "x", c='orange')

        # Annotate with mean peak/trough error
        y_min, y_max = axs[0].get_ylim()
        y_mid = (y_min + y_max) / 2
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid+10,
                    "Mean peak\nerror = " + str(round(mean_peak_error,1)) + " deg")
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid-10,
                    "Mean trough\nerror = " + str(round(mean_trough_error,1)) + " deg")

    """ Plot error graphs """

    # Axs 1 settings
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*max_error_angle1])))
    axs[1].grid(color="lightgrey")

    # Plot to time-series error
    axs[1].scatter(time, error_angle1, s=0.4)

    # Plot RMSE error lines and text
    axs[1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")

    fig.tight_layout(pad=2.0)
    fig.savefig(figure_results_dir + "\\" + joint_name + "_angles.png")
    plt.close()

    # Throw warning and quit if not equal number of peaks/troughs was found
    if plot_peaks == True:
        if len(OMC_peaks) != len(IMU_peaks) or len(OMC_troughs) != len(IMU_troughs):
            # print("Quit because number of OMC peaks/troughs found did not match IMU peaks/troughs found.")
            # quit()
            mean_peak_error = np.nan
            mean_trough_error = np.nan

    return RMSE_angle1, R, mean_peak_error, mean_trough_error

def plot_BA_any_JAs(joint_name, IMU_angles, OMC_angles, start_time, end_time, figure_results_dir,
                                 range_dict):

    IMU_angle = IMU_angles[[joint_name]].to_numpy()
    OMC_angle = OMC_angles[[joint_name]].to_numpy()
    time = IMU_angles[['time']].to_numpy()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    label = joint_name.replace('_', ' ').title()

    """ Get the peaks and troughs """

    """ CROSS CORRELATE """

    # Calculate cross-correlation lag
    lag = get_cross_cor_lag(OMC_angle, IMU_angle)

    # Shift IMU and OMC data if lag is within sensible range
    if -20 < lag < 0:
        IMU_angle = IMU_angle[-lag:]    # Remove first n values from IMU data
        OMC_angle = OMC_angle[:lag]     # Remove last n values from OMC data
        time = time[:lag]               # Remove last n values from time array
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => APPLIED')
    else:
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => NOT APPLIED')

    """ GET ERROR METRICS """

    error_angle1 = IMU_angle - OMC_angle       # Calculate error array

    # Calculate mean between OMC and IMU angles
    mean_angles = (OMC_angle + IMU_angle) / 2

    # Calculate bias (mean difference)
    bias = np.mean(error_angle1)

    # Calculate limits of agreement (LOA)
    std_diff = np.std(error_angle1)
    upper_loa = bias + (1.96 * std_diff)
    lower_loa = bias - (1.96 * std_diff)

    # Create the Bland-Altman plot
    plt.figure(figsize=(10, 6))
    plt.scatter(mean_angles, error_angle1, alpha=0.5)
    plt.axhline(y=bias, color='k', linestyle='-', label='Bias')
    plt.axhline(y=upper_loa, color='r', linestyle='--', label='+1.96 SD')
    plt.axhline(y=lower_loa, color='r', linestyle='--', label='-1.96 SD')

    plt.xlabel('Mean of OMC and IMU measurements')
    plt.ylabel('Absolute Error (OMC - IMU)')
    plt.title(f'Bland-Altman Plot for {label}')
    plt.legend()

    # Save the figure
    plt.savefig(join(figure_results_dir, f'bland_altman_{joint_name}.png'))
    plt.close()

def plot_BA_any_JAs_with_peak_data(joint_name, IMU_angles, OMC_angles, start_time, end_time, figure_results_dir,
                                 range_dict):

    IMU_angle = IMU_angles[[joint_name]].to_numpy()
    OMC_angle = OMC_angles[[joint_name]].to_numpy()
    time = IMU_angles[['time']].to_numpy()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    label = joint_name.replace('_', ' ').title()

    """ Get the peaks and troughs """

    time_start, time_end = range_dict[joint_name][0], range_dict[joint_name][1]  # Get the indices saved in the range dict

    OMC_peaks, OMC_peak_inds, OMC_peak_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end,
                                                                    peak_or_trough='peak', debug=False)
    IMU_peaks, IMU_peak_inds, IMU_peak_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end,
                                                                    peak_or_trough='peak', debug=False)
    OMC_troughs, OMC_trough_inds, OMC_trough_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end,
                                                                          peak_or_trough='trough', debug=False)
    IMU_troughs, IMU_trough_inds, IMU_trough_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end,
                                                                          peak_or_trough='trough', debug=False)

    # Check we have found enough peaks/troughs
    if any(len(var) < 4 for var in [OMC_peaks, IMU_peaks, OMC_troughs, IMU_troughs]):
        print(f"WARNING: No/not enough peaks found for {joint_name} (less than 4)")

    # Get the mean peak/trough error
    IMU_peaks_and_troughs = np.concatenate((IMU_peaks, IMU_troughs))
    OMC_peaks_and_troughs = np.concatenate((OMC_peaks, OMC_troughs))
    peak_trough_errors = IMU_peaks_and_troughs - OMC_peaks_and_troughs
    peak_trough_mean_angles = (IMU_peaks_and_troughs + OMC_peaks_and_troughs) / 2

    # Calculate bias (mean difference)
    bias = np.mean(peak_trough_errors)

    # Calculate limits of agreement (LOA)
    std_diff = np.std(peak_trough_errors)
    upper_loa = bias + (1.96 * std_diff)
    lower_loa = bias - (1.96 * std_diff)

    # Create the Bland-Altman plot
    plt.figure(figsize=(10, 6))
    plt.scatter(peak_trough_mean_angles, peak_trough_errors, alpha=0.5)
    plt.axhline(y=bias, color='k', linestyle='-', label='Bias')
    plt.axhline(y=upper_loa, color='r', linestyle='--', label='+1.96 SD')
    plt.axhline(y=lower_loa, color='r', linestyle='--', label='-1.96 SD')

    plt.xlabel('Mean of OMC and IMU measurements')
    plt.ylabel('Absolute Error (OMC - IMU)')
    plt.title(f'Bland-Altman Plot for {label}')
    plt.legend()

    # Save the figure
    plt.savefig(join(figure_results_dir, f'bland_altman_peaktroughs{joint_name}.png'))
    plt.close()






def alt_plot_for_thesis_compare_any_JAs(joint_name, IMU_angles, OMC_angles, start_time, end_time, figure_results_dir,
                                 range_dict, compare_name):

    IMU_angle = IMU_angles[[joint_name]].to_numpy()
    OMC_angle = OMC_angles[[joint_name]].to_numpy()
    time = IMU_angles[['time']].to_numpy()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    label = joint_name.replace('_', ' ').title() + ' Angle\n '

    """ Get the peaks and troughs """

    if joint_name not in ('thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation'):

        plot_peaks = False   # Don't bother calculating or plotting thorax peaks
        time_start, time_end = range_dict[joint_name][0], range_dict[joint_name][1]   # Get the indices saved in the range dict

        OMC_peaks, OMC_peak_inds, OMC_peak_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
        IMU_peaks, IMU_peak_inds, IMU_peak_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
        OMC_troughs, OMC_trough_inds, OMC_trough_times = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)
        IMU_troughs, IMU_trough_inds, IMU_trough_times = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)

        # Check we have found enough peaks/troughs
        if any(len(var) < 4 for var in [OMC_peaks, IMU_peaks, OMC_troughs, IMU_troughs]):
            print(f"WARNING: No/not enough peaks found for {joint_name} (less than 4)")

        # Get the mean peak/trough error
        mean_peak_error = get_peak_and_trough_errors(OMC_peaks, IMU_peaks, joint_name)
        mean_trough_error = get_peak_and_trough_errors(OMC_troughs, IMU_troughs, joint_name)

    else:
        plot_peaks = False
        mean_peak_error = np.nan     # Sub in value for plotting
        mean_trough_error = np.nan   # Sub in value for plotting

    """ CROSS CORRELATE """

    # Calculate cross-correlation lag
    lag = get_cross_cor_lag(OMC_angle, IMU_angle)

    # Shift IMU and OMC data if lag is within sensible range
    if -20 < lag < 0:
        IMU_angle = IMU_angle[-lag:]    # Remove first n values from IMU data
        OMC_angle = OMC_angle[:lag]     # Remove last n values from OMC data
        time = time[:lag]               # Remove last n values from time array
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => APPLIED')
    else:
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => NOT APPLIED')

    """ GET ERROR METRICS """

    error_angle1 = abs(OMC_angle - IMU_angle)       # Calculate error array
    R = get_pearsonr(OMC_angle, IMU_angle)          # Calculate Pearson correlation coefficient
    RMSE_angle1 = get_RMSE(error_angle1)            # Calculate RMSE
    max_error_angle1 = np.nanmax(error_angle1)      # Calculate max error

    """ CREATE FIGURE """

    plt.rcParams.update({'font.family': 'Times New Roman', 'font.size': 22})

    fig, axs = plt.subplots(2, 1, figsize=(16,9), height_ratios=[7,3])

    """ Plot joint angles """

    # Axs 0 settings
    axs[0].set_title(label, fontsize=24)
    axs[0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
    axs[0].legend(['OMC', 'IMU'])
    axs[0].grid(color="lightgrey")

    # Plot the joint angles
    axs[0].plot(time, OMC_angle, color='#003f5c', label='OMC')
    axs[0].plot(time, IMU_angle, color='#fe8616', label='IMC')

    axs[0].legend()  # Adds a legend using the labels

    """ Plot the peaks and troughs """

    if plot_peaks == True:
        axs[0].plot(OMC_trough_times, OMC_troughs, "x", c='blue')
        axs[0].plot(OMC_peak_times, OMC_peaks, "x", c='blue')
        axs[0].plot(IMU_trough_times, IMU_troughs, "x", c='orange')
        axs[0].plot(IMU_peak_times, IMU_peaks, "x", c='orange')

        # Annotate with mean peak/trough error
        y_min, y_max = axs[0].get_ylim()
        y_mid = (y_min + y_max) / 2
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid+10,
                    "Mean peak\nerror = " + str(round(mean_peak_error,1)) + " deg")
        axs[0].text(time[-1]+0.1*(end_time-start_time), y_mid-10,
                    "Mean trough\nerror = " + str(round(mean_trough_error,1)) + " deg")

    """ Plot error graphs """

    # Axs 1 settings
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,40))
    axs[1].grid(color="lightgrey")

    # Plot to time-series error
    axs[1].scatter(time, error_angle1, s=0.4)

    # Plot RMSE error lines and text
    axs[1].axhline(y=RMSE_angle1, linewidth=2, c="red", linestyle='--')
    axs[1].text(time[-1]+0.08*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    # axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red", linestyle='--')

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    # axs[1].text(time[-1]+0.08*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")

    fig.tight_layout(pad=2.0)
    fig.savefig(r'C:\Users\r03mm22\Documents\Protocol_Testing\Results\Other_Figs_From_Python' + "\\" + compare_name + joint_name + "_angles.png")
    plt.close()

    # Throw warning and quit if not equal number of peaks/troughs was found
    if plot_peaks == True:
        if len(OMC_peaks) != len(IMU_peaks) or len(OMC_troughs) != len(IMU_troughs):
            # print("Quit because number of OMC peaks/troughs found did not match IMU peaks/troughs found.")
            # quit()
            mean_peak_error = np.nan
            mean_trough_error = np.nan


# Define a function to plot IMU vs OMC model body orientation errors (single angle quaternion difference)
def plot_compare_body_oris(OMC_body_quats, IMU_body_quats,
                           heading_offset, start_time, end_time, figure_results_dir):

    thorax_OMC = get_quat_arr_from_df(OMC_body_quats, 'thorax')
    humerus_OMC = get_quat_arr_from_df(OMC_body_quats, 'humerus')
    radius_OMC = get_quat_arr_from_df(OMC_body_quats, 'radius')
    thorax_IMU = get_quat_arr_from_df(IMU_body_quats, 'thorax')
    humerus_IMU = get_quat_arr_from_df(IMU_body_quats, 'humerus')
    radius_IMU = get_quat_arr_from_df(IMU_body_quats, 'radius')
    time = OMC_body_quats['time'].to_numpy()

    label1 = "Thorax Orientation Error" + " (heading offset applied: " + str(round(heading_offset*180/np.pi,1)) + "deg)"
    label2 = "Humerus Orientation Error"
    label3 = "Radius Orientation Error"

    # Apply heading offset to all IMU frames
    heading_offset_R = R.from_euler('y', [heading_offset])

    thorax_IMU_R = R.from_quat(thorax_IMU[:, [1, 2, 3, 0]])
    humerus_IMU_R = R.from_quat(humerus_IMU[:, [1, 2, 3, 0]])
    radius_IMU_R = R.from_quat(radius_IMU[:, [1, 2, 3, 0]])
    thorax_OMC_R = R.from_quat(thorax_OMC[:, [1, 2, 3, 0]])
    humerus_OMC_R = R.from_quat(humerus_OMC[:, [1, 2, 3, 0]])
    radius_OMC_R = R.from_quat(radius_OMC[:, [1, 2, 3, 0]])

    thorax_IMU_rotated = thorax_IMU_R*heading_offset_R
    humerus_IMU_rotated = humerus_IMU_R*heading_offset_R
    radius_IMU_rotated = radius_IMU_R*heading_offset_R

    def find_single_angle_diff_between_two_CFs(body1, body2):
        diff = body1.inv()*body2
        angle = diff.magnitude()*180/np.pi
        return angle

    thorax_ori_error = find_single_angle_diff_between_two_CFs(thorax_OMC_R, thorax_IMU_rotated)
    humerus_ori_error = find_single_angle_diff_between_two_CFs(humerus_OMC_R, humerus_IMU_rotated)
    radius_ori_error = find_single_angle_diff_between_two_CFs(radius_OMC_R, radius_IMU_rotated)


    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(thorax_ori_error)) / len(thorax_ori_error)) ** 0.5
    RMSE_angle2 = (sum(np.square(humerus_ori_error)) / len(humerus_ori_error)) ** 0.5
    RMSE_angle3 = (sum(np.square(radius_ori_error)) / len(radius_ori_error)) ** 0.5
    max_error_angle1 = np.amax(thorax_ori_error)
    max_error_angle2 = np.amax(humerus_ori_error)
    max_error_angle3 = np.amax(radius_ori_error)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(14,9))

    # Plot error graphs

    axs[0].scatter(time, thorax_ori_error, s=0.4)
    axs[1].scatter(time, humerus_ori_error, s=0.4)
    axs[2].scatter(time, radius_ori_error, s=0.4)

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

    fig.savefig(figure_results_dir + r"\Body_Orientation_Diff.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3


# Get a numpy array with quats specific to that body from a data frame of quats from multiple bodies
def get_quat_arr_from_df(df, body):
    column0 = body + '_q0'
    column1 = body + '_q1'
    column2 = body + '_q2'
    column3 = body + '_q3'
    arr = df[[column0, column1, column2, column3]].to_numpy()
    return arr


# Function used in IK_compare to trim data tables to same length
def trim_tables_if_diff_lengths(n, OMC_table, IMU_table):

    if n > 0:  # If OMC data is longer than IMU data
        for i in range(n):
            OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))  # Remove n rows from the OMC table
        print(f'Removed last {n} rows from OMC data')

    if n < 0:  # If IMU data is longer than OMC data
        for i in range(n):
            IMU_table.removeRowAtIndex((IMU_table.getNumRows() - 1))  # Remove n rows from the IMU table
        print(f'Removed last {n} rows from IMU data')

    return OMC_table, IMU_table


# Function used in IK_compare to trim data tables to same length
def trim_body_ori_data_to_same_length(n, thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC):

    # Remove n rows from the body ori data (trim to same length for easier plotting)
    for i in range(n):
        thorax_OMC = np.delete(thorax_OMC, [-1], 0)
        humerus_OMC = np.delete(humerus_OMC, [-1], 0)
        radius_OMC = np.delete(radius_OMC, [-1], 0)
        thorax_IMU = np.delete(thorax_IMU, [-1], 0)
        humerus_IMU = np.delete(humerus_IMU, [-1], 0)
        radius_IMU = np.delete(radius_IMU, [-1], 0)

    return thorax_IMU, humerus_IMU, radius_IMU, thorax_OMC, humerus_OMC, radius_OMC


# Function to calculate the cross-correlation lag between two series, accounting for nans
def get_cross_cor_lag(x, y):

    # Remove elements from both arrays where either array is a nan
    x_nonans = np.delete(x, np.union1d(np.where(np.isnan(x)), np.where(np.isnan(y))))
    y_nonans = np.delete(y, np.union1d(np.where(np.isnan(x)), np.where(np.isnan(y))))

    # Run the cross correlation
    correlation = signal.correlate(x_nonans, y_nonans, mode="full")
    lags = signal.correlation_lags(x_nonans.size, y_nonans.size, mode="full")
    lag = lags[np.argmax(abs(correlation))]  # Get the lag value at the index where correlation is largest

    # # Diagnosis/investigation code:
    # print(lags)
    # print(correlation)
    # # Get middle results around where lag = 0
    # indices_within_range = [index for index, lag in enumerate(lags) if -20 <= lag <= 20]
    # print(lags[indices_within_range])
    # print(correlation[indices_within_range])
    # fig, axs = plt.subplots(1, 1, figsize=(14,9))
    # axs.plot(lags, correlation)
    # # axs.plot(x_nonans)
    # # axs.plot(y_nonans)
    # plt.show()

    return lag


# Function for getting the peaks or troughs from an array
def get_peaks_or_troughs(angles_df, joint_name, time_start, time_end, peak_or_trough, debug):

    # Get the angle of interest, and find the period between the specified start and end times
    angle_df = angles_df[joint_name][angles_df['time'].between(time_start, time_end)]
    angle_arr = angle_df.to_numpy()

    # Define the minimum prominence when searching for the peaks
    from constants import prominence

    # Flip the data to find the troughs
    if peak_or_trough == 'peak':
        x = angle_arr
    elif peak_or_trough == 'trough':
        x = -angle_arr
    else:
        print('peak/trough not written correctly')
        x = None

    #  Run the find_peaks function on the range selected by the span selector
    peak_inds_in_selected_range, _ = find_peaks(x, prominence=prominence)

    # Get the peak indices expressed in the original range, not in the small section defined by the span selector
    peak_inds = peak_inds_in_selected_range + angle_df.index[0]

    # Values of the peaks
    peaks = angles_df[joint_name].loc[peak_inds].to_numpy()

    peak_times = angles_df['time'].loc[peak_inds].to_numpy()

    if debug:
        print(f'For {joint_name}, looking at {peak_or_trough}s:')
        for i in range(len(peaks)):
            print(f"At index = {peak_inds[i]}, angle = {peaks[i]}, at time {peak_times[i]}")
        plt.plot(angles_df.index[:], angles_df[joint_name].to_numpy())
        plt.plot(peak_inds, peaks, 'x')
        plt.show()

    return peaks, peak_inds, peak_times


# Function for getting the average error between OMC values and IMU values
def get_peak_and_trough_errors(OMC_peaks, IMU_peaks, joint_name):

    if len(OMC_peaks) == len(IMU_peaks):
        errors = abs(OMC_peaks - IMU_peaks)
        mean_error = np.mean(errors)
    else:
        print(f'Warning: number of OMC peaks/troughs does not equal number of IMU peaks/troughs found for joint: {joint_name}')
        mean_error = 0      # Fill in number for plotting
    return mean_error


# Function to calculate Pearson Corr Coeff, from two series, dealing with any nans
def get_pearsonr(OMC_angle, IMU_angle):
    # Remove any nans values present
    OMC_angle_no_nans = np.delete(OMC_angle, np.union1d(np.where(np.isnan(OMC_angle)), np.where(np.isnan(IMU_angle))))
    IMU_angle_no_nans = np.delete(IMU_angle, np.union1d(np.where(np.isnan(OMC_angle)), np.where(np.isnan(IMU_angle))))
    R = pearsonr(OMC_angle_no_nans, IMU_angle_no_nans)[0]
    return R


# Function to calculate RMSE from an error array, dealing with any nans
def get_RMSE(error_arr):
    error_arr = error_arr[~np.isnan(error_arr)]
    RMSE = (sum(np.square(error_arr)) / len(error_arr)) ** 0.5
    return RMSE


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


# Define functions for finding vector projected joint angles for the HT joint
def get_vec_angles_from_two_CFs(CF1, CF2):

    def angle_between_two_2D_vecs(vec1, vec2):
        angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
        if vec1[0] < 0:
            angle = -angle
        return angle

    n_rows = len(CF1)
    x_rel2_X_on_XY = np.zeros((n_rows))
    x_rel2_X_on_ZX = np.zeros((n_rows))
    z_rel2_Z_on_YZ = np.zeros((n_rows))
    y_rel2_Y_on_XY = np.zeros((n_rows))

    for row in range(n_rows):
        joint_rot = quat_mul(quat_conj(CF1[row]), CF2[row])  # Calculate joint rotation quaternion
        joint_scipyR = R.from_quat([joint_rot[1], joint_rot[2], joint_rot[3], joint_rot[0]])  # In scalar last format
        joint_mat = joint_scipyR.as_matrix()    # Calculate the joint rotation matrix
        # Extract the vector components of CF2 axes relative to CF1 axes
        mat_x_X = joint_mat[0,0]    # This is the component of the CF2 x axis in the CF1 X direction
        mat_x_Y = joint_mat[1,0]    # This is the component of the CF2 x axis in the CF1 Y direction
        mat_x_Z = joint_mat[2,0]    # This is the component of the CF2 x axis in the CF1 Z direction
        mat_y_X = joint_mat[0,1]    # This is the component of the CF2 y axis in the CF1 X direction
        mat_y_Y = joint_mat[1,1]    # This is the component of the CF2 y axis in the CF1 Y direction
        mat_y_Z = joint_mat[2,1]    # This is the component of the CF2 y axis in the CF1 Z direction
        mat_z_X = joint_mat[0,2]    # This is the component of the CF2 z axis in the CF1 X direction
        mat_z_Y = joint_mat[1,2]    # This is the component of the CF2 z axis in the CF1 Y direction
        mat_z_Z = joint_mat[2,2]    # This is the component of the CF2 z axis in the CF1 Z direction
        # Make these values up into vectors projected on certain planes
        vec_x_on_XY = [mat_x_X, mat_x_Y]
        X_on_XY = [1, 0]
        vec_x_on_ZX = [mat_x_Z, mat_x_X]
        X_on_ZX = [0, 1]
        vec_z_on_YZ = [mat_z_Y, mat_z_Z]
        Z_on_YZ = [0, 1]
        vec_y_on_XY = [mat_y_X, mat_y_Y]
        Y_on_XY = [0, 1]

        # Calculate the angle of certain CF2 vectors on certain CF1 planes
        # Discount measures of vector angle unless the 2D vector is a good/stable projection on the plane
        # i.e. If the magnitude of the vector gets close to 0, its normal to plane and so angle is unstable
        threshold = 0.5
        if np.linalg.norm(vec_x_on_XY) > threshold:
            x_rel2_X_on_XY[row] = angle_between_two_2D_vecs(vec_x_on_XY, X_on_XY)
        else:
            x_rel2_X_on_XY[row] = np.nan

        if np.linalg.norm(vec_x_on_ZX) > threshold:
            x_rel2_X_on_ZX[row] = angle_between_two_2D_vecs(vec_x_on_ZX, X_on_ZX)
        else:
            x_rel2_X_on_ZX[row] = np.nan

        if np.linalg.norm(vec_z_on_YZ) > threshold:
            z_rel2_Z_on_YZ[row] = angle_between_two_2D_vecs(vec_z_on_YZ, Z_on_YZ)
        else:
            z_rel2_Z_on_YZ[row] = np.nan

        if np.linalg.norm(vec_y_on_XY) > threshold:
            y_rel2_Y_on_XY[row] = angle_between_two_2D_vecs(vec_y_on_XY, Y_on_XY)
        else:
            y_rel2_Y_on_XY[row] = np.nan

    # Assign to clinically relevant joint angles
    abduction = -y_rel2_Y_on_XY
    flexion = -z_rel2_Z_on_YZ
    rotation_elbow_down = x_rel2_X_on_ZX
    rotation_elbow_up = z_rel2_Z_on_YZ

    return abduction, flexion, rotation_elbow_down, rotation_elbow_up


# Function to run interactive span selector - with mouse, select specific area on plot - returns the range (min, max)
def run_span_selector(OMC_angle, IMU_angle, time, joint_name):

    # Create the figure for selecting the span
    fig, ax1 = plt.subplots(1, figsize=(14, 8))
    ax1.plot(time, OMC_angle)
    ax1.plot(time, IMU_angle)
    ax1.set_title(f'{joint_name}\nClick and drag left to right to select region with peaks and troughs.')

    # Function which runs when selection is made
    def onselect(xmin, xmax):
        global xmin_selected, xmax_selected
        xmin_selected = xmin
        xmax_selected = xmax
        plt.close()

    # Create the span selector and define settings
    span = SpanSelector(
        ax1,        # this is the plot the span selector's used on
        onselect,   # this is the function which takes the inputs of the range from the mouse drag
        "horizontal",
        useblit=True,
        props=dict(alpha=0.5, facecolor="tab:blue"),
        interactive=True,
        drag_from_anywhere=True)

    # Open the plot, use span selector
    plt.show()

    # Start and end time values which were selected during the span selector
    time_start = xmin_selected
    time_end = xmax_selected

    return time_start, time_end





def get_ROM_and_var(OMC_angles, joint_name, debug):

    OMC_angle = OMC_angles[joint_name].to_numpy()
    time = OMC_angles['time'].to_numpy()

    label = joint_name.replace('_', ' ').title()

    min = OMC_angle.min()
    max = OMC_angle.max()
    range = OMC_angle.ptp()
    std = OMC_angle.std()

    if debug:
        debug_print(OMC_angle=OMC_angle, joint_name=joint_name, min=min, max=max, range=range, std=std)

    return min, max, range, std


def get_JAs_from_OMC_IK_results(subject_code, trial_name, start_time, end_time):

    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_IK_results_dir = os.path.join(parent_dir, 'OMC', trial_name + '_IK_Results')
    OMC_analysis_sto_path = os.path.join(OMC_IK_results_dir, 'analyze_BodyKinematics_pos_global.sto')
    OMC_mot_file = os.path.join(OMC_IK_results_dir, 'OMC_IK_results.mot')
    # debug_print(trial_name=trial_name, start_time=start_time, end_time=end_time)

    """ READ IN DATA """

    # Read in coordinates from IK results .mot files
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)

    # Turn the osim coords tables into dataframes
    OMC_coords = convert_osim_table_to_df(OMC_table)
    OMC_coords_trimmed = trim_df(OMC_coords, start_time, end_time)

    # Read in the orientation data of the model bodies from the analysis sto results file
    intial_time = math.floor(OMC_table.getIndependentColumn()[0])
    final_time = math.floor(OMC_table.getIndependentColumn()[-1])
    OMC_body_quats = get_body_quats_from_analysis_sto(OMC_analysis_sto_path, intial_time, final_time)
    OMC_body_quats_trimmed = trim_df(OMC_body_quats, start_time, end_time)

    # Get projected vector humero-thoracic joint angles from the model body orientations
    OMC_HT_angles = get_HT_angles(OMC_body_quats_trimmed)

    # Join the coord and HT angles data together to make one data frame with all angles
    rename_dict = {'TH_y': 'thorax_rotation', 'TH_x': 'thorax_forward_tilt', 'TH_z': 'thorax_lateral_tilt',
                   'EL_x': 'elbow_flexion', 'PS_y': 'elbow_pronation'}
    OMC_angles_from_coords = OMC_coords_trimmed[['TH_y', 'TH_x', 'TH_z', 'EL_x', 'PS_y']].copy().rename(
        columns=rename_dict)
    OMC_angles = pd.concat([OMC_angles_from_coords, OMC_HT_angles], axis=1)
    # debug_print(OMC_angles=OMC_angles)

    return OMC_angles