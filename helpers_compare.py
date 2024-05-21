# Functions used in compare.py

from quat_functions import quat_mul
from quat_functions import quat_conj

import os
import opensim as osim
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.signal import find_peaks
from scipy.stats import pearsonr
from scipy.spatial.transform import Rotation as R
from matplotlib.widgets import SpanSelector






def get_body_quats_from_analysis_sto(analysis_sto_path, start_time, end_time):

    # Read in the analysis .sto file with pos and ori data for each model body
    analysis_table = osim.TimeSeriesTable(analysis_sto_path)   # Read in new states

    # Trim based on start and end times
    analysis_table.trim(start_time, end_time)

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
    thorax_eulers = np.stack((thorax_Ox, thorax_Oy, thorax_Oz), axis=1)
    humerus_eulers = np.stack((humerus_Ox, humerus_Oy, humerus_Oz), axis=1)
    radius_eulers = np.stack((radius_Ox, radius_Oy, radius_Oz), axis=1)

    # Create an array of scipy Rotations
    thorax_R = R.from_euler('XYZ', thorax_eulers, degrees=True)
    humerus_R = R.from_euler('XYZ', humerus_eulers, degrees=True)
    radius_R = R.from_euler('XYZ', radius_eulers, degrees=True)

    thorax_quats = thorax_R.as_quat()[:,[1, 2, 3, 0]]
    humerus_quats = humerus_R.as_quat()[:,[1, 2, 3, 0]]
    radius_quats = radius_R.as_quat()[:,[1, 2, 3, 0]]

    return thorax_quats, humerus_quats, radius_quats


# A function for calculating the average heading offset between an array of two bodies or IMUs, relative to a global frame
def find_heading_offset(OMC_thorax_quats, IMU_thorax_quats):
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

    return angle_z


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
def get_peaks_or_troughs(angle_arr, indmin, indmax, peak_or_trough, data_type):

    # Define the minimum prominence when searching for the peaks
    prominence = 40

    # Fli pthe data to find the troughs
    if peak_or_trough == 'peak':
        x = angle_arr
    elif peak_or_trough == 'trough':
        x = -angle_arr
    else:
        print('peak/trough not written correctly')
        x = None

    #  Run the find_peaks function on the range selected by the span selector
    peak_inds_in_selected_range, _ = find_peaks(x[indmin:indmax], prominence=prominence)

    # Get the peak indices expressed in the original range, not in the small section defined by the span selector
    peak_inds = peak_inds_in_selected_range + indmin

    # Values of the peaks
    peaks = angle_arr[peak_inds]

    # # For diagnostics
    # print(f'For {data_type}, looking at {peak_or_trough}s:')
    # for i in range(len(peaks)):
    #     print(f"At index = {peak_inds[i]}, angle = {peaks[i]}")
    # plt.plot(angle_arr)
    # plt.plot(peak_inds, peaks, 'x')
    # plt.show()

    return peaks, peak_inds


# Function for getting the average error between OMC values and IMU values
def get_peak_and_trough_errors(OMC_peaks, IMU_peaks):
    if len(OMC_peaks) == len(IMU_peaks):
        errors = abs(OMC_peaks - IMU_peaks)
        mean_error = np.mean(errors)
    else:
        print('Warning: number of OMC peaks/troughs does not equal number of IMU peaks/troughs found')
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
        vec_x_on_ZX = [mat_x_X, mat_x_Z]
        X_on_ZX = [0, 1]
        vec_z_on_YZ = [mat_z_Y, mat_z_Z]
        Z_on_YZ = [0, 1]
        vec_y_on_XY = [mat_y_X, mat_y_Y]
        Y_on_XY = [0, 1]

        # Calculate the angle of certain CF2 vectors on certain CF1 planes
        # Discount measures of vector angle unless then 2D vector is a good/stable projection on the plane
        # i.e. If the magnitude of the vector gets close to 0, its normal to plane and so angle is unstable
        threshold = 0.7
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
    flexion = z_rel2_Z_on_YZ
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

    # Indices which define the range selected
    indmin, indmax = np.searchsorted(time, (xmin_selected, xmax_selected))  # Get the indices which represent the range
    indmax = min(len(time) - 1, indmax)

    return indmin, indmax


def get_range_dict(JA_range_dict_file, OSim_coords_joint_ref_dict, HT_joint_ref_dict, OMC_table, IMU_table,
                   OMC_angle_dict, IMU_angle_dict, time):

    if os.path.exists(JA_range_dict_file) == False:

        range_dict_coords = {'elbow_flexion': [], 'elbow_pronation': []}
        range_dict_HTs = {'HT_abd': [], 'HT_flexion': [], 'HT_rotation': []}

        # Iterate through the osim coords
        for key in range_dict_coords.keys():
            ref = OSim_coords_joint_ref_dict[key]
            OMC_angle = OMC_table.getDependentColumn(ref).to_numpy()  # Extract coordinates from states table
            IMU_angle = IMU_table.getDependentColumn(ref).to_numpy()  # Extract coordinates from states table
            indmin, indmax = run_span_selector(OMC_angle, IMU_angle, time, joint_name=key)
            range_dict_coords[key] = [indmin, indmax]

        # Iterate through the HT angles
        for key in HT_joint_ref_dict.keys():
            OMC_angle = OMC_angle_dict[key]
            IMU_angle = IMU_angle_dict[key]
            indmin, indmax = run_span_selector(OMC_angle, IMU_angle, time, joint_name=key)
            range_dict_HTs[key] = [indmin, indmax]

        # Merge the two dicts
        range_dict = {**range_dict_coords, **range_dict_HTs}

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


def plot_compare_any_JAs(OMC_angle, IMU_angle, time, start_time, end_time,
                         figure_results_dir, range_dict, joint_name):

    label = joint_name.replace('_', ' ').title()

    # For thorax rotation (heading), compare change in value from initial ref point
    if joint_name == 'thorax_rotation':
        OMC_angle = OMC_angle - OMC_angle[0]
        IMU_angle = IMU_angle - IMU_angle[0]

    # Calculate cross-correlation lag and shift IMU data
    lag = get_cross_cor_lag(OMC_angle, IMU_angle)
    if -20 < lag < 0:   # Only apply the shift if lag is in a sensible range
        IMU_angle = IMU_angle[-lag:]    # Remove first n values from IMU data
        OMC_angle = OMC_angle[:lag]     # Remove last n values from OMC data
        time = time[:lag]               # Remove last n values from time array
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => APPLIED')
    else:
        print(f' CROSS-COR: For {joint_name}, lag = {lag} => NOT APPLIED')

    # Get the peaks and troughs
    if joint_name not in ('thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation'):
        plot_peaks = True   # Don't bother calculating or plotting thorax peaks
        indmin, indmax = range_dict[joint_name][0], range_dict[joint_name][1]
        OMC_peaks, OMC_peak_inds = get_peaks_or_troughs(OMC_angle, indmin, indmax, peak_or_trough='peak', data_type='OMC')
        IMU_peaks, IMU_peak_inds = get_peaks_or_troughs(IMU_angle, indmin, indmax, peak_or_trough='peak', data_type='IMU')
        OMC_troughs, OMC_trough_inds = get_peaks_or_troughs(OMC_angle, indmin, indmax, peak_or_trough='trough', data_type='OMC')
        IMU_troughs, IMU_trough_inds = get_peaks_or_troughs(IMU_angle, indmin, indmax, peak_or_trough='trough', data_type='IMU')

        # Check we have found enough peaks/troughs
        if any(len(var) < 4 for var in [OMC_peaks, IMU_peaks, OMC_troughs, IMU_troughs]):
            print(f"WARNING: No/not enough peaks found for {joint_name} (less than 4)")

        # Get the mean peak/trough error
        mean_peak_error = get_peak_and_trough_errors(OMC_peaks, IMU_peaks)
        mean_trough_error = get_peak_and_trough_errors(OMC_troughs, IMU_troughs)

    else:
        plot_peaks = False
        mean_peak_error = 0     # Sub in value for plotting
        mean_trough_error = 0   # Sub in value for plotting

    # Get error metrics
    error_angle1 = abs(OMC_angle - IMU_angle)       # Calculate error array
    R = get_pearsonr(OMC_angle, IMU_angle)          # Calculate Pearson correlation coefficient
    RMSE_angle1 = get_RMSE(error_angle1)            # Calculate RMSE
    max_error_angle1 = np.nanmax(error_angle1)      # Calculate max error

    # Create figure
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

    # Plot the peaks and troughs
    if plot_peaks == True:
        axs[0].plot(time[OMC_trough_inds], OMC_troughs, "x", c='blue')
        axs[0].plot(time[OMC_peak_inds], OMC_peaks, "x", c='blue')
        axs[0].plot(time[IMU_trough_inds], IMU_troughs, "x", c='orange')
        axs[0].plot(time[IMU_peak_inds], IMU_peaks, "x", c='orange')

    # Annotate with mean peak/trough error
    y_min, y_max = axs[0].get_ylim()
    y_mid = (y_min + y_max) / 2
    if plot_peaks == True:
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
            print("Number of OMC peaks/troughs found did not match IMU peaks/troughs found.")
            quit()

    return RMSE_angle1, R, mean_peak_error, mean_trough_error



# Define a function to plot IMU vs OMC model body orientation errors (single angle quaternion difference)
def plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                           heading_offset, time, start_time, end_time, figure_results_dir):

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