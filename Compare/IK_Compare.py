# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

# Test change made in Master

import matplotlib.pyplot as plt
import numpy as np
import os
from functions import *
import scipy

""" SETTINGS """

# Quick Settings
trial_name = 'IMU_IMU_combined_cal'    # Tag to describe this trial
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_01_22"  # Name of the working folder
start_time = 0
end_time = 37
results_dir = parent_dir + r"\Comparison12_combined_cal_IMUIMU"
create_new_ori_csvs = True     # Set this to False if you've already run this code and csv file has been created
labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

# Define some file names
IMU_states_file = results_dir + "\\" + trial_name + '_StatesReporter_states.sto'
OMC_states_file = results_dir + r'\OMC_StatesReporter_states.sto'
path_to_IMU_model_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\das3.osim"
path_to_OMC_model_file = parent_dir + r"\OMC\das3_scaled_and_placed.osim"
figure_results_dir = results_dir + "\\TimeRange_" + str(start_time) + "_" + str(end_time) + "s"
if os.path.exists(figure_results_dir) == False:
    os.mkdir(figure_results_dir)
osim.Logger.addFileSink(results_dir + r'\opensim.log')


""" MAIN """

# Read in states for states files
OMC_table = osim.TimeSeriesTable(OMC_states_file)
IMU_table = osim.TimeSeriesTable(IMU_states_file)
print(OMC_table.getNumRows())
print(IMU_table.getNumRows())

# Check if they're the same length and remove last row from OMC table if not.
if OMC_table.getNumRows() != IMU_table.getNumRows():
    OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

# Calculate body orientations from states table for full recording period and write to csv file
if create_new_ori_csvs == True:
    extract_body_quats(OMC_table, path_to_OMC_model_file, results_dir, tag="OMC")
    extract_body_quats(IMU_table, path_to_IMU_model_file, results_dir, tag="IMU")


# Find the heading offset between IMU model thorax and OMC model thorax (as a descriptor of global frame offset) (read in untrimmed data)
thorax_OMC_all, humerus_OMC_all, radius_OMC_all = read_in_quats(start_time, end_time, file_name=results_dir + r"\OMC_quats.csv", trim_bool=False)
thorax_IMU_all, humerus_IMU_all, radius_IMU_all = read_in_quats(start_time, end_time, file_name=results_dir + r"\IMU_quats.csv", trim_bool=False)
heading_offset = find_heading_offset(thorax_OMC_all, thorax_IMU_all)

# Read in body orientations from newly created csv files (as trimmed np arrays (Nx4))
thorax_OMC, humerus_OMC, radius_OMC = read_in_quats(start_time, end_time, file_name=results_dir + r"\OMC_quats.csv", trim_bool=True)
thorax_IMU, humerus_IMU, radius_IMU = read_in_quats(start_time, end_time, file_name=results_dir + r"\IMU_quats.csv", trim_bool=True)

# Trim tables based on time of interest
OMC_table.trim(start_time, end_time)
IMU_table.trim(start_time, end_time)

time = OMC_table.getIndependentColumn()  # Get the time data


# Define a function to plot IMU vs OMC, with extra plot of errors to see distribution, using OpenSim coords
def plot_compare_JAs(joint_of_interest):

    if joint_of_interest == "Thorax":
        ref1 = "/jointset/base/TH_x/value"
        ref2 = "/jointset/base/TH_z/value"
        ref3 = "/jointset/base/TH_y/value"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "(Change in) Trunk Rotation"

    elif joint_of_interest == "Elbow":
        ref1 = "/jointset/hu/EL_x/value"
        ref2 = "/jointset/ur/PS_y/value"
        ref3 = "/jointset/ur/PS_y/value"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    # Extract coordinates from states table
    OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy() * 180 / np.pi
    OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy() * 180 / np.pi
    OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy() * 180 / np.pi
    IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy() * 180 / np.pi
    IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy() * 180 / np.pi
    IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy() * 180 / np.pi

    # Update trunk rotation angle to be the change in direction based on initial direction
    if joint_of_interest == "Thorax":
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    # Smooth data
    window_length = 20
    polynomial = 3
    # OMC_angle1 = scipy.signal.savgol_filter(OMC_angle1, window_length, polynomial)
    # OMC_angle2 = scipy.signal.savgol_filter(OMC_angle2, window_length, polynomial)
    # OMC_angle3 = scipy.signal.savgol_filter(OMC_angle3, window_length, polynomial)
    # IMU_angle1 = scipy.signal.savgol_filter(IMU_angle1, window_length, polynomial)
    # IMU_angle2 = scipy.signal.savgol_filter(IMU_angle2, window_length, polynomial)
    # IMU_angle3 = scipy.signal.savgol_filter(IMU_angle3, window_length, polynomial)

    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)
    error_angle2 = abs(OMC_angle2 - IMU_angle2)
    error_angle3 = abs(OMC_angle3 - IMU_angle3)

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1)) / len(error_angle1)) ** 0.5
    RMSE_angle2 = (sum(np.square(error_angle2)) / len(error_angle2)) ** 0.5
    RMSE_angle3 = (sum(np.square(error_angle3)) / len(error_angle3)) ** 0.5
    max_error_angle1 = np.amax(error_angle1)
    max_error_angle2 = np.amax(error_angle2)
    max_error_angle3 = np.amax(error_angle3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    axs[0,0].plot(time, OMC_angle1)
    axs[0,0].plot(time, IMU_angle1)

    axs[1,0].plot(time, OMC_angle2)
    axs[1,0].plot(time, IMU_angle2)

    axs[2,0].plot(time, OMC_angle3)
    axs[2,0].plot(time, IMU_angle3)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)

    for i in range(0, 3):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([labelA, labelB])
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
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + "_angles.png")



# Define a function to plot IMU vs OMC for the shoulder joint euler anlges
def plot_compare_JAs_shoulder_eulers(joint_of_interest):

    label1 = "Plane of Elevation (Y)"
    label2 = "Elevation (Z)"
    label3 = "Internal/External Rotation (Y)"

    OMC_angle1_all, OMC_angle2, OMC_angle3_all = get_JA_euls_from_quats(thorax_OMC, humerus_OMC, eul_seq="YZY")
    IMU_angle1_all, IMU_angle2, IMU_angle3_all = get_JA_euls_from_quats(thorax_IMU, humerus_IMU, eul_seq="YZY")

    # Discount 1st and 3rd euler if 2nd euler (elevation) is below threshold
    elevation_threshold = 45
    OMC_angle1 = np.where(OMC_angle2 > elevation_threshold, OMC_angle1_all, np.nan)
    OMC_angle3 = np.where(OMC_angle2 > elevation_threshold, OMC_angle3_all, np.nan)
    IMU_angle1 = np.where(OMC_angle2 > elevation_threshold, IMU_angle1_all, np.nan)
    IMU_angle3 = np.where(OMC_angle2 > elevation_threshold, IMU_angle3_all, np.nan)

    # Calculate error arrays
    error_angle1_with_nans = abs(OMC_angle1 - IMU_angle1)
    error_angle2_with_nans = abs(OMC_angle2 - IMU_angle2)
    error_angle3_with_nans = abs(OMC_angle3 - IMU_angle3)
    # Remove any rows where there's nan values
    error_angle1 = error_angle1_with_nans[~np.isnan(error_angle1_with_nans)]
    error_angle2 = error_angle2_with_nans[~np.isnan(error_angle2_with_nans)]
    error_angle3 = error_angle3_with_nans[~np.isnan(error_angle3_with_nans)]

    # Calculate RMSE
    RMSE_angle1 = find_RMSE_of_error_array(error_angle1)
    RMSE_angle2 = find_RMSE_of_error_array(error_angle2)
    RMSE_angle3 = find_RMSE_of_error_array(error_angle3)
    max_error_angle1 = find_max_in_error_array(error_angle1)
    max_error_angle2 = find_max_in_error_array(error_angle2)
    max_error_angle3 = find_max_in_error_array(error_angle3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    axs[0,0].plot(time, OMC_angle1)
    axs[0,0].plot(time, IMU_angle1)
    axs[0,0].plot(time, OMC_angle1_all, linestyle='dotted', c='lightgrey')
    axs[0,0].plot(time, IMU_angle1_all, linestyle='dotted', c='lightgrey')

    axs[1,0].plot(time, OMC_angle2)
    axs[1,0].plot(time, IMU_angle2)

    axs[2,0].plot(time, OMC_angle3)
    axs[2,0].plot(time, IMU_angle3)
    axs[2,0].plot(time, OMC_angle3_all, linestyle='dotted', c='lightgrey')
    axs[2,0].plot(time, IMU_angle3_all, linestyle='dotted', c='lightgrey')

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)

    for i in range(0, 3):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([labelA, labelB])
        axs[i,0].grid(color="lightgrey")

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1_with_nans, s=0.4)
    axs[1,1].scatter(time, error_angle2_with_nans, s=0.4)
    axs[2,1].scatter(time, error_angle3_with_nans, s=0.4)

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
        elif max_error < (RMSE * 1.1):
            text_placement = RMSE * 1.1
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



# Define a function to plot IMU vs OMC model body orientation errors (single angle quaternion difference)
def plot_compare_body_oris(joint_of_interest):

    label1 = "Thorax Orientation Error" + " (heading offset applied: " + str(round(heading_offset*180/np.pi,1)) + "deg)"
    label2 = "Humerus Orientation Error"
    label3 = "Radius Orientation Error"

    # Apply heading offset to all IMU bodies
    heading_offset_quat = np.array([np.cos(heading_offset/2), 0, np.sin(heading_offset/2), 0])
    for row in range(len(thorax_IMU)):
        thorax_IMU[row] = quat_mul(thorax_IMU[row], heading_offset_quat)
        humerus_IMU[row] = quat_mul(humerus_IMU[row], heading_offset_quat)
        radius_IMU[row] = quat_mul(radius_IMU[row], heading_offset_quat)

    def find_single_angle_diff_between_two_CFs(body1, body2):
        n_rows = len(body1)
        angle_arr = np.zeros((n_rows))
        for row in range(n_rows):
            quat_diff = quat_mul(quat_conj(body1[row]), body2[row])
            angle_arr[row] = 2 * np.arccos(abs(quat_diff[0])) * 180 / np.pi
        return angle_arr

    thorax_ori_error = find_single_angle_diff_between_two_CFs(thorax_OMC, thorax_IMU)
    humerus_ori_error = find_single_angle_diff_between_two_CFs(humerus_OMC, humerus_IMU)
    radius_ori_error = find_single_angle_diff_between_two_CFs(radius_OMC, radius_IMU)

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

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + ".png")



# Define a function to plot IMU vs OMC HT angles, based on projected vector directions
def plot_vector_HT_angles(joint_of_interest):

    label1 = "Abduction (x_rel2_X_on_XY)"
    label2 = "Flexion (z_rel2_Z_on_ZY)"
    label3 = "Rotation - Elbow Down (x_rel2_X_on_XZ)"
    label4 = "Rotation - Elbow Up (z_rel2_Z_on_ZY)"

    # Calculate HT Euler angles (to be used as a reference)
    OMC_angle1_all, OMC_angle2_all, OMC_angle3_all = get_JA_euls_from_quats(thorax_OMC, humerus_OMC, eul_seq="YZY")
    IMU_angle1_all, IMU_angle2_all, IMU_angle3_all = get_JA_euls_from_quats(thorax_IMU, humerus_IMU, eul_seq="YZY")

    # Calculate the projected vector angles based on the body orientations of thorax and humerus
    abduction_all_OMC, flexion_all_OMC, rotation_elbow_down_all_OMC, rotation_elbow_up_all_OMC = \
        get_vec_angles_from_two_CFs(thorax_OMC, humerus_OMC)
    abduction_all_IMU, flexion_all_IMU, rotation_elbow_down_all_IMU, rotation_elbow_up_all_IMU = \
        get_vec_angles_from_two_CFs(thorax_IMU, humerus_IMU)

    # Trim the arrays above based on criteria to avoid singularities and only focus on angles of interest
    abduction_OMC, flexion_OMC, rotation_elbow_down_OMC, rotation_elbow_up_OMC = \
        trim_vec_prof_angles(abduction_all_OMC, flexion_all_OMC, rotation_elbow_down_all_OMC, rotation_elbow_up_all_OMC,
                             OMC_angle1_all, OMC_angle2_all)
    abduction_IMU, flexion_IMU, rotation_elbow_down_IMU, rotation_elbow_up_IMU = \
        trim_vec_prof_angles(abduction_all_IMU, flexion_all_IMU, rotation_elbow_down_all_IMU, rotation_elbow_up_all_IMU,
                             IMU_angle1_all, IMU_angle2_all)

    # Calculate error arrays
    error_angle1_with_nans = abs(abduction_OMC - abduction_IMU)
    error_angle2_with_nans = abs(flexion_OMC - flexion_IMU)
    error_angle3_with_nans = abs(rotation_elbow_down_OMC - rotation_elbow_down_IMU)
    error_angle4_with_nans = abs(rotation_elbow_up_OMC - rotation_elbow_up_IMU)

    # Remove any rows where there's nan values
    error_angle1 = error_angle1_with_nans[~np.isnan(error_angle1_with_nans)]
    error_angle2 = error_angle2_with_nans[~np.isnan(error_angle2_with_nans)]
    error_angle3 = error_angle3_with_nans[~np.isnan(error_angle3_with_nans)]
    error_angle4 = error_angle4_with_nans[~np.isnan(error_angle4_with_nans)]

    # Calculate RMSE
    RMSE_angle1 = find_RMSE_of_error_array(error_angle1)
    RMSE_angle2 = find_RMSE_of_error_array(error_angle2)
    RMSE_angle3 = find_RMSE_of_error_array(error_angle3)
    RMSE_angle4 = find_RMSE_of_error_array(error_angle4)
    max_error_angle1 = find_max_in_error_array(error_angle1)
    max_error_angle2 = find_max_in_error_array(error_angle2)
    max_error_angle3 = find_max_in_error_array(error_angle3)
    max_error_angle4 = find_max_in_error_array(error_angle4)

    # Create figure with three subplots
    fig, axs = plt.subplots(4, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    line1, = axs[0,0].plot(time, abduction_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[0,0].plot(time, abduction_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[0,0].plot(time, abduction_OMC)
    line4, = axs[0,0].plot(time, abduction_IMU)

    line1, = axs[1,0].plot(time, flexion_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[1,0].plot(time, flexion_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[1,0].plot(time, flexion_OMC)
    line4, = axs[1,0].plot(time, flexion_IMU)

    line1, = axs[2,0].plot(time, rotation_elbow_down_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[2,0].plot(time, rotation_elbow_down_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[2,0].plot(time, rotation_elbow_down_OMC)
    line4, = axs[2,0].plot(time, rotation_elbow_down_IMU)

    line1, = axs[3,0].plot(time, rotation_elbow_up_all_OMC, linestyle='dotted', c='lightgrey')
    line2, = axs[3,0].plot(time, rotation_elbow_up_all_IMU, linestyle='dotted', c='lightgrey')
    line3, = axs[3,0].plot(time, rotation_elbow_up_OMC)
    line4, = axs[3,0].plot(time, rotation_elbow_up_IMU)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)
    axs[3,0].set_title(label4)

    for i in range(0, 4):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
        axs[i,0].legend([line3, line4], [labelA, labelB])
        axs[i,0].grid(color="lightgrey")

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1_with_nans, s=0.4)
    axs[1,1].scatter(time, error_angle2_with_nans, s=0.4)
    axs[2,1].scatter(time, error_angle3_with_nans, s=0.4)
    axs[3,1].scatter(time, error_angle4_with_nans, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")
    axs[3,1].axhline(y=RMSE_angle4, linewidth=2, c="red")
    axs[3,1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle4, "RMSE = " + str(round(RMSE_angle4,1)) + " deg")

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
            text_placement = RMSE + 3
        else:
            text_placement = max_error
        return text_placement

    # Plot max error lines
    y_max_line_placement_1 = y_max_line_placement(max_error_angle1)
    y_max_line_placement_2 = y_max_line_placement(max_error_angle2)
    y_max_line_placement_3 = y_max_line_placement(max_error_angle3)
    y_max_line_placement_4 = y_max_line_placement(max_error_angle4)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")
    axs[3,1].axhline(y=y_max_line_placement_4, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    y_max_text_placement_4 = y_max_text_placement(max_error_angle4, RMSE_angle4)
    axs[0,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")
    axs[3,1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_4, "Max = " + str(round(max_error_angle4,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 4):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1, max_error_angle2, max_error_angle3])])), xlim=(start_time, end_time))
        axs[i,1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + ".png")




# Plot IMU vs OMC joint angles based on OpenSim coordinates
plot_compare_JAs(joint_of_interest="Thorax")
plot_compare_JAs(joint_of_interest="Elbow")
plot_compare_JAs_shoulder_eulers(joint_of_interest="HT_Eulers")
plot_compare_body_oris(joint_of_interest="Body_Orientation_Diff")
plot_vector_HT_angles(joint_of_interest="HT_Vectors")