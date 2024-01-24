# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import matplotlib.pyplot as plt
from functions import *
import scipy

""" SETTINGS """

# Quick Settings
trial_name = 'IMU_cal_pose1'    # Tag to describe this trial
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20"  # Name of the working folder
start_time = 10
end_time = 40   # If you enter same end_time as you used in IK here, OMC angles will be one too long
results_dir = parent_dir + r"\Comparison2_cal_pose_1"
create_new_ori_csvs = False

# Define some file names
IMU_states_file = results_dir + "\\" + trial_name + '_StatesReporter_states.sto'
OMC_states_file = results_dir + r'\OMC_StatesReporter_states.sto'
path_to_IMU_model_file = parent_dir + "\\" + trial_name + "\\" + "Calibrated_das3.osim"
path_to_OMC_model_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20\OMC" + "\\" + "das3_scaled_and_placed.osim"
delete_last_row_of_OMC = True   # Set to True if length of OMC data doesn't match IMU data

osim.Logger.addFileSink(results_dir + r'\opensim.log')


""" MAIN """

# Read in states for states files
OMC_table = osim.TimeSeriesTable(OMC_states_file)
IMU_table = osim.TimeSeriesTable(IMU_states_file)

# Check if they're the same length and remove last row from OMC table if not.
if OMC_table.getNumRows() != IMU_table.getNumRows():
    OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

# Calculate body orientations from states table for full recording period and write to csv file
if create_new_ori_csvs == True:
    extract_body_quats(OMC_table, path_to_OMC_model_file, results_dir, tag="OMC")
    extract_body_quats(IMU_table, path_to_IMU_model_file, results_dir, tag="IMU")

# Read in body orientations from newly created csv files (as trimmed np arrays)
thorax_OMC, humerus_OMC, radius_OMC = read_in_quats(start_time, end_time, file_name=results_dir + r"\OMC_quats.csv")
thorax_IMU, humerus_IMU, radius_IMU = read_in_quats(start_time, end_time, file_name=results_dir + r"\IMU_quats.csv")

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
        label3 = "Trunk Rotation"

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
    OMC_angle1 = scipy.signal.savgol_filter(OMC_angle1, window_length, polynomial)
    OMC_angle2 = scipy.signal.savgol_filter(OMC_angle2, window_length, polynomial)
    OMC_angle3 = scipy.signal.savgol_filter(OMC_angle3, window_length, polynomial)
    IMU_angle1 = scipy.signal.savgol_filter(IMU_angle1, window_length, polynomial)
    IMU_angle2 = scipy.signal.savgol_filter(IMU_angle2, window_length, polynomial)
    IMU_angle3 = scipy.signal.savgol_filter(IMU_angle3, window_length, polynomial)

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
        axs[i,0].legend(["OMC", "IMU"])

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1, s=0.4)
    axs[1,1].scatter(time, error_angle2, s=0.4)
    axs[2,1].scatter(time, error_angle3, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+3, RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+3, RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+3, RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

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
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0,1].text(time[-1]+3, y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+3, y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+3, y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 3):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,40))

    fig.tight_layout(pad=2.0)

    fig.savefig(results_dir + "\\" + joint_of_interest + "_angles.png")

    # plt.show()



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
        axs[i,0].legend(["OMC", "IMU"])

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1_with_nans, s=0.4)
    axs[1,1].scatter(time, error_angle2_with_nans, s=0.4)
    axs[2,1].scatter(time, error_angle3_with_nans, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+3, RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+3, RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+3, RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")

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
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[0,1].text(time[-1]+3, y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+3, y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+3, y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 3):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,40), xlim=(start_time, end_time))

    fig.tight_layout(pad=2.0)

    fig.savefig(results_dir + "\\" + joint_of_interest + "_angles.png")


def plot_compare_JAs_shoulder(joint_of_interest):


    # Get HT joint angles with different function
    OMC_angle1, OMC_angle2, OMC_angle3, OMC_IER_x, IMU_angle1, IMU_angle2, IMU_angle3, IMU_IER, OMC_IER_z, OMC_IER_y = \
        get_joint_angles_from_states(OMC_states_file, path_to_OMC_model_file, start_time, end_time)
    # IMU_angle1, IMU_angle2, IMU_angle3, IMU_IER = \
    #     get_joint_angles_from_states(IMU_states_file, path_to_IMU_model_file, start_time, end_time)
        # TODO: Tidy lines above - create new function specifically for calculating vector angles

    # Algorthm to determine IER
    n_rows = len(OMC_angle1)
    OMC_IER = np.zeros((n_rows))
    for row in range(n_rows):
        if OMC_angle2[row] < 45 or OMC_angle2[row] > 135:    # If elevation is less than 45 deg, or above 135 deg, use x axis angle
            OMC_IER[row] = OMC_IER_x[row]
        else:   # Else, (if elevation is between 45 and 135 deg)
            if 45 < OMC_angle1[row] < 135:   # And if arm is in flexion, keep using x axis angle
                OMC_IER[row] = OMC_IER_x[row]
            else:    # But if arm is in abduction, use z axis angle instead
                OMC_IER[row] = OMC_IER_z[row]
        # TODO: Add functionality to algorithm so it will work when int/ext happens during flex or abduction
        #   Try using x on x-y plane



    # Discount vector angle when elevation is above cutoff
    # OMC_IER_x = np.where(OMC_angle2 < 45, OMC_IER_x, np.nan)
    # IMU_IER = np.where(OMC_angle2 < 45, IMU_IER, np.nan)
        # TODO: Decide when and how I want to discount data

    label1 = "Euler Y - Plane of Elevation"
    label2 = "Euler Z - Elevation"
    label3 = "Euler YY - Int/Ext Rotation"
    label4 = "Vector Angle - Int/Ext Rotation"


    # Smooth data
    # OMC_angle1 = scipy.signal.savgol_filter(OMC_angle1, 50, 3)
    # OMC_angle2 = scipy.signal.savgol_filter(OMC_angle2, 50, 3)
    # OMC_angle3 = scipy.signal.savgol_filter(OMC_angle3, 50, 3)
    # IMU_angle1 = scipy.signal.savgol_filter(IMU_angle1, 50, 3)
    # IMU_angle2 = scipy.signal.savgol_filter(IMU_angle2, 50, 3)
    # IMU_angle3 = scipy.signal.savgol_filter(IMU_angle3, 50, 3)
    # OMC_IER_x = scipy.signal.savgol_filter(OMC_IER_x, 50, 3)
    # IMU_IER = scipy.signal.savgol_filter(IMU_IER, 50, 3)

    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)
    error_angle2 = abs(OMC_angle2 - IMU_angle2)
    error_angle3 = abs(OMC_angle3 - IMU_angle3)
    error_IER = abs(OMC_IER_x - IMU_IER)

    # Remove nan values from error arrays
    error_angle1_no_nans = error_angle1[np.logical_not(np.isnan(error_angle1))]
    error_angle2_no_nans = error_angle2[np.logical_not(np.isnan(error_angle2))]
    error_angle3_no_nans = error_angle3[np.logical_not(np.isnan(error_angle3))]
    error_IER_no_nans = error_IER[np.logical_not(np.isnan(error_IER))]

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1_no_nans)) / len(error_angle1_no_nans)) ** 0.5
    RMSE_angle2 = (sum(np.square(error_angle2_no_nans)) / len(error_angle2_no_nans)) ** 0.5
    RMSE_angle3 = (sum(np.square(error_angle3_no_nans)) / len(error_angle3_no_nans)) ** 0.5
    RMSE_IER = (sum(np.square(error_IER_no_nans)) / len(error_IER_no_nans)) ** 0.5
    max_error_angle1 = np.amax(error_angle1_no_nans)
    max_error_angle2 = np.amax(error_angle2_no_nans)
    max_error_angle3 = np.amax(error_angle3_no_nans)
    max_error_IER = np.amax(error_IER_no_nans)

    # Create figure with four subplots
    fig, axs = plt.subplots(4, 2, figsize=(14,9), width_ratios=[9,1])

    # Plot joint angles

    axs[0,0].plot(time, OMC_angle1)
    axs[0,0].plot(time, IMU_angle1)

    axs[1,0].plot(time, OMC_angle2)
    axs[1,0].plot(time, IMU_angle2)

    axs[2,0].plot(time, OMC_angle3)
    axs[2,0].plot(time, IMU_angle3)

    axs[3,0].plot(time, OMC_IER_x)
    axs[3,0].plot(time, OMC_IER_z)
    axs[3,0].plot(time, -OMC_angle3)

    axs[0,0].set_title(label1)
    axs[1,0].set_title(label2)
    axs[2,0].set_title(label3)
    axs[3,0].set_title(label4)

    for i in range(0, 4):
        axs[i,0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]", xlim=(time[0], time[-1]))

    for i in range(0, 3):
        axs[i,0].legend(["OMC", "IMU"])

    axs[3,0].legend(["IER_x", "IER_z", "Eul3"])

    # Plot error graphs

    axs[0,1].scatter(time, error_angle1, s=0.4)
    axs[1,1].scatter(time, error_angle2, s=0.4)
    axs[2,1].scatter(time, error_angle3, s=0.4)
    axs[3,1].scatter(time, error_IER, s=0.4)

    # Plot RMSE error lines and text
    axs[0,1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[0,1].text(time[-1]+3, RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[1,1].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[1,1].text(time[-1]+3, RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")
    axs[2,1].axhline(y=RMSE_angle3, linewidth=2, c="red")
    axs[2,1].text(time[-1]+3, RMSE_angle3, "RMSE = " + str(round(RMSE_angle3,1)) + " deg")
    axs[3,1].axhline(y=RMSE_IER, linewidth=2, c="red")
    axs[3,1].text(time[-1]+3, RMSE_IER, "RMSE = " + str(round(RMSE_IER,1)) + " deg")

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
    y_max_line_placement_4 = y_max_line_placement(max_error_IER)
    axs[0,1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[1,1].axhline(y=y_max_line_placement_2, linewidth=1, c="red")
    axs[2,1].axhline(y=y_max_line_placement_3, linewidth=1, c="red")
    axs[3,1].axhline(y=y_max_line_placement_4, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    y_max_text_placement_4 = y_max_text_placement(max_error_IER, RMSE_IER)
    axs[0,1].text(time[-1]+3, y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[1,1].text(time[-1]+3, y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")
    axs[2,1].text(time[-1]+3, y_max_text_placement_3, "Max = " + str(round(max_error_angle3,1)) + " deg")
    axs[3,1].text(time[-1]+3, y_max_text_placement_4, "Max = " + str(round(max_error_IER,1)) + " deg")

    # Set a shared x axis
    for i in range(0, 4):
        axs[i,1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,40), xlim=(time[0], time[-1]))

    fig.tight_layout(pad=2.0)

    fig.savefig(results_dir + "\\" + joint_of_interest + "_angles.png")

    # plt.show()




# Plot IMU vs OMC joint angles based on OpenSim coordinates
# plot_compare_JAs(joint_of_interest="Thorax")
# plot_compare_JAs(joint_of_interest="Elbow")
plot_compare_JAs_shoulder_eulers(joint_of_interest="GH_Eulers")


# plot_compare_JAs_shoulder(joint_of_interest="Shoulder")


