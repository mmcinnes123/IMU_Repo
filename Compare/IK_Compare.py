# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

import matplotlib.pyplot as plt
from functions import *
import scipy

""" SETTINGS """

results_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20\Comparison1"
IMU_input_file = results_dir + r'\Right-scaled_StatesReporter_states_IMU.sto'
OMC_input_file = results_dir + r'\Right-scaled_StatesReporter_states.sto'
path_to_IMU_model_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20\IMU" + "\\" + "Calibrated_das3.osim"
path_to_OMC_model_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\23_12_20\OMC" + "\\" + "das3_scaled_and_placed.osim"
delete_last_row_of_OMC = True   # Set to True if length of OMC data doesn't match IMU data


""" MAIN """

# Define a function to plot IMU vs OMC joint angles, with extra plot of errors to see distribution
def plot_compare_JAs(joint_of_interest):

    # Read in joint angles for .mot file
    OMC_table = osim.TimeSeriesTable(OMC_input_file)
    IMU_table = osim.TimeSeriesTable(IMU_input_file)
    if delete_last_row_of_OMC == True:
        OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

    if joint_of_interest == "Thorax":
        ref1 = "/jointset/base/TH_x/value"
        ref2 = "/jointset/base/TH_z/value"
        ref3 = "/jointset/base/TH_y/value"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "Trunk Rotation"
        OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        # Update trunk rotation angle to be the change in direction based on initial direction
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    elif joint_of_interest == "Elbow":
        ref1 = "/jointset/hu/EL_x/value"
        ref2 = "/jointset/ur/PS_y/value"
        ref3 = "/jointset/ur/PS_y/value"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"
        OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()*180/np.pi

    elif joint_of_interest == "Shoulder":
        # Get HT joint angles with different function
        OMC_angle1, OMC_angle2, OMC_angle3 = get_joint_angles_from_states(OMC_input_file, path_to_OMC_model_file)
        IMU_angle1, IMU_angle2, IMU_angle3 = get_joint_angles_from_states(IMU_input_file, path_to_IMU_model_file)
        if delete_last_row_of_OMC == True:
            OMC_angle1 = OMC_angle1[:-1]
            OMC_angle2 = OMC_angle2[:-1]
            OMC_angle3 = OMC_angle3[:-1]
        label1 = "Plane of Elevation"
        label2 = "Elevation"
        label3 = "Int/Ext Rotation"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    time = OMC_table.getIndependentColumn()

    # Smooth data
    OMC_angle1 = scipy.signal.savgol_filter(OMC_angle1, 50, 3)
    OMC_angle2 = scipy.signal.savgol_filter(OMC_angle2, 50, 3)
    OMC_angle3 = scipy.signal.savgol_filter(OMC_angle3, 50, 3)
    IMU_angle1 = scipy.signal.savgol_filter(IMU_angle1, 50, 3)
    IMU_angle2 = scipy.signal.savgol_filter(IMU_angle2, 50, 3)
    IMU_angle3 = scipy.signal.savgol_filter(IMU_angle3, 50, 3)

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


def plot_compare_JAs_shoulder(joint_of_interest):

    # Read in joint angles for .mot file
    OMC_table = osim.TimeSeriesTable(OMC_input_file)
    IMU_table = osim.TimeSeriesTable(IMU_input_file)
    if delete_last_row_of_OMC == True:
        OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

    if joint_of_interest == "Thorax":
        ref1 = "/jointset/base/TH_x/value"
        ref2 = "/jointset/base/TH_z/value"
        ref3 = "/jointset/base/TH_y/value"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "Trunk Rotation"
        OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        # Update trunk rotation angle to be the change in direction based on initial direction
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    elif joint_of_interest == "Elbow":
        ref1 = "/jointset/hu/EL_x/value"
        ref2 = "/jointset/ur/PS_y/value"
        ref3 = "/jointset/ur/PS_y/value"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"
        OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()*180/np.pi
        IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()*180/np.pi
        IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()*180/np.pi
        IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()*180/np.pi

    elif joint_of_interest == "Shoulder":
        # Get HT joint angles with different function
        OMC_angle1, OMC_angle2, OMC_angle3, OMC_IER = get_joint_angles_from_states(OMC_input_file, path_to_OMC_model_file)
        IMU_angle1, IMU_angle2, IMU_angle3, IMU_IER = get_joint_angles_from_states(IMU_input_file, path_to_IMU_model_file)

        #Lets plot EIR for now:
        OMC_angle3 = OMC_IER
        IMU_angle3 = IMU_IER

        if delete_last_row_of_OMC == True:
            OMC_angle1 = OMC_angle1[:-1]
            OMC_angle2 = OMC_angle2[:-1]
            OMC_angle3 = OMC_angle3[:-1]
        label1 = "Plane of Elevation"
        label2 = "Elevation"
        label3 = "Int/Ext Rotation"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    time = OMC_table.getIndependentColumn()

    # Smooth data
    OMC_angle1 = scipy.signal.savgol_filter(OMC_angle1, 50, 3)
    OMC_angle2 = scipy.signal.savgol_filter(OMC_angle2, 50, 3)
    OMC_angle3 = scipy.signal.savgol_filter(OMC_angle3, 50, 3)
    IMU_angle1 = scipy.signal.savgol_filter(IMU_angle1, 50, 3)
    IMU_angle2 = scipy.signal.savgol_filter(IMU_angle2, 50, 3)
    IMU_angle3 = scipy.signal.savgol_filter(IMU_angle3, 50, 3)

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




# Plot IMU vs OMC joint angles
# plot_compare_JAs(joint_of_interest="Thorax")
# plot_compare_JAs(joint_of_interest="Elbow")
plot_compare_JAs_shoulder(joint_of_interest="Shoulder")




