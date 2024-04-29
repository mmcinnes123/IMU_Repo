import matplotlib.pyplot as plt
import os
import opensim as osim
import numpy as np
import pandas as pd
from scipy.stats import pearsonr


def my_dist_func(x_i, y_j):
    dist = abs(x_i - y_j)
    return dist

def compute_distance_matrix(x, y) -> np.array:
    """Calculate distance matrix
    This method calcualtes the pairwise Euclidean distance between two sequences.
    The sequences can have different lengths.
    """
    dist = np.zeros((len(y), len(x)))
    for i in range(len(y)):
        for j in range(len(x)):
            dist[i,j] = abs(x[j]-y[i])
    return dist


def compute_accumulated_cost_matrix(x, y) -> np.array:
    """Compute accumulated cost matrix for warp path using Euclidean distance
    """
    distances = compute_distance_matrix(x, y)

    # Initialization
    cost = np.zeros((len(y), len(x)))
    cost[0, 0] = distances[0, 0]

    for i in range(1, len(y)):
        cost[i, 0] = distances[i, 0] + cost[i - 1, 0]

    for j in range(1, len(x)):
        cost[0, j] = distances[0, j] + cost[0, j - 1]

        # Accumulated warp path cost
    for i in range(1, len(y)):
        for j in range(1, len(x)):
            cost[i, j] = min(
                cost[i - 1, j],  # insertion
                cost[i, j - 1],  # deletion
                cost[i - 1, j - 1]  # match
            ) + distances[i, j]

    return cost




# Get numpy arrays for time-series JAs from .mot file
def read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100
    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

    # Define some file names
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
    data_dir = os.path.join(parent_dir, 'RawData')
    results_dir = os.path.join(parent_dir, "Results")
    IMU_mot_file = os.path.join(data_dir, 'IMU_IK_results.mot')
    OMC_mot_file = os.path.join(data_dir, 'OMC_IK_results.mot')

    if os.path.exists(results_dir) == False:
        os.mkdir(results_dir)
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(results_dir + r"\opensim.log")


    """ MAIN """

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)

    # Set start and end time if trim_bool is false
    if trim_bool == False:
        start_time = 0
        end_time = (IMU_table.getNumRows() - 1) / sample_rate


    # Trim tables based on time of interest
    OMC_table.trim(start_time, end_time)
    IMU_table.trim(start_time, end_time)

    if OMC_table.getNumRows() == IMU_table.getNumRows() + 1:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() == IMU_table.getNumRows() + 2:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() != IMU_table.getNumRows():
        print('Tables are different sizes.')

    OMC_angle_np = OMC_table.getDependentColumn('EL_x').to_numpy()
    IMU_angle_np = IMU_table.getDependentColumn('EL_x').to_numpy()

    return OMC_angle_np, IMU_angle_np


# Read data from .mot file then plot time-series JAs
def run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100
    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

    # Define some file names
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
    data_dir = os.path.join(parent_dir, 'RawData')
    results_dir = os.path.join(parent_dir, "Results")
    IMU_mot_file = os.path.join(data_dir, 'IMU_IK_results.mot')
    OMC_mot_file = os.path.join(data_dir, 'OMC_IK_results.mot')

    if os.path.exists(results_dir) == False:
        os.mkdir(results_dir)
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(results_dir + r"\opensim.log")


    """ MAIN """

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)

    # Set start and end time if trim_bool is false
    if trim_bool == False:
        start_time = 0
        end_time = (IMU_table.getNumRows() - 1) / sample_rate


    # Trim tables based on time of interest
    OMC_table.trim(start_time, end_time)
    IMU_table.trim(start_time, end_time)

    # Account for discrepancies between trimming function/time values
    if OMC_table.getNumRows() == IMU_table.getNumRows() + 1:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() == IMU_table.getNumRows() + 2:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() != IMU_table.getNumRows():
        print('Tables are different sizes.')


    time = OMC_table.getIndependentColumn()  # Get the time data


    """ PLOT """

    # Plot IMU vs OMC joint angles based on OpenSim coordinates
    print('Plotting results...')

    RMSE_elbow_flexion, RMSE_elbow_pronation, RMSE_elbow_pronation_2, \
        R_elbow_flexion, R_elbow_pronation, R_elbow_pronation_2 = \
        plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     results_dir, labelA, labelB, joint_of_interest="Elbow")


    # "Trial Name:": str(compare_name)

    final_RMSE_values_df = pd.DataFrame.from_dict(
        {"elbow_flexion": RMSE_elbow_flexion},
        orient='index', columns=["RMSE"])

    final_R_values_df = pd.DataFrame.from_dict(
        {"elbow_flexion": R_elbow_flexion},
        orient='index', columns=["R"])

    all_data = pd.concat((final_RMSE_values_df, final_R_values_df), axis=1)

    # Write final RMSE values to a csv
    print('Writing results to .csv.')



    all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                                mode='w', encoding='utf-8', na_rep='nan')


# Define a function to plot IMU vs OMC, with extra plot of errors to see distribution, using OpenSim coords
def plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     figure_results_dir, labelA, labelB, joint_of_interest):

    if joint_of_interest == "Thorax":
        ref1 = "TH_x"
        ref2 = "TH_z"
        ref3 = "TH_y"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "(Change in) Trunk Rotation"

    elif joint_of_interest == "Elbow":
        ref1 = "EL_x"
        ref2 = "PS_y"
        ref3 = "PS_y"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    # Extract coordinates from states table
    OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()
    OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()
    OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()
    IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()
    IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()
    IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()

    # Update trunk rotation angle to be the change in direction based on initial direction
    if joint_of_interest == "Thorax":
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    # Calculate Pearson correlation coefficient
    R_1 = pearsonr(OMC_angle1, IMU_angle1)[0]
    R_2 = pearsonr(OMC_angle2, IMU_angle2)[0]
    R_3 = pearsonr(OMC_angle3, IMU_angle3)[0]

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
    fig, axs = plt.subplots(2, 1, figsize=(15,6))

    # Plot joint angles
    axs[0].scatter(time, OMC_angle1, s=0.4)
    axs[0].scatter(time, IMU_angle1, s=0.4)
    axs[0].set_title(label1)
    axs[0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
    axs[0].legend([labelA, labelB])
    axs[0].grid(color="lightgrey")


    # Plot error graphs
    axs[1].scatter(time, error_angle1, s=0.4)

    # Plot RMSE error lines and text
    axs[1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")

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
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")


    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")

    # Set a shared x axis
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + "_angles.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3


# Read data from .mot file then plot time-series JAs
def run_IK_compare_new_errors(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type, error_arr):

    print(f'\nRunning a comparison between IMU and OMC for {subject_code}, {trial_name}, calibration type: {calibration_name}')

    """ SETTINGS """
    sample_rate = 100
    labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
    labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

    # Define some file names
    compare_name = subject_code + '_' + calibration_name + '_' + trial_name
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
    data_dir = os.path.join(parent_dir, 'RawData')
    results_dir = os.path.join(parent_dir, "Results")
    IMU_mot_file = os.path.join(data_dir, 'IMU_IK_results.mot')
    OMC_mot_file = os.path.join(data_dir, 'OMC_IK_results.mot')

    if os.path.exists(results_dir) == False:
        os.mkdir(results_dir)
    osim.Logger.removeFileSink()
    osim.Logger.addFileSink(results_dir + r"\opensim.log")


    """ MAIN """

    # Read in coordinates from IK results .mot files
    print('Reading coordinates from .mot files...')
    OMC_table = osim.TimeSeriesTable(OMC_mot_file)
    IMU_table = osim.TimeSeriesTable(IMU_mot_file)

    # Set start and end time if trim_bool is false
    if trim_bool == False:
        start_time = 0
        end_time = (IMU_table.getNumRows() - 1) / sample_rate


    # Trim tables based on time of interest
    OMC_table.trim(start_time, end_time)
    IMU_table.trim(start_time, end_time)

    # Account for discrepancies between trimming function/time values
    if OMC_table.getNumRows() == IMU_table.getNumRows() + 1:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() == IMU_table.getNumRows() + 2:
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
        OMC_table.removeRowAtIndex((OMC_table.getNumRows() - 1))
    elif OMC_table.getNumRows() != IMU_table.getNumRows():
        print('Tables are different sizes.')


    time = OMC_table.getIndependentColumn()  # Get the time data


    """ PLOT """

    # Plot IMU vs OMC joint angles based on OpenSim coordinates
    print('Plotting results...')

    RMSE_elbow_flexion, RMSE_elbow_pronation, RMSE_elbow_pronation_2, \
        R_elbow_flexion, R_elbow_pronation, R_elbow_pronation_2 = \
        plot_compare_JAs_new_error(OMC_table, IMU_table, time, start_time, end_time,
                     results_dir, labelA, labelB, error_arr, joint_of_interest="Elbow")


    # "Trial Name:": str(compare_name)

    final_RMSE_values_df = pd.DataFrame.from_dict(
        {"elbow_flexion": RMSE_elbow_flexion},
        orient='index', columns=["RMSE"])

    final_R_values_df = pd.DataFrame.from_dict(
        {"elbow_flexion": R_elbow_flexion},
        orient='index', columns=["R"])

    all_data = pd.concat((final_RMSE_values_df, final_R_values_df), axis=1)

    # Write final RMSE values to a csv
    print('Writing results to .csv.')



    all_data.to_csv(results_dir + "\\" + str(compare_name) + r"_Final_RMSEs.csv",
                                mode='w', encoding='utf-8', na_rep='nan')


# Define a function to plot IMU vs OMC, with extra plot of new errors derived from DTW analysis
def plot_compare_JAs_new_error(OMC_table, IMU_table, time, start_time, end_time,
                     figure_results_dir, labelA, labelB, error_arr, joint_of_interest):

    if joint_of_interest == "Thorax":
        ref1 = "TH_x"
        ref2 = "TH_z"
        ref3 = "TH_y"
        label1 = "Forward Tilt"
        label2 = "Lateral Tilt"
        label3 = "(Change in) Trunk Rotation"

    elif joint_of_interest == "Elbow":
        ref1 = "EL_x"
        ref2 = "PS_y"
        ref3 = "PS_y"
        label1 = "Elbow Flexion"
        label2 = "Pro/Supination"
        label3 = "Pro/Supination"

    else:
        print("Joint_of_interest isn't typed correctly")
        quit()

    # Extract coordinates from states table
    OMC_angle1 = OMC_table.getDependentColumn(ref1).to_numpy()
    OMC_angle2 = OMC_table.getDependentColumn(ref2).to_numpy()
    OMC_angle3 = OMC_table.getDependentColumn(ref3).to_numpy()
    IMU_angle1 = IMU_table.getDependentColumn(ref1).to_numpy()
    IMU_angle2 = IMU_table.getDependentColumn(ref2).to_numpy()
    IMU_angle3 = IMU_table.getDependentColumn(ref3).to_numpy()

    # Update trunk rotation angle to be the change in direction based on initial direction
    if joint_of_interest == "Thorax":
        OMC_angle3 = OMC_angle3 - OMC_angle3[0]
        IMU_angle3 = IMU_angle3 - IMU_angle3[0]

    # Calculate Pearson correlation coefficient
    R_1 = pearsonr(OMC_angle1, IMU_angle1)[0]
    R_2 = pearsonr(OMC_angle2, IMU_angle2)[0]
    R_3 = pearsonr(OMC_angle3, IMU_angle3)[0]

    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)
    error_angle2 = error_arr
    new_error_time = np.linspace(start_time, end_time, num=len(error_arr))
    error_angle3 = abs(OMC_angle3 - IMU_angle3)

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1)) / len(error_angle1)) ** 0.5
    RMSE_angle2 = (sum(np.square(error_angle2)) / len(error_angle2)) ** 0.5
    RMSE_angle3 = (sum(np.square(error_angle3)) / len(error_angle3)) ** 0.5
    max_error_angle1 = np.amax(error_angle1)
    max_error_angle2 = np.amax(error_angle2)
    max_error_angle3 = np.amax(error_angle3)

    # Create figure with three subplots
    fig, axs = plt.subplots(3, 1, figsize=(15,6))

    # Plot joint angles
    axs[0].scatter(time, OMC_angle1, s=0.4)
    axs[0].scatter(time, IMU_angle1, s=0.4)
    axs[0].set_title(label1)
    axs[0].set(xlabel="Time [s]", ylabel="Joint Angle [deg]")
    axs[0].legend([labelA, labelB])
    axs[0].grid(color="lightgrey")


    # Plot error graphs
    axs[1].scatter(time, error_angle1, s=0.4)
    axs[2].scatter(new_error_time, error_angle2, s=0.4)

    # Plot RMSE error lines and text
    axs[1].axhline(y=RMSE_angle1, linewidth=2, c="red")
    axs[1].text(time[-1]+0.1*(end_time-start_time), RMSE_angle1, "RMSE = " + str(round(RMSE_angle1,1)) + " deg")
    axs[2].axhline(y=RMSE_angle2, linewidth=2, c="red")
    axs[2].text(time[-1]+0.1*(end_time-start_time), RMSE_angle2, "RMSE = " + str(round(RMSE_angle2,1)) + " deg")

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
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[2].axhline(y=y_max_line_placement_2, linewidth=1, c="red")


    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    y_max_text_placement_3 = y_max_text_placement(max_error_angle3, RMSE_angle3)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[2].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")

    # Set a shared y axis
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[1].grid(color="lightgrey")
    axs[2].set(xlabel="Time [s]", ylabel="New Error from DTW [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[2].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + joint_of_interest + "_angles_with_new_error.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, RMSE_angle3, R_1, R_2, R_3