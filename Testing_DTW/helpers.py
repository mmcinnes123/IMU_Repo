# Helper functions for DTW analysis


import os
import opensim as osim
import numpy as np
from scipy.stats import pearsonr
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sbn

mpl.rcParams['figure.dpi'] = 150


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

    time = OMC_table.getIndependentColumn()  # Get the time data

    return OMC_angle_np, IMU_angle_np, time



# A function to plot IMU vs OMC from np arrays, with extra plot of errors, with matching time axis
def plot_compare_JAs(OMC_angle1, IMU_angle1, time, start_time, end_time,
                     figure_results_dir):


    labelA = 'OMC'
    labelB = 'IMU'
    label1 = "Elbow Flexion"


    # Calculate Pearson correlation coefficient
    R_1 = pearsonr(OMC_angle1, IMU_angle1)[0]

    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1)) / len(error_angle1)) ** 0.5
    max_error_angle1 = np.amax(error_angle1)


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
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")

    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")

    # Set a shared x axis
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[1].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + "Elbow_angles.png")

    plt.close()

    return RMSE_angle1, R_1



# A function to plot two time series data, x and y, with vertical paths showing how each point is connected
def plot_data(x, y, results_dir):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Remove the border and axes ticks
    fig.patch.set_visible(False)
    ax.axis('off')

    xx = [(i, x[i]) for i in np.arange(0, len(x))]
    yy = [(j, y[j]) for j in np.arange(0, len(y))]

    for i, j in zip(xx, yy[:-2]):
        ax.plot([i[0], j[0]], [i[1], j[1]], '--k', linewidth=0.3)

    ax.plot(x, '-ro', label='x', linewidth=1, markersize=1, markerfacecolor='lightcoral', markeredgecolor='lightcoral')
    ax.plot(y, '-bo', label='y', linewidth=1, markersize=1, markerfacecolor='skyblue', markeredgecolor='skyblue')
    ax.set_title("Euclidean Distance", fontsize=10, fontweight="bold")

    # plt.show()
    fig.savefig(results_dir + "\\" + "Original_data_dist_plot.png")




# A function to plot IMU vs OMC from np arrays, with extra plot of errors, with matching time axis
def plot_compare_JAs_with_new_errors(OMC_angle1, IMU_angle1, new_error_arr, time, start_time, end_time,
                     figure_results_dir):


    labelA = 'OMC'
    labelB = 'IMU'
    label1 = "Elbow Flexion"


    # Calculate Pearson correlation coefficient
    R_1 = pearsonr(OMC_angle1, IMU_angle1)[0]


    # Calculate error arrays
    error_angle1 = abs(OMC_angle1 - IMU_angle1)
    error_angle2 = new_error_arr
    new_error_time = np.linspace(start_time, end_time, num=len(new_error_arr))

    # Calculate RMSE
    RMSE_angle1 = (sum(np.square(error_angle1)) / len(error_angle1)) ** 0.5
    RMSE_angle2 = (sum(np.square(error_angle2)) / len(error_angle2)) ** 0.5
    max_error_angle1 = np.amax(error_angle1)
    max_error_angle2 = np.amax(error_angle2)

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
    axs[1].axhline(y=y_max_line_placement_1, linewidth=1, c="red")
    axs[2].axhline(y=y_max_line_placement_2, linewidth=1, c="red")


    # Plot max error text
    y_max_text_placement_1 = y_max_text_placement(max_error_angle1, RMSE_angle1)
    y_max_text_placement_2 = y_max_text_placement(max_error_angle2, RMSE_angle2)
    axs[1].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_1, "Max = " + str(round(max_error_angle1,1)) + " deg")
    axs[2].text(time[-1]+0.1*(end_time-start_time), y_max_text_placement_2, "Max = " + str(round(max_error_angle2,1)) + " deg")

    # Set a shared y axis
    axs[1].set(xlabel="Time [s]", ylabel="IMU Error [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[1].grid(color="lightgrey")
    axs[2].set(xlabel="Time [s]", ylabel="New Error from DTW [deg]", ylim=(0,np.min([40,1.1*np.max([max_error_angle1])])))
    axs[2].grid(color="lightgrey")

    fig.tight_layout(pad=2.0)

    fig.savefig(figure_results_dir + "\\" + "Elbow_angles_with_new_error.png")

    plt.close()

    return RMSE_angle1, RMSE_angle2, R_1


# Function from online source for computing distance matrix (just the abs difference between every value in each array)
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


# Function from online source for computing cost matrix (see source for definition)
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


# Plot a heatmap of the cost matrix, showing the warp path
def plot_cost_matrix(cost_matrix, warp_path, results_dir):
    fig, ax = plt.subplots(figsize=(6, 4))
    ax = sbn.heatmap(cost_matrix, annot=False, square=True, cmap="YlGnBu", ax=ax)
    ax.invert_yaxis()

    # Get the warp path in x and y directions
    path_x = [p[0] for p in warp_path]
    path_y = [p[1] for p in warp_path]

    # Align the path from the center of each cell
    path_xx = [x + 0.5 for x in path_x]
    path_yy = [y + 0.5 for y in path_y]

    ax.plot(path_xx, path_yy, color='blue', linewidth=1, alpha=0.2)

    # plt.show()
    fig.savefig(results_dir + "\\" + "Cost_matrix_heat_map.png")


# Plot a heatmap of the distance matrix, showing the warp path (these values are used to replace original RMSE calc)
def plot_distance_matrix(dist_matrix, warp_path, results_dir):
    fig, ax = plt.subplots(figsize=(6, 4))
    ax = sbn.heatmap(dist_matrix, annot=False, square=True, cmap="YlGnBu", ax=ax)
    ax.invert_yaxis()

    # Get the warp path in x and y directions
    path_x = [p[0] for p in warp_path]
    path_y = [p[1] for p in warp_path]

    # Align the path from the center of each cell
    path_xx = [x + 0.5 for x in path_x]
    path_yy = [y + 0.5 for y in path_y]

    ax.plot(path_xx, path_yy, color='blue', linewidth=1, alpha=0.2)

    # plt.show()
    fig.savefig(results_dir + "\\" + "Distance_matrix_heat_map.png")


# Plot the time series data again, but showing the new connections defined by the warp path
def plot_data_with_new_paths(x, y, warp_path, results_dir):
    fig, ax = plt.subplots(figsize=(10, 6))

    # Remove the border and axes ticks
    fig.patch.set_visible(False)
    ax.axis('off')

    for [map_x, map_y] in warp_path:
        ax.plot([map_x, map_y], [x[map_x], y[map_y]], '--k', linewidth=0.2)

    ax.plot(x, '-ro', label='x', linewidth=1, markersize=1, markerfacecolor='lightcoral', markeredgecolor='lightcoral')
    ax.plot(y, '-bo', label='y', linewidth=1, markersize=1, markerfacecolor='skyblue', markeredgecolor='skyblue')

    ax.set_title("DTW Distance", fontsize=10, fontweight="bold")

    # plt.show()
    fig.savefig(results_dir + "\\" + "Data_dist_plot_new_connections.png")







