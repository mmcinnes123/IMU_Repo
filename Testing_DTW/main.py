
from helpers import run_IK_compare
from helpers import read_in_mot_as_numpy
from helpers import compute_distance_matrix
from helpers import compute_accumulated_cost_matrix
from helpers import my_dist_func
from helpers import run_IK_compare_new_errors

import pandas as pd
import numpy as np
import os

# Plotting Packages
import matplotlib.pyplot as plt
import seaborn as sbn

import matplotlib as mpl
mpl.rcParams['figure.dpi'] = 150
savefig_options = dict(format="png", dpi=150, bbox_inches="tight")

# Computation packages
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean


# Quick Settings
subject_code = 'P3'
trial_name = 'JA_Fast'
calibration_name = 'METHOD2_Alt_self'
start_time = 15
end_time = 21
trim_bool = True
IMU_type = 'Real'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
data_dir = os.path.join(parent_dir, 'RawData')
results_dir = os.path.join(parent_dir, "Results")
prec = 2
np.set_printoptions(suppress=True,precision=prec)


""" READ IN DATA """

# Use my functions for printing time-series JAs and calculating RMSEs and Pearson's R
# run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

# Read in same data as np arrays, ready for use in new code
OMC_angle_np, IMU_angle_np = read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

x = OMC_angle_np
y = IMU_angle_np

# Example arrays for analysis
# x = np.array([7, 1, 2, 5, 9])
# y = np.array([1, 8, 0, 4, 4, 2, 0])

print(x[:10])
print(y[:10])



""" PLOT ORIGINAL DATA """
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




"""" RUN FASTDTW AND CALCULATE COST MATRIX """

# Calculate original error metrics
av_DIST = np.mean(abs(x-y))
print("\nAverage distance before correction: ", av_DIST)
RMSE = np.mean(np.square(abs(x-y)))**0.5
print("RMSE before correction: ", RMSE)


# Run the fastdtw() function
dtw_distance, warp_path = fastdtw(x, y, dist=2)
print("DTW distance: ", dtw_distance)
print("Warp path: ", warp_path)



""" CONFIRM RESULTS WITH MANUAL FUNCTIONS """


# Use custom functions to inspect results
dist_matrix = compute_distance_matrix(x, y)
cost_matrix = compute_accumulated_cost_matrix(x, y)

# Get the list of values in the distance matrix, defined by the warp path
distance_matrix_list = [dist_matrix[j][i] for i, j in warp_path]
print("\nDistance values in the distance matrix which correspond to the path: ", distance_matrix_list)

fastdtw_dist = sum(distance_matrix_list)
print("Distance value equivalent to that returned by the fastdtw() function: ", fastdtw_dist)

RMSE_eq = np.mean(np.square(distance_matrix_list))**0.5
print("RMSE equivalent using the distance matrix values path: ", RMSE_eq)

# Use my functions for printing time-series JAs and calculating RMSEs and Pearson's R
run_IK_compare_new_errors(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type, distance_matrix_list)




# """ PLOT THE COST MATRIX """
    # note: too slow for big data (>5s)
#
# fig, ax = plt.subplots(figsize=(6, 4))
# ax = sbn.heatmap(cost_matrix, annot=True, square=True, cmap="YlGnBu", ax=ax)
# ax.invert_yaxis()
#
# # Get the warp path in x and y directions
# path_x = [p[0] for p in warp_path]
# path_y = [p[1] for p in warp_path]
#
# # Align the path from the center of each cell
# path_xx = [x+0.5 for x in path_x]
# path_yy = [y+0.5 for y in path_y]
#
# ax.plot(path_xx, path_yy, color='blue', linewidth=1, alpha=0.2)
#
# plt.show()
# fig.savefig(results_dir + "\\" + "Cost_matrix_heat_map.png")
#
#
# """ PLOT THE DISTANCE MATRIX """
#
# fig, ax = plt.subplots(figsize=(6, 4))
# ax = sbn.heatmap(dist_matrix, annot=True, square=True, cmap="YlGnBu", ax=ax)
# ax.invert_yaxis()
#
# # Get the warp path in x and y directions
# path_x = [p[0] for p in warp_path]
# path_y = [p[1] for p in warp_path]
#
# # Align the path from the center of each cell
# path_xx = [x+0.5 for x in path_x]
# path_yy = [y+0.5 for y in path_y]
#
# ax.plot(path_xx, path_yy, color='blue', linewidth=1, alpha=0.2)
#
# plt.show()
# fig.savefig(results_dir + "\\" + "Distance_matrix_heat_map.png")






""" PLOT THE DATA AGAIN, BUT WITH THE NEW CONNECTING LINES SHOWING PAIRS """

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
