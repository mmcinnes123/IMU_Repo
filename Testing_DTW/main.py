
from helpers import run_IK_compare
from helpers import read_in_mot_as_numpy
from helpers import compute_distance_matrix
from helpers import compute_accumulated_cost_matrix
from helpers import my_dist_func

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

prec = 0
np.set_printoptions(suppress=True,precision=prec)

# Quick Settings
subject_code = 'P3'
trial_name = 'JA_Fast'
calibration_name = 'METHOD2_Alt_self'
start_time = 15.8
end_time = 17.2
trim_bool = True
IMU_type = 'Real'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
data_dir = os.path.join(parent_dir, 'RawData')
results_dir = os.path.join(parent_dir, "Results")

# Use my functions for printing time-series JAs and calculating RMSEs and Pearson's R
run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)


# Read in same data as np arrays, ready for use in new code
OMC_angle_np, IMU_angle_np = read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

x = OMC_angle_np
y = IMU_angle_np

print(x[:10])
print(y[:10])



""" PLOT ORIGINAL DATA """
fig, ax = plt.subplots(figsize=(6, 4))

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




"""" CALCUALTE THE COST MATRIX """

# Average distance before correction:
av_DIST = np.mean(abs(x-y))
print("Average  distance before correction: ", av_DIST)

# Run the fastdtw() function
dtw_distance, warp_path = fastdtw(x, y)
print("DTW distance: ", dtw_distance)
print("Warp path: ", warp_path)

# Use custom functions to inspect results
dist_matrix = compute_distance_matrix(x, y)
cost_matrix = compute_accumulated_cost_matrix(x, y)



""" MY OWN CALCS FOR VERIFICATION """

# # Distance matrix
# print("Distance matrix: ")
# print(np.flipud(dist_matrix[:10, :10]))
#
# # Get the list of distances using the warp path
# dist_matrix_list = np.zeros((len(warp_path)))
# for i in range(len(warp_path)):
#     a = warp_path[i][0]
#     b = warp_path[i][1]
#     dist_matrix_list[i] = dist_matrix[a, b]
#
# print("Distance matrix list: ", dist_matrix_list.round(decimals=0))
# print("List mean: ", np.mean(dist_matrix_list))
#
#
# # Cost matrix
# print("Cost matrix: ")
# print(np.flipud(cost_matrix[40:50, 40:50]))
#
# # Get the list of cost matrix values using the warp path
# cost_dist_matrix_list = np.zeros((len(warp_path)))
# for i in range(len(warp_path)):
#     a = warp_path[i][0]
#     b = warp_path[i][1]
#     cost_dist_matrix_list[i] = cost_matrix[a, b]
#
# print(cost_dist_matrix_list.round(decimals=0))
#
# print("Cost matrix list: ", cost_dist_matrix_list.round(decimals=0))
# print("List mean: ", np.mean(cost_dist_matrix_list))




""" PLOT THE COST MATRIX """

fig, ax = plt.subplots(figsize=(6, 4))
ax = sbn.heatmap(cost_matrix, annot=False, square=True, cmap="YlGnBu", ax=ax)
ax.invert_yaxis()

# Get the warp path in x and y directions
path_x = [p[0] for p in warp_path]
path_y = [p[1] for p in warp_path]

# Align the path from the center of each cell
path_xx = [x+0.5 for x in path_x]
path_yy = [y+0.5 for y in path_y]

ax.plot(path_xx, path_yy, color='blue', linewidth=1, alpha=0.2)

# plt.show()
fig.savefig(results_dir + "\\" + "Cost_matrix_heat_map.png")


""" PLOT THE DATA AGAIN, BUT WITH DIFFERNET CONNECTING LINES """
fig, ax = plt.subplots(figsize=(6, 4))

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
