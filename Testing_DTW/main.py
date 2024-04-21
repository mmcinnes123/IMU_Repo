
from helpers import run_IK_compare
from helpers import read_in_mot_as_numpy
from helpers import compute_euclidean_distance_matrix
from helpers import compute_accumulated_cost_matrix

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
from scipy.spatial.distance import euclidean
from fastdtw import fastdtw



# Quick Settings
subject_code = 'P3'
trial_name = 'JA_Fast'
calibration_name = 'METHOD2_Alt_self'
start_time = 15.5
end_time = 16.5
trim_bool = True
IMU_type = 'Real'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
data_dir = os.path.join(parent_dir, 'RawData')
results_dir = os.path.join(parent_dir, "Results")

# Use my functions for printing time-series JAs and calculating RMSEs and Pearson's R
# run_IK_compare(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)


# Read in same data as np arrays, ready for use in new code
OMC_angle_np, IMU_angle_np = read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

x = OMC_angle_np
y = IMU_angle_np




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

plt.show()
fig.savefig(results_dir + "\\" + "Original_data_dist_plot.png")




"""" CALCUALTE THE COST MATRIX """
#
dtw_distance, warp_path = fastdtw(x, y)
RMSE = (dtw_distance/len(warp_path))**0.5
print("DTW distance: ", dtw_distance)
# print("Warp path: ", warp_path)
print("RMSE equivalent: ", RMSE)
cost_matrix = compute_accumulated_cost_matrix(x, y)



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

plt.show()
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

plt.show()
fig.savefig(results_dir + "\\" + "Data_dist_plot_new_connections.png")
