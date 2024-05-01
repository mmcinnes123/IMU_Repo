"""
The following is the application of a dynamic time warp analysis of a small set of example data.
Most of the theory and the code is from the following article:
    https://www.theaidream.com/post/dynamic-time-warping-dtw-algorithm-in-time-series#:~:text=In%20time%20series%20analysis%2C%20Dynamic,similar%20elements%20between%20time%20series.

Input is .mot OpenSim coordinate data, which is found based on trial_name, subject_code etc.
Output is some plots of the intermediate calcs, and some new error metrics, which theoretically could replace RMSE.

This analysis was deemed unsuitable for this application because, although it does reduce the error contribution from
the 'moving' phases of the data, it reduces the error in these sections to almost zero, which still isn't a realistic
description of the actual IMU error.

*** Note - there is a small problem with the code provided in the article - they use 'euclidean distance' as their
distance function, imported from scipy, but the shape of the input/output arrays didn't work with the fastdtw() function.
In this code, abs(x-y) is used as the distance metric instead. This is written in the manual function
compute_accumulated_cost_matrix, and in the fastdtw() function, abs(x-y) is the default.

Other useful articles:
    GitHub code for fastdtw() function: https://github.com/slaypni/fastdtw/blob/v0.3.4/fastdtw/fastdtw.py#L30
    https://medium.com/walmartglobaltech/time-series-similarity-using-dynamic-time-warping-explained-9d09119e48ec
    https://rtavenar.github.io/blog/dtw.html
Helpful forum post:
    https://stackoverflow.com/questions/77277096/error-in-calculating-dynamic-time-warping
Paper from the function:
    https://cs.fit.edu/~pkc/papers/tdm04.pdf

"""


from helpers import read_in_mot_as_numpy
from helpers import plot_data
from helpers import plot_compare_JAs
from helpers import plot_compare_JAs_with_new_errors
from helpers import compute_distance_matrix
from helpers import compute_accumulated_cost_matrix
from helpers import plot_cost_matrix
from helpers import plot_distance_matrix
from helpers import plot_data_with_new_paths

import numpy as np
import os
from fastdtw import fastdtw


# Quick Settings
start_time = 15.8
end_time = 17.3
trim_bool = True

subject_code = 'P3'
trial_name = 'JA_Fast'
calibration_name = 'METHOD2_Alt_self'
IMU_type = 'Real'
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Testing_DTW'
data_dir = os.path.join(parent_dir, 'RawData')
results_dir = os.path.join(parent_dir, "Results")
np.set_printoptions(suppress=True, precision=2)


""" READ IN DATA """

# Read in data from mot file as np arrays
OMC_angle_np, IMU_angle_np, time = read_in_mot_as_numpy(subject_code, trial_name, calibration_name, start_time, end_time, trim_bool, IMU_type)

# Plot the data using my own style/function
plot_compare_JAs(OMC_angle_np, IMU_angle_np, time, start_time, end_time, results_dir)

# Example arrays for analysis
# x = np.array([7, 1, 2, 5, 9])
# y = np.array([1, 8, 0, 4, 4, 2, 0])

# Define the two time-series data sets to be used in the DTW analysis
x = OMC_angle_np
y = IMU_angle_np

print(x[:10])
print(y[:10])


""" PLOT ORIGINAL DATA """

plot_data(x, y, results_dir)


"""" RUN FASTDTW """

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

# Use custom functions from online source to inspect results
dist_matrix = compute_distance_matrix(x, y)
cost_matrix = compute_accumulated_cost_matrix(x, y)

# Get the list of values in the distance matrix, defined by the warp path
distance_matrix_list = [dist_matrix[j][i] for i, j in warp_path]
print("\nDistance values in the distance matrix which correspond to the path: ", distance_matrix_list)

fastdtw_dist = sum(distance_matrix_list)
print("Distance value equivalent to that returned by the fastdtw() function: ", fastdtw_dist)

RMSE_eq = np.mean(np.square(distance_matrix_list))**0.5
print("RMSE equivalent using the distance matrix values path: ", RMSE_eq)

# Use my plotting function again but included the new list of errors for comparison
plot_compare_JAs_with_new_errors(OMC_angle_np, IMU_angle_np, distance_matrix_list,
                                 time, start_time, end_time, results_dir)


""" PLOT THE DISTANCE AND ACCUMULATED COST MATRIX """

# plot_cost_matrix(cost_matrix, warp_path, results_dir)     # note: too slow for big data (>5s)

# plot_distance_matrix(dist_matrix, warp_path, results_dir) # note: too slow for big data (>5s)


""" PLOT THE DATA AGAIN, BUT WITH THE NEW CONNECTING LINES SHOWING PAIRS """

plot_data_with_new_paths(x, y, warp_path, results_dir)
