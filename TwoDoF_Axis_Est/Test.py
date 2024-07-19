


from helpers_2DoF import get_ang_vels_from_quats
from helpers_2DoF import filter_gyr_data
from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import get_np_quats_from_txt_file
from joint_axis_est_2d import _qmult
from joint_axis_est_2d import _rotate
from joint_axis_est_2d import _cross
from joint_axis_est_2d import inner1d
from joint_axis_est_2d import axisToThetaPhi
from joint_axis_est_2d import axisFromThetaPhi

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join
import logging
import numpy as np
import pandas as pd
from tkinter.filedialog import askopenfilename, askdirectory

np.set_printoptions(suppress=True)
osim.Logger.setLevelString("Off")
logging.basicConfig(level=logging.INFO, filename="FE_axis.log", filemode="w")


def err_func(d, theta1, phi1, var1, theta2, phi2, var2, delta):

    q1 = d['quat1']
    q2 = d['quat2']
    w1_e1 = d['gyr1_E1']  # gyr1_E1 = qmt.rotate(quat1, gyr1)
    w2_e2 = d['gyr2_E2']  # gyr2_E2 = qmt.rotate(quat2, gyr2)
    N = q1.shape[0]
    assert q1.shape == q2.shape == (N, 4)
    assert w1_e1.shape == w2_e2.shape == (N, 3)

    # Get axes from spherical coords
    j1_est = axisFromThetaPhi(theta1, phi1, var1)
    j2_est = axisFromThetaPhi(theta2, phi2, var2)

    q_E2_E1 = np.array([np.cos(delta / 2), 0, 0, np.sin(delta / 2)], float)

    q2_e1_est = _qmult(q_E2_E1, q2)
    j1_e1 = _rotate(q1, j1_est)
    j2_e1 = _rotate(q2_e1_est, j2_est)
    w2_e1 = _rotate(q_E2_E1, w2_e2)

    ax_orig = _cross(j1_e1, j2_e1)
    ax_norm = np.linalg.norm(ax_orig, axis=1)[:, None]
    ax = ax_orig / ax_norm
    w_d = w1_e1 - w2_e1
    err = inner1d(w_d, ax)

    cost = np.mean(err ** 2)

    return cost


def get_qs_and_angvels(quat1, quat2, rate):

    # Update the parameters (settings) with default values if they haven't been specified in the input
    defaults = dict(method='rot', gyrCutoff=5, downsampleRate=20)

    # Check the quaternion and gyro arrays are the correct shape
    N = quat1.shape[0]
    assert quat1.shape == (N, 4)
    assert quat2.shape == (N, 4)

    # Define each setting from the params dict
    method = defaults['method']
    gyrCutoff = defaults['gyrCutoff']
    downsampleRate = defaults['downsampleRate']
    assert method in ('rot', 'ori', 'rot_noDelta')

    # Downsample the orientation data
    if rate == downsampleRate or downsampleRate is None:
        ind = slice(None)
    else:
        assert downsampleRate < rate
        M = int(round(N*downsampleRate/rate))
        ind = np.linspace(0, N-1, M, dtype=int)
    q1 = quat1[ind].copy()
    q2 = quat2[ind].copy()

    # Create synthesised gyro data from quaternion data
    # Use the down-sampled orientation data to calculate angular velocities
    # Note: these are already in the IMUs reference frame, not in the local frame as real gyro data would be
    gyr1_E1 = get_ang_vels_from_quats(q1, downsampleRate, debug_plot=False)
    gyr2_E2 = get_ang_vels_from_quats(q2, downsampleRate, debug_plot=False)

    # And remove the last row from the ori data to match the size of the gyro data
    q1 = q1[:-1]
    q2 = q2[:-1]

    # Apply a butterworth low pass filter to the angular velocity data
    # (The gyrCutoff is the cut-off frequency used to filter the angular rates)
    if gyrCutoff is not None:  # apply Butterworth low pass filter
        gyr1_E1 = filter_gyr_data(gyr1_E1, gyrCutoff, downsampleRate, plot=False)
        gyr2_E2 = filter_gyr_data(gyr2_E2, gyrCutoff, downsampleRate, plot=False)

    # Remove rows with nans from quat and gyr data
    nan_rows = np.isnan(q1).any(axis=1) | np.isnan(q2).any(axis=1) | np.isnan(gyr1_E1).any(axis=1) | np.isnan(gyr2_E2).any(axis=1)
    perc_removed = 100 * np.sum(nan_rows)/len(q1)
    if perc_removed > 5:
        print(f'WARNING: {perc_removed:.1f}% data was missing from the available optimisation period.')
    if perc_removed > 20:
        print(f'QUITTING: {perc_removed:.1f}% of the optimisation data was missing')
        quit()
    q1 = q1[~nan_rows]
    q2 = q2[~nan_rows]
    gyr1_E1 = gyr1_E1[~nan_rows]
    gyr2_E2 = gyr2_E2[~nan_rows]

    # Define the dict of data to be used in the optimisation function
    d = dict(quat1=q1, quat2=q2, gyr1_E1=gyr1_E1, gyr2_E2=gyr2_E2)

    return d



def get_input_data_from_file(subject_code, IMU_type_for_opt, start_time, end_time, trial_for_opt, sample_rate):

    if IMU_type_for_opt == 'Perfect':
        report_ext = ' - Report3 - Cluster_Quats.txt'
    elif IMU_type_for_opt == 'Real':
        report_ext = ' - Report2 - IMU_Quats.txt'
    else:
        report_ext = None

    # Get the .txt file with quats based on subject code, whcih trial to use, and which type of IMU
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    raw_data_dir = join(parent_dir, 'RawData')
    tmm_txt_file_name = subject_code + '_' + trial_for_opt + report_ext
    tmm_txt_file = join(raw_data_dir, tmm_txt_file_name)

    # Read in the IMU quaternion data from a TMM report .txt file
    IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(tmm_txt_file)

    # Trim the IMU data based on the period of interest
    start_ind = start_time * sample_rate
    end_ind = end_time * sample_rate
    IMU2_trimmed = IMU2_np[start_ind:end_ind]
    IMU3_trimmed = IMU3_np[start_ind:end_ind]

    quat1 = IMU2_trimmed
    quat2 = IMU3_trimmed

    return quat1, quat2



"""" RUN FUNCTIONS ABOVE """

subject_code = 'P23'
IMU_type_for_opt = 'Perfect'
# start_time = 38
start_time = 28
# end_time = 46
end_time = 37
# start_time = 12
# end_time = 24
trial_for_opt = 'CP'
sample_rate = 100          # This is the sample rate of the data going into the function
opt_method = 'rot'

quat1, quat2 = get_input_data_from_file(subject_code, IMU_type_for_opt, start_time, end_time, trial_for_opt, sample_rate)

data = get_qs_and_angvels(quat1, quat2, sample_rate)

# Run the optimisation
from TwoDoF_Axis_Est.joint_axis_est_2d import jointAxisEst2D
params = dict(method=opt_method)
opt_results = jointAxisEst2D(quat1, quat2, None, None, sample_rate, params=params, debug=True, plot=False)
j1_est = opt_results['j1']
j2_est = opt_results['j2']
delta = opt_results['delta']

# Turn j1 and j2 into spherical coordinates for use in the cost function
var1 = 3
var2 = 3
theta1, phi1 = axisToThetaPhi(j1_est, var1)
theta2, phi2 = axisToThetaPhi(j2_est, var2)

print('FE: ', j1_est)
print('PS: ', j2_est)
print('Heading offset (rad): ', delta)
print('Heading offset (deg): ', np.rad2deg(delta))
print('Cost: ', opt_results['debug']['cost'])
print('Theta1 (rad): ', theta1)
print('Phi1 (rad): ', phi1)
print('Theta2 (rad): ', theta2)
print('Phi2 (rad): ', phi2)

# visulalise_opt_result_vec_on_IMU(j1_est, np.array([0, 0, 1]), None)


# Apply my function to check cost function results match
cost = err_func(data, theta1, phi1, var1, theta2, phi2, var2, delta)
print(f'My Cost: {cost}')


""" PLOT THE COST FUNCTION """

import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots


# Range of values to plot
theta1_values = np.linspace(-np.pi, np.pi, 100)
phi1_values = np.linspace(-np.pi, np.pi, 100)
delta_values = np.linspace(-np.pi/2, np.pi/2, 100)

#
# # Varying theta1 and delta
# fig = make_subplots(rows=1, cols=1, specs=[[{'type': 'surface'}]])
# theta1_grid, delta_grid = np.meshgrid(theta1_values, delta_values)
# cost_grid = np.zeros_like(theta1_grid)
# for i in range(theta1_grid.shape[0]):
#     for j in range(theta1_grid.shape[1]):
#         cost_grid[i, j] = err_func(data, theta1_grid[i, j], phi1, var1, theta2, phi2, var2, delta_grid[i, j])
# fig.add_trace(
#     go.Surface(z=cost_grid, x=theta1_grid, y=delta_grid, colorscale='Viridis'),
#     row=1, col=1)
# # Add a countour plot
# fig.update_traces(contours_z=dict(show=True, usecolormap=True,
#                                   highlightcolor="limegreen", project_z=True, start=cost_grid.min(), end=cost_grid.max(), size=0.2))
# # Plot a marker at the actual solution
# solution_cost = err_func(data, theta1, phi1, var1, theta2, phi2, var2, delta)
# fig.add_trace(go.Scatter3d(
#     x=[theta1],
#     y=[delta],
#     z=[solution_cost],
#     mode='markers',
#     marker=dict(size=5, color='red', symbol='cross'),
#     name='Solution Point'
# ))
# fig.update_layout(scene=dict(
#     xaxis_title='theta1',
#     yaxis_title='delta',
#     zaxis_title='Cost'
# ), title_text=f'Varying Theta1 and Delta')
#




# Varying phi1 and delta
fig = make_subplots(rows=1, cols=1, specs=[[{'type': 'surface'}]])
phi1_grid, delta_grid = np.meshgrid(phi1_values, delta_values)
cost_grid = np.zeros_like(phi1_grid)
for i in range(phi1_grid.shape[0]):
    for j in range(phi1_grid.shape[1]):
        cost_grid[i, j] = err_func(data, theta1, phi1_grid[i, j], var1, theta2, phi2, var2, delta_grid[i, j])
fig.add_trace(
    go.Surface(z=cost_grid, x=phi1_grid, y=delta_grid, colorscale='Viridis'),
    row=1, col=1)
# Add a countour plot
fig.update_traces(contours_z=dict(show=True, usecolormap=True,
                                  highlightcolor="limegreen", project_z=True, start=cost_grid.min(), end=cost_grid.max(), size=0.2))
# Plot a marker at the actual solution
solution_cost = err_func(data, theta1, phi1, var1, theta2, phi2, var2, delta)
fig.add_trace(go.Scatter3d(
    x=[phi1],
    y=[delta],
    z=[solution_cost],
    mode='markers',
    marker=dict(size=5, color='red', symbol='cross'),
    name='Solution Point'
))
fig.update_layout(scene=dict(
    xaxis_title='phi1',
    yaxis_title='delta',
    zaxis_title='Cost'
), title_text=f'Varying phi1 and Delta')


fig.show()

save_fig_path = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\R Analysis\R 2DoF Opt'
save_file = join(save_fig_path, 'phi1vsDelta_ADL.html')
# fig.write_html(save_file)



""" GET THE WHOLE SOLUTION SPACE """
#
# # Define the range of values for each variable
# num_points = 20
# theta1_values = np.linspace(0, np.pi, num_points)
# phi1_values = np.linspace(-np.pi, np.pi, num_points)
# theta2_values = np.linspace(0, np.pi, num_points)
# phi2_values = np.linspace(-np.pi, np.pi, num_points)
# delta_values = np.linspace(-np.pi/2, np.pi/2, num_points)
#
# # Create the meshgrid for all five variables
# theta1_grid, phi1_grid, theta2_grid, phi2_grid, delta_grid = np.meshgrid(theta1_values, phi1_values,
#                                                                          theta2_values, phi2_values,
#                                                                          delta_values)
#
# # Initialize the result array
# cost_values = np.zeros(theta1_grid.shape)
#
#
# # Iterate over all grid points and compute the cost
# for i in range(num_points):
#     for j in range(num_points):
#         for k in range(num_points):
#             for l in range(num_points):
#                 for m in range(num_points):
#                     cost_values[i, j, k, l, m] = err_func(
#                         data,
#                         theta1_grid[i, j, k, l, m],
#                         phi1_grid[i, j, k, l, m],
#                         var1,
#                         theta2_grid[i, j, k, l, m],
#                         phi2_grid[i, j, k, l, m],
#                         var2,
#                         delta_grid[i, j, k, l, m]
#                     )
#
# # Find the minimum value in the cost_values array
# min_cost = np.min(cost_values)
#
# # Find the indices of the minimum value in the cost_values array
# min_index = np.unravel_index(np.argmin(cost_values), cost_values.shape)
#
# # Retrieve the corresponding values of theta1, phi1, theta2, phi2, and delta
# min_theta1 = theta1_grid[min_index]
# min_phi1 = phi1_grid[min_index]
# min_theta2 = theta2_grid[min_index]
# min_phi2 = phi2_grid[min_index]
# min_delta = delta_grid[min_index]
#
# # Print the results
# print(f"Minimum cost value: {min_cost}")
# print(f"theta1: {min_theta1}")
# print(f"phi1: {min_phi1}")
# print(f"theta2: {min_theta2}")
# print(f"phi2: {min_phi2}")
# print(f"delta: {min_delta}")

