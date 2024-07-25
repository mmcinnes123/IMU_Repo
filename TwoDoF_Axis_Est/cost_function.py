

from helpers_2DoF import visulalise_opt_result_vec_on_IMU
from helpers_2DoF import rot_err_func
from helpers_2DoF import get_qs_and_angvels
from helpers_2DoF import get_input_data_from_file
from joint_axis_est_2d import axisToThetaPhi
from joint_axis_est_2d import jointAxisEst2D

import qmt
import opensim as osim
from os.path import join
import logging
import numpy as np
import plotly.graph_objs as go
from plotly.subplots import make_subplots
from datetime import datetime
from tkinter.filedialog import askopenfilename, askdirectory

np.set_printoptions(suppress=True)
osim.Logger.setLevelString("Off")
logging.basicConfig(level=logging.INFO, filename="FE_axis.log", filemode="w")


""" SETTINGS """

subject_code = 'P23'
IMU_type_for_opt = 'Perfect'
sample_rate = 100          # This is the sample rate of the data going into the function
opt_method = 'rot'
save_fig_path = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\R Analysis\R 2DoF Opt\3D Contour Plots'

# Chose the movement data to use for the optimisation/cost function
movement = 'ADL'

if movement == 'ArmDown':
    start_time = 28
    end_time = 37
    trial_for_opt = 'CP'

if movement == 'ArmElev':
    start_time = 38
    end_time = 46
    trial_for_opt = 'CP'

if movement == 'ADL':
    start_time = 12
    end_time = 24
    trial_for_opt = 'ADL'


# Get the quaternion data from the TMM .txt file, based on start and end time
quat1, quat2 = get_input_data_from_file(subject_code, IMU_type_for_opt, start_time, end_time, trial_for_opt, sample_rate)

# Get the variables in the form used in the opt (quat is down-sampled, ang vel is calculated and filtered)
data = get_qs_and_angvels(quat1, quat2, sample_rate)

run_the_opt = True
if run_the_opt:
    # Run the optimisation
    params = dict(method=opt_method)
    opt_results = jointAxisEst2D(quat1, quat2, None, None, sample_rate, params=params, debug=True, plot=False)
    j1_est = opt_results['j1']
    j2_est = opt_results['j2']
    delta = opt_results['delta']

    # Turn j1 and j2 into spherical coordinates for use in the cost function
    var1 = 3
    var2 = 3
    # Note, with spherical var = 3, theta is 'tilt' angle relative to y-axis, phi is 'heading' angle around y-axis.
    theta1, phi1 = axisToThetaPhi(j1_est, var1)
    theta2, phi2 = axisToThetaPhi(j2_est, var2)

    print('Optimisation Results: ')
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

    # Apply the cost function to the results (check that my cost = opt_results cost)
    cost = rot_err_func(data, theta1, phi1, var1, theta2, phi2, var2, delta)
    print(f'My Cost: {cost}')


""" PLOT THE COST FUNCTION """

# Range of values to plot
theta1_values = np.linspace(-np.pi, np.pi, 100)
phi1_values = np.linspace(-np.pi, np.pi, 100)
delta_values = np.linspace(-np.pi, np.pi, 100)

# Varying phi1 and delta
plot_cost_vs_phi1_vs_delta = True
if plot_cost_vs_phi1_vs_delta:

    # Get all the values to plot
    phi1_grid, delta_grid = np.meshgrid(phi1_values, delta_values)
    cost_grid = np.zeros_like(phi1_grid)
    for i in range(phi1_grid.shape[0]):
        for j in range(phi1_grid.shape[1]):
            cost_grid[i, j] = rot_err_func(data, theta1, phi1_grid[i, j], var1, theta2, phi2, var2, delta_grid[i, j])

    # Make the figure
    fig = make_subplots(rows=1, cols=2, specs=[[{'type': 'surface'}, {'type': 'contour'}]],
                        subplot_titles=("3D Surface Plot", "2D Contour Plot"))

    # Add the 3D surface
    fig.add_trace(go.Surface(z=cost_grid, x=phi1_grid, y=delta_grid, colorscale='Viridis'),
                  row=1, col=1)

    # Add a contour plot
    fig.update_traces(contours_z=dict(show=True, usecolormap=True, highlightcolor="limegreen",
                                      project_z=True, start=cost_grid.min(), end=cost_grid.max(), size=0.05))

    # Add a marker at the solution point
    solution_cost = rot_err_func(data, theta1, phi1, var1, theta2, phi2, var2, delta)
    fig.add_trace(go.Scatter3d(
        x=[phi1],
        y=[delta],
        z=[solution_cost],
        mode='markers',
        marker=dict(size=16, color='red', symbol='cross'),
        textposition='top center',  # Position of the label
        hoverinfo='text',  # Show only text on hover
        name='Solution Point',
        showlegend=False))

    # Add an annotation to the marker
    fig.update_layout(scene=dict(annotations=[dict(
                                                    showarrow=False,
                                                    x=phi1,
                                                    y=delta,
                                                    z=solution_cost,
                                                    text=f"Solution Point",
                                                    xanchor="left",
                                                    yanchor="bottom",
                                                    font=dict(color='#fde725', size=16))]))

    # Add 2D contour plot
    fig.add_trace(go.Contour(z=cost_grid, x=phi1_grid[0], y=delta_grid[:, 0], colorscale='Viridis', showscale=False), row=1, col=2)

    # Add red cross to the 2D contour plot
    fig.add_trace(go.Scatter(
            x=[phi1],
            y=[delta],
            mode='markers',
            marker=dict(color='red', size=13, symbol='cross'),
            name='Red Cross',
            hoverinfo='text',  # Show only text on hover
            showlegend=False),
        row=1, col=2)

    # Add annotation to the red cross on the 2D contour plot
    fig.add_annotation(
        x=phi1,
        y=delta,
        text=f"Solution Point <br>&nbsp;&nbsp;Delta: {round(delta*180/np.pi, 1)}&deg; "
             f"<br>&nbsp;&nbsp;Phi: {round(phi1*180/np.pi, 1)}&deg;",
        showarrow=False,
        xanchor='left',
        yanchor='top',
        # ax=20,
        # ay=-30,
        font=dict(color='#fde725', size=16),
        align='left',
        row=1,
        col=2)

    # Update axis titles for 2D plot
    fig.update_xaxes(title_text="Phi [rad]", titlefont=dict(size=18), tickfont=dict(size=16), row=1, col=2)
    fig.update_yaxes(title_text="Delta [rad]", titlefont=dict(size=18), tickfont=dict(size=16), row=1, col=2)

    # Update 3D plots axis titles and ticks
    fig.update_layout(scene=dict(
        xaxis_title=f'Phi [rad]',
        yaxis_title=f'Delta [rad]',
        zaxis_title='Cost',
        xaxis=dict(
            title=dict(font=dict(size=18)),
            tickfont=dict(size=14)),
        yaxis=dict(
            title=dict(font=dict(size=18)),
            tickfont=dict(size=14))),
        title=dict(
            text='Cost Function - Functional Movement',
            font=dict(size=24),
            x=0.5,
            xanchor='center'))

    fig.show()
    save_file = join(save_fig_path, 'Phi1vsDelta_' + movement + '.html')
    fig.write_html(save_file)


""" GET THE WHOLE SOLUTION SPACE """

run_sol_space = False
if run_sol_space:

    # Define the range of values for each variable
    num_points = 20
    theta1_values = np.linspace(-np.pi, np.pi, num_points)
    phi1_values = np.linspace(-np.pi, np.pi, num_points)
    theta2_values = np.linspace(-np.pi, np.pi, num_points)
    phi2_values = np.linspace(-np.pi, np.pi, num_points)
    delta_values = np.linspace(-np.pi/2, np.pi/2, num_points)

    # Create the meshgrid for all five variables
    theta1_grid, phi1_grid, theta2_grid, phi2_grid, delta_grid = np.meshgrid(theta1_values, phi1_values,
                                                                             theta2_values, phi2_values,
                                                                             delta_values)

    # Initialize the result array
    cost_values = np.zeros(theta1_grid.shape)

    comp_start_time = datetime.now()
    print('Time at start of calc: ', comp_start_time)

    # Iterate over all grid points and compute the cost
    for i in range(num_points):
        for j in range(num_points):
            for k in range(num_points):
                for l in range(num_points):
                    for m in range(num_points):
                        cost_values[i, j, k, l, m] = rot_err_func(
                            data,
                            theta1_grid[i, j, k, l, m],
                            phi1_grid[i, j, k, l, m],
                            var1,
                            theta2_grid[i, j, k, l, m],
                            phi2_grid[i, j, k, l, m],
                            var2,
                            delta_grid[i, j, k, l, m]
                        )
            print('i:', i)
            print('j:', j)


    comp_end_time = datetime.now()
    time_difference = (comp_end_time - comp_start_time).total_seconds()/60
    print('Time at end of computation: ', comp_end_time)
    print("Execution time of program is: ", time_difference, "minutes")

    # Find the minimum value in the cost_values array
    min_cost = np.min(cost_values)

    # Find the indices of the minimum value in the cost_values array
    min_index = np.unravel_index(np.argmin(cost_values), cost_values.shape)

    # Retrieve the corresponding values of theta1, phi1, theta2, phi2, and delta
    min_theta1 = theta1_grid[min_index]
    min_phi1 = phi1_grid[min_index]
    min_theta2 = theta2_grid[min_index]
    min_phi2 = phi2_grid[min_index]
    min_delta = delta_grid[min_index]



    # Print the results
    print('Solution Space Results:')
    print(f"Minimum cost value: {min_cost}")
    print(f"theta1: {min_theta1}")
    print(f"phi1: {min_phi1}")
    print(f"theta2: {min_theta2}")
    print(f"phi2: {min_phi2}")
    print(f"delta: {min_delta}")

