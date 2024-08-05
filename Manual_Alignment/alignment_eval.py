# This script analyses how well the manual placement of the sensors matches the CFs of the underlying body segments
# Using perfect IMU data (i.e. marker cluster CFs) and calibrated OMC model
import qmt

from alignment_eval_helpers import get_IMU_in_body_frame

from os.path import join
import opensim as osim

import plotly.figure_factory as ff
import plotly.graph_objects as go
import plotly
import numpy as np


dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'

# Repress opensim logging
osim.Logger.setLevelString("Off")



def get_all_clus_in_body_frame(subject_list):

    # Initiate empty list with hum_cluster ori results
    hum_clus_all = {}
    rad_clus_all = {}

    for subject_code in subject_list:

        OMC_dir = join(dir, subject_code, 'OMC')
        model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

        # Get the orientation of the 'perfect' IMU CF (cluster CF), in the body CF
        hum_clus_in_hum, rad_clus_in_rad = get_IMU_in_body_frame(model_file)

        hum_clus_all[subject_code] = hum_clus_in_hum
        rad_clus_all[subject_code] = rad_clus_in_rad

    return hum_clus_all, rad_clus_all


def plot_local_vec_on_global_plane(clus_all, local_axis, global_axis_1, global_axis_2):

    # Initiate dict with vecs
    vecs_dict = {}

    for subject in clus_all.keys():

        hum_clus_in_hum = clus_all[subject]

        local_index = dict(x=0, y=1, z=2)[local_axis]
        global_index_1 = dict(X=0, Y=1, Z=2)[global_axis_1]
        global_index_2 = dict(X=0, Y=1, Z=2)[global_axis_2]

        # Get the local IMU CF axes of interest
        hum_clus_local_vec = qmt.quatToRotMat(hum_clus_in_hum)[:, local_index]

        # Get the local axis vector on the 2D plane of interest
        hum_clus_vec_on_2D_plane = [hum_clus_local_vec[global_index_1], hum_clus_local_vec[global_index_2]]

        vecs_dict[subject] = hum_clus_vec_on_2D_plane


    # Define the CF axes
    axis_u = [0, 1]
    axis_v = [1, 0]
    o_u = [0]
    o_v = [0]
    v_axis_label = global_axis_1
    u_axis_label = global_axis_2

    vecs = np.array(list(vecs_dict.values()))
    vec_us = vecs[:, 0]
    vec_vs = vecs[:, 1]
    vec_origins_u = [0]*len(vec_us)
    vec_origins_v = [0]*len(vec_us)


    # Create the first quiver plot (red)
    quiver_vecs = ff.create_quiver(vec_origins_u, vec_origins_v, vec_us, vec_vs, line_color='red', scale=1, arrow_scale=.1, angle=np.pi / 6,
                                   name='vec1', line=dict(width=1))

    # Create the second quiver plot (blue)
    quiver_U = ff.create_quiver(o_u, o_v, [axis_u[0]], [axis_u[1]], line_color='black', scale=1, arrow_scale=.1, angle=np.pi / 6,
                                name='vec1', line=dict(width=1))
    quiver_V = ff.create_quiver(o_u, o_v, [axis_v[0]], [axis_v[1]], line_color='black', scale=1, arrow_scale=.1, angle=np.pi / 6,
                                name='vec1', line=dict(width=1))


    # Initialize the figure
    fig = go.Figure()

    # Add the first quiver plot
    for trace in quiver_vecs['data']:
        fig.add_trace(trace)

    # Add the second quiver plot
    for trace in quiver_U['data']:
        fig.add_trace(trace)

    # Add the third quiver plot
    for trace in quiver_V['data']:
        fig.add_trace(trace)

    # Add annotations with custom labels
    annotations = [
        # dict(
        #     x=vecs[0], y=vecs[1], xref="x", yref="y",
        #     text=vec_label, showarrow=False,
        #     xanchor="left", yanchor="top",  # Offset position
        #     xshift=10, yshift=10  # Pixel offset from the point
        # ),
        dict(
            x=axis_u[0], y=axis_u[1], xref="x", yref="y",
            text=u_axis_label, showarrow=False, xanchor="center", yanchor="bottom",  # Offset position
            xshift=0, yshift=0  # Pixel offset from the point
        ),
        dict(
            x=axis_v[0], y=axis_v[1], xref="x", yref="y",
            text=v_axis_label, showarrow=False, xanchor="right", yanchor="middle",  # Offset position
            xshift=0, yshift=0  # Pixel offset from the point
        )
    ]



    # Set the axes labels and properties
    fig.update_layout(
        xaxis=dict(
            title='',
            zeroline=True,
            autorange='reversed',  # Invert the x-axis
            showgrid=False,
            showline=False,
            showticklabels=False
        ),
        yaxis=dict(
            title='',
            zeroline=True,
            scaleanchor="x",  # Ensure equal scaling
            scaleratio=1,
            showgrid=False,
            showline=False,
            showticklabels=False
        ),
        title=f'IMU {local_axis}-axis on the Humerus {global_axis_1}{global_axis_2} Plane',
        showlegend=False,
        width=600,  # Adjust width to ensure equal scaling visually
        height=600,
        plot_bgcolor='white',
        annotations=annotations
    )

    # Display the plot
    fig.show()


subject_list = [f'P{str(i).zfill(3)}' for i in range(1, 21)]

hum_clus_all, rad_clus_all = get_all_clus_in_body_frame(subject_list)

plot_local_vec_on_global_plane(hum_clus_all, local_axis='y', global_axis_1='Z', global_axis_2='Y')

