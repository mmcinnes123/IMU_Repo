
import opensim as osim
import qmt
import plotly.figure_factory as ff
import plotly.graph_objects as go
import plotly.io as pio
import numpy as np
from os.path import join



def get_IMU_in_body_frame(model_file):
    """Get the cluster frames, expressed relative to the body frames, specific to the subject_code's model"""

    # Read in calibrated model file to get position of humerus markers in humerus body frame
    my_model = osim.Model(model_file)
    marker_1_in_hum = my_model.getMarkerSet().get('Hum_Clus_1').get_location().to_numpy()
    marker_3_in_hum = my_model.getMarkerSet().get('Hum_Clus_3').get_location().to_numpy()
    marker_4_in_hum = my_model.getMarkerSet().get('Hum_Clus_4').get_location().to_numpy()
    # And radius markers in radius body frame
    marker_1_in_rad = my_model.getMarkerSet().get('Fore_Clus_1').get_location().to_numpy()
    marker_2_in_rad = my_model.getMarkerSet().get('Fore_Clus_2').get_location().to_numpy()
    marker_4_in_rad = my_model.getMarkerSet().get('Fore_Clus_4').get_location().to_numpy()

    # Humerus cluster CF expressed in the humerus CF, using the marker positions
    hum_clus_y_axis = marker_1_in_hum - marker_4_in_hum  # y_axis is marker 4 to marker 1 (pointing down)
    hum_clus_x_axis = marker_3_in_hum - marker_4_in_hum  # x_axis is marker 4 to marker 3 (pointing backwards)
    hum_clus_in_hum = qmt.quatFrom2Axes(hum_clus_x_axis, hum_clus_y_axis, None, plot=False)

    # Forearm cluster CF expressed in the radius CF, using the marker positions
    rad_clus_y_axis = marker_2_in_rad - marker_1_in_rad  # y_axis is marker 1 to marker 2 (pointing down)
    rad_clus_x_axis = marker_1_in_rad - marker_4_in_rad  # x_axis is marker 4 to marker 1 (pointing backwards)
    rad_clus_in_rad = qmt.quatFrom2Axes(rad_clus_x_axis, rad_clus_y_axis, None, plot=False)

    return hum_clus_in_hum, rad_clus_in_rad


def get_all_clus_in_body_frame(subject_list):

    from constants import data_dir
    parent_dir = data_dir

    # Initiate empty list with hum_cluster ori results
    hum_clus_all = {}
    rad_clus_all = {}

    for subject_code in subject_list:

        OMC_dir = join(parent_dir, subject_code, 'OMC')
        model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

        # Get the orientation of the 'perfect' IMU CF (cluster CF), in the body CF
        hum_clus_in_hum, rad_clus_in_rad = get_IMU_in_body_frame(model_file)

        hum_clus_all[subject_code] = hum_clus_in_hum
        rad_clus_all[subject_code] = rad_clus_in_rad

    return hum_clus_all, rad_clus_all


def get_vecs_and_error_between_local_and_global(clus_all, JA_name, local_axis, global_axis_1, global_axis_2,
                                                target_global_axis):
    # Initiate dict with vecs
    vecs_dict = {}
    error_angles_dict = {}

    for subject in clus_all.keys():

        clus_in_body = clus_all[subject]

        local_index = dict(x=0, y=1, z=2)[local_axis]
        global_index_1 = dict(X=0, Y=1, Z=2)[global_axis_1]
        global_index_2 = dict(X=0, Y=1, Z=2)[global_axis_2]

        # Get the target global axis, expressed in the global plane
        assert target_global_axis.lstrip('-') in [global_axis_1, global_axis_2]
        if target_global_axis.lstrip('-') == global_axis_1:
            target_vec = np.array([1, 0])
        elif target_global_axis.lstrip('-') == global_axis_2:
            target_vec = np.array([0, 1])
        else:
            target_vec = np.array([0, 0])
            quit()

        if target_global_axis.startswith('-'):
            target_vec = -target_vec

        # Get the local IMU CF axes of interest
        clus_local_vec = qmt.quatToRotMat(clus_in_body)[:, local_index]

        # Get the local axis vector on the 2D plane of interest
        clus_vec_on_2D_plane = [clus_local_vec[global_index_1], clus_local_vec[global_index_2]]
        vecs_dict[subject] = clus_vec_on_2D_plane

        # Get the angle between the vector and the target vector (signed)
        angle_between = angle_between_2D_vecs_arctan(target_vec, clus_vec_on_2D_plane)
        error_angles_dict[subject] = angle_between

    return error_angles_dict, vecs_dict


def get_variation_in_error_angles(error_angles_dict):

    error_angles = np.array(list(error_angles_dict.values()))

    error_angles_mean = np.mean(error_angles)

    error_anlges_SD = np.std(error_angles)

    return error_angles_mean, error_anlges_SD


def plot_local_vec_on_global_plane(clus_all, JA_name, local_axis, global_axis_1, global_axis_2, target_global_axis, create_plot):

    error_angles_dict, vecs_dict = get_vecs_and_error_between_local_and_global(clus_all, JA_name,
                                                                               local_axis, global_axis_1,
                                                                               global_axis_2,
                                                                               target_global_axis)
    error_angles_mean, error_angles_SD = get_variation_in_error_angles(error_angles_dict)

    if create_plot:

        # Define the CF axes
        target_axis_u = [0, -1.5]
        target_axis_v = [1.5, 0]
        o_u = [0]
        o_v = [0]
        v_axis_label = global_axis_1
        u_axis_label = '-' + global_axis_2

        vecs = np.array(list(vecs_dict.values()))
        vec_us = vecs[:, 0]
        vec_vs = vecs[:, 1]
        vec_origins_u = [0]*len(vec_us)
        vec_origins_v = [0]*len(vec_us)


        # Create quiver plots of the 20 vec results
        quiver_vecs = ff.create_quiver(vec_origins_u, vec_origins_v, vec_us, vec_vs, line_color='red', scale=1.3, arrow_scale=.1, angle=np.pi / 6,
                                       name='vec1', line=dict(width=1))

        # Create the CF quiver plots (black)
        target_quiver_U = ff.create_quiver(o_u, o_v, [target_axis_u[0]], [target_axis_u[1]], line_color='black', scale=1, arrow_scale=.1, angle=np.pi / 6,
                                           name='vec1', line=dict(width=2))
        target_quiver_V = ff.create_quiver(o_u, o_v, [target_axis_v[0]], [target_axis_v[1]], line_color='black', scale=1, arrow_scale=.1, angle=np.pi / 6,
                                           name='vec1', line=dict(width=2))


        # Initialize the figure
        fig = go.Figure()

        # Add the first CF quiver
        for trace in target_quiver_U['data']:
            fig.add_trace(trace)

        # Add the second CF quiver
        for trace in target_quiver_V['data']:
            fig.add_trace(trace)

        # Add the 20 results quivers
        for trace in quiver_vecs['data']:
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
                x=target_axis_u[0], y=target_axis_u[1], xref="x", yref="y",
                text=u_axis_label, showarrow=False, xanchor="center", yanchor="top",  # Offset position
                xshift=0, yshift=0,  # Pixel offset from the point
                font=dict(size=20, family="Times New Roman Bold, Times, serif", color="black")
            ),
            dict(
                x=target_axis_v[0], y=target_axis_v[1], xref="x", yref="y",
                text=v_axis_label, showarrow=False, xanchor="right", yanchor="middle",  # Offset position
                xshift=0, yshift=0,  # Pixel offset from the point
                font=dict(size=20, family="Times New Roman Bold, Times, serif", color="black")
            )
        ]



        # Set the axes labels and properties
        fig.update_layout(
            xaxis=dict(
                title='',
                zeroline=True,
                # autorange='reversed',  # Invert the x-axis
                range = [2, -2],
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
            title={
                'text': f'Radius Sensor {local_axis}-axis on<br>Radius Segment {global_axis_1}{global_axis_2} Plane',
                'font': {'size': 34, 'color': 'black', 'family': 'Times New Roman, Times, serif'},
                'x': 0.5,
                'xanchor': 'center'},
            showlegend=False,
            width=600,  # Adjust width to ensure equal scaling visually
            height=600,
            plot_bgcolor='white',
            annotations=annotations
        )

        # Display the plot
        # fig.show()

        save_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\Reports\Paper\Image Development\Alignment Images'
        image_name = JA_name + '.png'
        image_path = join(save_dir, image_name)
        fig.write_image(image_path, width=600, height=600, scale=3, format='png',engine='kaleido')

    return error_angles_mean, error_angles_SD

def angle_between_two_2D_vecs(target_vec, vec2):

    # Calculate the angle (always a positive result)
    angle = np.arccos(np.dot(target_vec, vec2) / (np.linalg.norm(target_vec) * np.linalg.norm(vec2)))

    angle_deg = np.rad2deg(angle)

    return angle_deg




def angle_between_2D_vecs_arctan(target_vec, v2):
    """
    Calculate the signed angle between two 2D vectors using the cross product method.

    Parameters:
    v1 (np.array): The first vector as a numpy array [x1, y1].
    v2 (np.array): The second vector as a numpy array [x2, y2].

    Returns:
    float: The angle between the vectors in degrees. Positive indicates counterclockwise, negative indicates clockwise.
    """
    # Calculate the cross product (2D scalar version)
    cross_product = target_vec[0] * v2[1] - target_vec[1] * v2[0]

    # Calculate the dot product
    dot_product = np.dot(target_vec, v2)

    # Calculate the angle using atan2 (returns angle in radians)
    angle = np.arctan2(cross_product, dot_product)

    angle_deg = np.rad2deg(angle)

    return angle_deg


def get_single_angle_error(IMU_name, hum_clus_all, rad_clus_all):

    if IMU_name == 'humerus':
        local_axis = 'y'
        target_axis = [0, -1, 0]
        clus_all = hum_clus_all

    elif IMU_name == 'radius':
        local_axis = 'x'
        target_axis = [-1, 0, 0]
        clus_all = rad_clus_all

    local_index = dict(x=0, y=1, z=2)[local_axis]

    single_angle_errors = {}
    for subject in clus_all.keys():
        # Orientation of the cluster in the body frame
        clus_in_body = clus_all[subject]

        # Get the local IMU CF axes of interest
        clus_local_vec = qmt.quatToRotMat(clus_in_body)[:, local_index]

        # Get the anlge between the two 3D vecs
        angle = qmt.angleBetween2Vecs(target_axis, clus_local_vec)
        single_angle_errors[subject] = np.rad2deg(angle)

    def get_mean_and_sd_from_dict(dict):
        error_angles = np.array(list(dict.values()))
        error_angles_mean = np.mean(abs(error_angles))
        error_anlges_SD = np.std(error_angles)
        return error_angles_mean, error_anlges_SD

    mean_single_angle_error, sd_single_angle_error = get_mean_and_sd_from_dict(single_angle_errors)

    return mean_single_angle_error, sd_single_angle_error