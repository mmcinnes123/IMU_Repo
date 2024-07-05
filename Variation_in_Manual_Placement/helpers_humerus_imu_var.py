import opensim as osim
import numpy as np
from os.path import join
import matplotlib.pyplot as plt


# Angle between two 2D vectors (note - always positive)
def angle_between_two_2D_vecs(vec1, vec2):
    angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))) * 180 / np.pi
    return angle


# From a calibration OMC model file (with markers) get the y-axis of the humerus cluster (in the humerus body CF)
def get_hum_clus_y_axis(subject_code, directory):

    # Get the cailbrated model file
    parent_dir = join(directory, subject_code)
    OMC_dir = join(parent_dir, 'OMC')
    model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')

    # Marker data
    osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files
    my_model = osim.Model(model_file)
    marker_1_in_hum = my_model.getMarkerSet().get('Hum_Clus_1').get_location().to_numpy()
    marker_4_in_hum = my_model.getMarkerSet().get('Hum_Clus_4').get_location().to_numpy()
    hum_clus_y_axis = marker_1_in_hum - marker_4_in_hum  # y_axis is marker 4 to marker 1 (pointing down)

    return hum_clus_y_axis


# Add an arrow to a 3D plot
def plot_arrow(ax, start, direction, color, linewidth, length, label):
    ax.plot([start[0], start[0] + direction[0] * length],
            [start[1], start[1] + direction[1] * length],
            [start[2], start[2] + direction[2] * length], color=color, linewidth=linewidth)
    if label != None:
        ax.text(start[0] + direction[0] * length * 1.1, start[1] + direction[1] * length * 1.1,
                start[2] + direction[2] * length * 1.1, label,
                color=color, fontsize=12)


# Get the angles (projected on a 2D plane) between the input axis and the coordinate frame's negative Y axis
def get_angles_from_axis(y_axis):
    y_axis_Xcomp = y_axis[0]  # X component of the axis
    y_axis_Ycomp = y_axis[1]  # Y component of the axis
    y_axis_Zcomp = y_axis[2]  # Z component of the axis

    # Get the component of the y-axis which is in the YZ plane
    y_axis_YZ = np.array([y_axis_Ycomp, y_axis_Zcomp])

    # Get the component of the y-axis which is in the YX plane
    y_axis_YX = np.array([y_axis_Ycomp, y_axis_Xcomp])

    # Get the angle between these vectors and the negative Y axis, on 2D planes
    negY_axis_YZ = np.array([-1, 0])
    negY_axis_YX = np.array([-1, 0])
    angle_YZ = angle_between_two_2D_vecs(y_axis_YZ, negY_axis_YZ)  # On the YZ plane
    angle_YX = angle_between_two_2D_vecs(y_axis_YX, negY_axis_YX)  # On the YX plane

    # On the YZ plane, if the z-component of the vector is positive, lets define the angle as negative
    if y_axis_YZ[1] > 0:
        angle_YZ = -angle_YZ

    # On the YX plane, if the x-component of the vector is negative, lets define the angle as negative
    if y_axis_YX[1] < 0:
        angle_YX = -angle_YX

    return angle_YX, angle_YZ


def plot_all_vec_3D(all_axes_np, mean_clus_y_axis):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    origin = np.array([0, 0, 0])

    # Plot a coordinate frame X, Y and Z axis
    x_axis = np.array([0.05, 0, 0])
    y_axis = np.array([0, 0.05, 0])
    z_axis = np.array([0, 0, 0.05])
    plot_arrow(ax, origin, x_axis, 'black', linewidth=3, length=1.3, label='X')
    plot_arrow(ax, origin, y_axis, 'black', linewidth=3, length=1.3, label='Y')
    plot_arrow(ax, origin, z_axis, 'black', linewidth=3, length=1.3, label='Z')

    # Plot all the cluster y-axes calculated above
    for vec in all_axes_np:
        plot_arrow(ax, origin, vec, 'blue', linewidth=3, length=0.8, label='')

    # Plot the average of all these vectors
    plot_arrow(ax, origin, mean_clus_y_axis, 'red', linewidth=2, length=0.8, label='y_man')

    # Plot settings
    ax.set_xlim([-0.05, 0.05])
    ax.set_ylim([-0.05, 0.05])
    ax.set_zlim([-0.05, 0.05])
    ax.invert_zaxis()
    ax.invert_xaxis()
    plt.show()
    plt.close()


