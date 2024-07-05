
from helpers_humerus_imu_var import get_hum_clus_y_axis
from helpers_humerus_imu_var import get_angles_from_axis
from helpers_humerus_imu_var import plot_all_vec_3D

import numpy as np
import matplotlib.pyplot as plt

directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
figure_results_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\Variation_in_Manual_Placement'

subject_list = [f'P{i}' for i in range(1, 23)]
all_clus_y_axes = []

# For all aubjects, get the y-axis of the humerus cluster frame, expressed in the model's humerus body frame
for subject_code in subject_list:
    clus_y_axis = get_hum_clus_y_axis(subject_code, directory)
    all_clus_y_axes.append(clus_y_axis)

# Get the average y-axis of the cluster, relative to the humerus body frame
all_axes_np = np.array(all_clus_y_axes)
mean_clus_y_axis = np.mean(all_axes_np, axis=0)
mean_clus_y_axis /= np.linalg.norm(mean_clus_y_axis)

print(f'Average y-axis vector is: {mean_clus_y_axis}')

""" 3D PLOT ALL THE IMU Y AXIS VECTORS (IN THE HUMERUS FRAME) """

plot_all_vec_3D(all_axes_np, mean_clus_y_axis)


""" 2D PLOT THE IMU Y AXIS VECTORS, ON THE YZ (flexion) AND YX (abduction) PLANES """

# Initiate some arrays to hold the angle results
angles_YZ = []
angles_YX = []
subject_no = 0

for y_axis in all_axes_np:
    subject_no = subject_no + 1
    angle_YX, angle_YZ = get_angles_from_axis(y_axis)
    angles_YZ.append(angle_YZ)
    angles_YX.append(angle_YX)
    print(f'Subject: {subject_no}, \n\tangle on YZ plane: {angle_YZ}, \n\tangle on YX plane: {angle_YX}')

angles_YZ = np.array(angles_YZ)
angles_YX = np.array(angles_YX)
print(f'Average angle in the YZ plane: {np.mean(angles_YZ)}')
print(f'Average angle in the YX plane: {np.mean(angles_YX)}')

# Create the 2D plot on the YZ plane

fig1 = plt.figure(figsize=(8, 8))
for y_axis in all_axes_np:
    y_axis_Xcomp = y_axis[0]    # X component of the axis
    y_axis_Ycomp = y_axis[1]    # Y component of the axis
    y_axis_Zcomp = y_axis[2]    # Z component of the axis
    # Get the component of the y-axis which is in the YZ plane
    y_axis_YZ = np.array([y_axis_Ycomp, y_axis_Zcomp])
    plt.quiver(0, 0, y_axis_YZ[0], y_axis_YZ[1], angles='xy', scale_units='xy', scale=1)
plt.xlim(-0.1, 0.05)
plt.ylim(-0.1, 0.05)
plt.grid()
plt.xlabel('Y axis')
plt.ylabel('Z axis')
plt.title('For all subjects, the direction of the IMU y-axis on the humerus YZ plane (flexion)')
plt.show()
plt.close()
fig1.savefig(figure_results_dir + "\\" + "IMU_y_axis_on_YZ.png")

# Create the 2D plot on the YZX plane

fig2 = plt.figure(figsize=(8, 8))
for y_axis in all_axes_np:
    y_axis_Xcomp = y_axis[0]    # X component of the axis
    y_axis_Ycomp = y_axis[1]    # Y component of the axis
    y_axis_Zcomp = y_axis[2]    # Z component of the axis
    # Get the component of the y-axis which is in the YZ plane
    y_axis_YX = np.array([y_axis_Ycomp, y_axis_Xcomp])
    plt.quiver(0, 0, y_axis_YX[0], y_axis_YX[1], angles='xy', scale_units='xy', scale=1)
plt.xlim(-0.1, 0.05)
plt.ylim(-0.1, 0.05)
plt.grid()
plt.xlabel('Y axis')
plt.ylabel('X axis')
plt.title('For all subjects, the direction of the IMU y-axis on the humerus YX plane (abduction)')
plt.show()
plt.close()
fig2.savefig(figure_results_dir + "\\" + "IMU_y_axis_on_YX.png")

