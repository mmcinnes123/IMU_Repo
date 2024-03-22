










""" Written in IMU_preproccess to work with IMU1, IMU2, IMU3 dataframes"""
# Function to calculate orientation accuracy of real IMUs compared with perfect IMUs
def compare_oris_real_vs_perfect(IMU_df_Real, IMU_df_Perfect):

    # Get orientation at specified static time
    static_time_index = static_time * sample_rate
    IMU_Perfect_R_atT = R.from_quat(IMU_df_Perfect.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])
    IMU_Real_R_atT = R.from_quat(IMU_df_Real.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])

    # Create an array to fill with error values
    single_angle_diff = np.zeros((len(IMU_df_Real)))

    for row in range(len(IMU_df_Real)):

        # For all rows which don't contain any nan values in either perfect or real df
        if IMU_df_Real.iloc[row].isna().any() == False and IMU_df_Perfect.iloc[row].isna().any() == False:

            # Get scipy format orientations, ready for comparison
            IMU_Perfect_R = R.from_quat(IMU_df_Perfect.iloc[row].to_numpy()[[1, 2, 3, 0]])
            IMU_Real_R = R.from_quat(IMU_df_Real.iloc[row].to_numpy()[[1, 2, 3, 0]])

            # Get CHANGE in orientations, relative to orientation at specified static time
            ori_change_Perfect_R = IMU_Perfect_R_atT.inv() * IMU_Perfect_R
            ori_change_Real_R = IMU_Real_R_atT.inv() * IMU_Real_R

            # Compare real vs perfect change in orientations
            ori_diff = ori_change_Perfect_R.inv() * ori_change_Real_R
            single_angle_diff_i = ori_diff.magnitude() * 180 / np.pi

        else:
            single_angle_diff_i = np.nan     # If there are nans in either perfect or real dfs, set the angle diff to nan

        single_angle_diff[row] = single_angle_diff_i

    return single_angle_diff


# Apply the function above to compare the real and perfect IMUs
IMU1_single_angle_diff = compare_oris_real_vs_perfect(IMU1_df_Real, IMU1_df_Perfect)
IMU2_single_angle_diff = compare_oris_real_vs_perfect(IMU2_df_Real, IMU2_df_Perfect)
IMU3_single_angle_diff = compare_oris_real_vs_perfect(IMU3_df_Real, IMU3_df_Perfect)

# Calculate RMSE and plot the time-varying error
IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = plot_compare_real_vs_perfect(IMU1_single_angle_diff, IMU2_single_angle_diff, IMU3_single_angle_diff, parent_dir)

# Write final RMSE values to a csv
final_RMSE_values_df = pd.DataFrame.from_dict(
    {"Thorax IMU orientation error:": IMU1_single_angle_RMSE,
     "Humerus IMU orientation error:": IMU2_single_angle_RMSE,
     "Forearm IMU orientation error:": IMU3_single_angle_RMSE,}, orient='index')
final_RMSE_values_df.to_csv(parent_dir + "\\" + "Real_vs_Perfect_Ori_Errors.csv",
                            mode='w', encoding='utf-8', na_rep='nan')




""" Written to invesitgate relative orientation of IMU and cluster local CFs"""

def read_angvel_data_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    Cluster_df = df.filter(["Forearm_Cluster_X", "Forearm_Cluster_Y", "Forearm_Cluster_Z"], axis=1)
    IMU_df = df.filter(["Forearm_IMU_X", "Forearm_IMU_Y", "Forearm_IMU_Z"], axis=1)
    return Cluster_df, IMU_df

input_file = 'P2_JA_Slow - Report7 - Angular_Velocities_LOCAL.txt'
input_file_path = os.path.join(raw_data_dir, input_file)

Cluster_df, IMU_df = read_angvel_data_from_file(input_file_path)

ang_vel_arr_real = IMU_df.to_numpy()
ang_vel_arr_perfect = Cluster_df.to_numpy()

# Use time frame:
start_time = 20
end_time = 80
delay = 5
ang_vel_arr_real = ang_vel_arr_real[((start_time*sample_rate)+delay):((end_time*sample_rate)+delay)]
ang_vel_arr_perfect = ang_vel_arr_perfect[(start_time*sample_rate):(end_time*sample_rate)]


# Filter the arrays to remove values less than cut-off
cut_off = 40
indices_to_delete = np.concatenate((np.where(np.linalg.norm(ang_vel_arr_real, axis=1) < cut_off),
                                    np.where(np.linalg.norm(ang_vel_arr_perfect, axis=1) < cut_off)), axis=1)
ang_vel_arr_real = np.delete(ang_vel_arr_real, indices_to_delete, axis=0)
ang_vel_arr_perfect = np.delete(ang_vel_arr_perfect, indices_to_delete, axis=0)

# Normalise the angular velocity vectors so we're only comparing directions
for row in range(len(ang_vel_arr_real)):
    ang_vel_arr_real[row] = ang_vel_arr_real[row]/np.linalg.norm(ang_vel_arr_real[row])
    ang_vel_arr_perfect[row] = ang_vel_arr_perfect[row]/np.linalg.norm(ang_vel_arr_perfect[row])


# Use scipy's 'align_vectors' to try to solve for the average misalignment (this shouldn't actually apply to the local CFs)
a = ang_vel_arr_perfect
b = ang_vel_arr_real
rot, rssd, sens = R.align_vectors(a, b, return_sensitivity=True)
print(rot.as_matrix())
print(rot.as_euler('yxz', degrees=True))
print(rssd)
print(sens)


# # Plot the quaternions
# import matplotlib.pyplot as plt # importing the required module
# window_length = 50
# polynomial = 2
# # ang_vel_arr_real_filt = savgol_filter(ang_vel_arr_real[:,0], window_length, polynomial)
# time = list(range(len( IMU3_df_Perfect.iloc[2000:3000])))
# plt.scatter(time, IMU3_df_Perfect.iloc[7200:8200]['IMU3_Q0'], s=0.5, c='blue')
# plt.scatter(time, IMU3_df_Perfect.iloc[7200:8200]['IMU3_Q1'], s=0.5, c='blue')
# plt.scatter(time, IMU3_df_Perfect.iloc[7200:8200]['IMU3_Q2'], s=0.5, c='blue')
# plt.scatter(time, IMU3_df_Perfect.iloc[7200:8200]['IMU3_Q3'], s=0.5, c='blue')
# # plt.scatter(time, ang_vel_arr_real[:,1])
# # plt.scatter(time, ang_vel_arr_real[:,2])
# # plt.scatter(time, ang_vel_arr_real_filt)
# plt.xlabel('x - axis')
# plt.ylabel('y - axis')
# plt.show()

#



#
# # Plot the  angular velocity
# import matplotlib.pyplot as plt     # importing the required module
# time = list(range(len(ang_vel_arr_real[:,0])))
# plt.scatter(time, ang_vel_arr_real[:,0], s=0.5, c='blue')
# plt.scatter(time, ang_vel_arr_real[:,1], s=0.5, c='blue')
# plt.scatter(time, ang_vel_arr_real[:,2], s=0.5, c='blue')
# plt.scatter(time, ang_vel_arr_perfect[:,0], s=0.5, c='red')
# plt.scatter(time, ang_vel_arr_perfect[:,1], s=0.5, c='red')
# plt.scatter(time, ang_vel_arr_perfect[:,2], s=0.5, c='red')
# plt.xlabel('x - axis')
# plt.ylabel('y - axis')
# plt.show()

#
#
# # Plot the magnitude of the angular velocities
# mag_real = np.linalg.norm(ang_vel_arr_real, axis=1)
# mag_perfect = np.linalg.norm(ang_vel_arr_perfect, axis=1)
# import matplotlib.pyplot as plt # importing the required module
# time = list(range(len(mag_real)))
# plt.scatter(time, mag_real, s=0.5, c='blue')
# # plt.scatter(time, mag_perfect, s=0.5, c='red')
# plt.xlabel('x - axis')
# plt.ylabel('y - axis')
# plt.show()
# #
#
#
# # Animate the 3d vectors
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
#
# fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))
# # Create new quivers
# quiver_real = ax.quiver(0, 0, 0, ang_vel_arr_real[0,0], ang_vel_arr_real[0,1], ang_vel_arr_real[0,2], color='blue')
# quiver_perfect = ax.quiver(0, 0, 0, ang_vel_arr_perfect[0,0], ang_vel_arr_perfect[0,1], ang_vel_arr_perfect[0,2], color='red')
# # Set the range of the plot axes
# plot_range = 1
# ax.set_xlim(-plot_range, plot_range)
# ax.set_ylim(-plot_range, plot_range)
# ax.set_zlim(-plot_range, plot_range)
#
# # Define a function for updating the quiver (vector) values, by iterating theta through the arrays created above
# def update_ang_vel_vec_real(theta):
#     global quiver_real
#     quiver_real.remove()
#     quiver_real = ax.quiver(0, 0, 0, ang_vel_arr_real[theta,0], ang_vel_arr_real[theta,1], ang_vel_arr_real[theta,2], color='blue')
# def update_ang_vel_vec_perfect(theta):
#     global quiver_perfect
#     quiver_perfect.remove()
#     quiver_perfect = ax.quiver(0, 0, 0, ang_vel_arr_perfect[theta,0], ang_vel_arr_perfect[theta,1], ang_vel_arr_perfect[theta,2], color='red')
#
# # Create the animations, where 'update' is the function, 'frames' creates the values of theta to iterate through, and interval is the gap between frames (ms)
# ani_perfect = FuncAnimation(fig, update_ang_vel_vec_real, frames=np.array(range(len(ang_vel_arr_perfect))), interval=10, repeat_delay=1000)
# ani_real = FuncAnimation(fig, update_ang_vel_vec_perfect, frames=np.array(range(len(ang_vel_arr_perfect))), interval=10, repeat_delay=1000)
# plt.show()




# Old way of doing it:
# # Some code if you only want to read in two files
# input_file_Perfect = "P2_JA_Slow - Report3_alt - Cluster_Quats.txt"     # Name of the txt file with perfect IMU data
# input_file_Real = "P2_JA_Slow - Report2_alt - IMU_Quats.txt"     # Name of the txt file with real IMU data
# file_path_Perfect = os.path.join(raw_data_dir, input_file_Perfect)
# file_path_Real = os.path.join(raw_data_dir, input_file_Real)
# cal_pose_time_dict = {"Pose2_assisted": 26}  # List of pose times for calibration
# IMU1_df_Perfect, IMU2_df_Perfect, IMU3_df_Perfect = read_data_frame_from_file(file_path_Perfect)
# IMU1_df_Real, IMU2_df_Real, IMU3_df_Real = read_data_frame_from_file(file_path_Real)
#
# # Some code to calculate angular velocities in the local frame *NOTE* This doesn't handle quaternions which flip from canonical to non-canonical (double coverage)
# def angular_velocities(q1, q2, dt):
#     return (2 / dt) * np.array([
#         q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
#         q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
#         q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])
#
# qs = IMU3_df_Perfect.iloc[2000:3000].to_numpy()
# real_qs = IMU3_df_Real.iloc[2000:3000].to_numpy()
#
# ang_vel_arr_perfect = np.zeros(((len(qs)-10), 3))
# for i in range(0, len(qs)-10):
#     ang_vel = angular_velocities(qs[i], qs[i+10], 0.1)
#     ang_vel_arr_perfect[i] = ang_vel
#
# ang_vel_arr_real = np.zeros(((len(qs)-10), 3))
# for i in range(0, len(qs)-10):
#     ang_vel = angular_velocities(real_qs[i], real_qs[i+10], 0.1)
#     ang_vel_arr_real[i] = ang_vel


#
#










