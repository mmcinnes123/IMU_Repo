# This script preprocess IMU data, ready for use in OpenSim
# Input is Motion Monitor .txt report file
# Output is .sto OpenSim file
# It also does an initial comparison of the raw orientation data from the 'real' and 'perfect' IMUs
# Output is a .csv with RMSE values for each IMU, and a .png plot

from functions import *
from IMU_IK_functions import APDM_2_sto_Converter
import os


""" SETTINGS """

# Quick Settings
subject_code = 'P2'
trial_name_list = ['CP', 'JA_Slow', 'JA_Fast', 'ROM', 'ADL']
parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
input_file_Perfect = "P2_JA_Slow - Report4 - Cluster_Quats_Stylus_CFs.txt"     # Name of the txt file with perfect IMU data
input_file_Real = "P2_JA_Slow - Report2 - IMU_Quats.txt"     # Name of the txt file with real IMU data
cal_pose_time_dict = {"Pose2_assisted": 26}  # List of pose times for calibration
sample_rate = 100
static_time = 1    # Input first known static time to use as reference for change in orientation error

# Required Files in Folder
template_file = "APDM_template_4S.csv"
APDM_settings_file = "APDMDataConverter_Settings.xml"

raw_data_dir = parent_dir + "\RawData"
file_path_Perfect = raw_data_dir + "\\" + input_file_Perfect
file_path_Real = raw_data_dir + "\\" + input_file_Real

# Create a new results directory
results_dir = parent_dir + "\Preprocessed_Data"
if os.path.exists(results_dir) == False:
    os.mkdir(results_dir)

osim.Logger.setLevelString("Off")


""" MAIN """


# Function to extract quaternion orientation data from .txt file and save as .sto file
def write_movements_and_calibration_stos(file_path, cal_pose_time_dict, IMU_type):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(file_path)

    # # Write data to APDM format .csv
    # file_tag = "APDM_Quats_" + IMU_type
    # write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, template_file, results_dir, file_tag)
    # # Write data to .sto using OpenSim APDM converter tool
    # APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
    #                      output_file_name=results_dir + "\\" + file_tag + ".sto")
    #
    # # Iterate through list of calibration poses and associated times to create separate .sto files
    # for pose_name in cal_pose_time_dict.keys():
    #
    #     cal_pose_time = cal_pose_time_dict[pose_name]
    #
    #     # Extract one row based on time of calibration pose
    #     IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
    #     IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
    #     IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)
    #
    #     # Write data to APDM format .csv
    #     file_tag = "APDM_Quats_" + IMU_type + "_" + str(cal_pose_time) + "s"
    #     write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, template_file, results_dir, file_tag)
    #     # Write data to .sto using OpenSim APDM converter tool
    #     APDM_2_sto_Converter(APDM_settings_file, input_file_name=results_dir + "\\" + file_tag + ".csv",
    #                          output_file_name=results_dir + "\\" + file_tag + ".sto")

    return IMU1_df, IMU2_df, IMU3_df

# Apply function above to create orientation dataframes and to write data to .sto files
IMU1_df_Perfect, IMU2_df_Perfect, IMU3_df_Perfect = write_movements_and_calibration_stos(file_path_Perfect, cal_pose_time_dict, IMU_type="Perfect")
IMU1_df_Real, IMU2_df_Real, IMU3_df_Real = write_movements_and_calibration_stos(file_path_Real, cal_pose_time_dict, IMU_type="Real")


# # Function to calculate orientation accuracy of real IMUs compared with perfect IMUs
# def compare_oris_real_vs_perfect(IMU_df_Real, IMU_df_Perfect):
#
#     # Get orientation at specified static time
#     static_time_index = static_time * sample_rate
#     IMU_Perfect_R_atT = R.from_quat(IMU_df_Perfect.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])
#     IMU_Real_R_atT = R.from_quat(IMU_df_Real.iloc[static_time_index].to_numpy()[[1, 2, 3, 0]])
#
#     # Create an array to fill with error values
#     single_angle_diff = np.zeros((len(IMU_df_Real)))
#
#     for row in range(len(IMU_df_Real)):
#
#         # For all rows which don't contain any nan values in either perfect or real df
#         if IMU_df_Real.iloc[row].isna().any() == False and IMU_df_Perfect.iloc[row].isna().any() == False:
#
#             # Get scipy format orientations, ready for comparison
#             IMU_Perfect_R = R.from_quat(IMU_df_Perfect.iloc[row].to_numpy()[[1, 2, 3, 0]])
#             IMU_Real_R = R.from_quat(IMU_df_Real.iloc[row].to_numpy()[[1, 2, 3, 0]])
#
#             # Get CHANGE in orientations, relative to orientation at specified static time
#             ori_change_Perfect_R = IMU_Perfect_R_atT.inv() * IMU_Perfect_R
#             ori_change_Real_R = IMU_Real_R_atT.inv() * IMU_Real_R
#
#             # Compare real vs perfect change in orientations
#             ori_diff = ori_change_Perfect_R.inv() * ori_change_Real_R
#             single_angle_diff_i = ori_diff.magnitude() * 180 / np.pi
#
#         else:
#             single_angle_diff_i = np.nan     # If there are nans in either perfect or real dfs, set the angle diff to nan
#
#         single_angle_diff[row] = single_angle_diff_i
#
#     return single_angle_diff
#
#
# # Apply the function above to compare the real and perfect IMUs
# IMU1_single_angle_diff = compare_oris_real_vs_perfect(IMU1_df_Real, IMU1_df_Perfect)
# IMU2_single_angle_diff = compare_oris_real_vs_perfect(IMU2_df_Real, IMU2_df_Perfect)
# IMU3_single_angle_diff = compare_oris_real_vs_perfect(IMU3_df_Real, IMU3_df_Perfect)
#
# # Calculate RMSE and plot the time-varying error
# IMU1_single_angle_RMSE, IMU2_single_angle_RMSE, IMU3_single_angle_RMSE = plot_compare_real_vs_perfect(IMU1_single_angle_diff, IMU2_single_angle_diff, IMU3_single_angle_diff, parent_dir)
#
# # Write final RMSE values to a csv
# final_RMSE_values_df = pd.DataFrame.from_dict(
#     {"Thorax IMU orientation error:": IMU1_single_angle_RMSE,
#      "Humerus IMU orientation error:": IMU2_single_angle_RMSE,
#      "Forearm IMU orientation error:": IMU3_single_angle_RMSE,}, orient='index')
# final_RMSE_values_df.to_csv(parent_dir + "\\" + "Real_vs_Perfect_Ori_Errors.csv",
#                             mode='w', encoding='utf-8', na_rep='nan')



""" Finding global misalignment """


def angular_velocities(q1, q2, dt):
    return (2 / dt) * np.array([
        q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
        q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
        q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])

qs = IMU3_df_Perfect.iloc[3800:3900].to_numpy()
real_qs = IMU3_df_Real.iloc[3800:3900].to_numpy()

ang_vel_arr_perfect = np.zeros(((len(qs)-10), 3))
for i in range(0, len(qs)-10):
    ang_vel = angular_velocities(qs[i], qs[i+10], 0.1)
    ang_vel_arr_perfect[i] = ang_vel

ang_vel_arr_real = np.zeros(((len(qs)-10), 3))
for i in range(0, len(qs)-10):
    ang_vel = angular_velocities(real_qs[i], real_qs[i+10], 0.1)
    ang_vel_arr_real[i] = ang_vel


# Calculate the angle between the two angular velocity vectors
diff_arr = np.zeros((len(ang_vel_arr_perfect)))
for i in range(len(ang_vel_arr_perfect)):
    ang_vel_perfect = ang_vel_arr_perfect[i]
    ang_vel_real = ang_vel_arr_real[i]
    diff = np.arccos(np.dot(ang_vel_perfect, ang_vel_real) /
                           (np.linalg.norm(ang_vel_perfect) * np.linalg.norm(ang_vel_real)))
    diff_arr[i] = diff*180/np.pi

print(diff_arr[:10])
print(np.nanmean(diff_arr))

# Animate the 3d vectors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(subplot_kw=dict(projection="3d"))

def get_arrow(theta, ang_vel_arr):
    x = 0
    y = 0
    z = 0
    u = ang_vel_arr[theta,0]
    v = ang_vel_arr[theta,1]
    w = ang_vel_arr[theta,2]
    return x,y,z,u,v,w

quiver_perfect = ax.quiver(*get_arrow(0, ang_vel_arr_perfect), color='red')
quiver_real = ax.quiver(*get_arrow(0, ang_vel_arr_real), color='blue')

plot_range = 1
ax.set_xlim(-plot_range, plot_range)
ax.set_ylim(-plot_range, plot_range)
ax.set_zlim(-plot_range, plot_range)

def update_perfect(theta):
    global quiver_perfect
    quiver_perfect.remove()
    quiver_perfect = ax.quiver(*get_arrow(theta, ang_vel_arr_perfect), color='red')

def update_real(theta):
    global quiver_real
    quiver_real.remove()
    quiver_real = ax.quiver(*get_arrow(theta, ang_vel_arr_real), color='blue')

ani_perfect = FuncAnimation(fig, update_perfect, frames=np.array(range(len(ang_vel_arr_perfect))), interval=10, repeat_delay=1000)
ani_real = FuncAnimation(fig, update_real, frames=np.array(range(len(ang_vel_arr_perfect))), interval=10, repeat_delay=1000)

plt.show()
