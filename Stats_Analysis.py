import numpy as np
import pandas as pd
import os


directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'
list_of_subjects = ['P1', 'P2', 'P3']
trial_name = 'JA_Slow'


# Function to read the results form csv
def read_from_csv(file_path):
    with open(file_path, 'r') as file:
        df = pd.read_csv(file, index_col=0, header=1, sep=',', dtype={1: np.float64})
    # Make seperate data_out frames
    RMSE_thorax_ori = df.filter(["RMSE_thorax_ori"], axis=0).iloc[0, 0]
    RMSE_humerus_ori = df.filter(["RMSE_humerus_ori"], axis=0).iloc[0, 0]
    RMSE_radius_ori = df.filter(["RMSE_radius_ori"], axis=0).iloc[0, 0]
    RMSE_thorax_forward_tilt = df.filter(["RMSE_thorax_forward_tilt"], axis=0).iloc[0, 0]
    RMSE_thorax_lateral_tilt = df.filter(["RMSE_thorax_lateral_tilt"], axis=0).iloc[0, 0]
    RMSE_thorax_rotation = df.filter(["RMSE_thorax_rotation"], axis=0).iloc[0, 0]
    RMSE_elbow_flexion = df.filter(["RMSE_elbow_flexion"], axis=0).iloc[0, 0]
    RMSE_elbow_pronation = df.filter(["RMSE_elbow_pronation"], axis=0).iloc[0, 0]
    RMSE_HT_abd = df.filter(["RMSE_HT_abd"], axis=0).iloc[0, 0]
    RMSE_HT_flexion = df.filter(["RMSE_HT_flexion"], axis=0).iloc[0, 0]
    RMSE_HT_rotation = df.filter(["RMSE_HT_rotation"], axis=0).iloc[0, 0]
    RMSE_HT_Y_plane_of_elevation = df.filter(["RMSE_HT_Y_plane_of_elevation"], axis=0).iloc[0, 0]
    RMSE_HT_Z_elevation = df.filter(["RMSE_HT_Z_elevation"], axis=0).iloc[0, 0]
    RMSE_HT_YY_rotation = df.filter(["RMSE_HT_YY_rotation"], axis=0).iloc[0, 0]

    RMSEs = {
        "thorax_ori": RMSE_thorax_ori,
        "humerus_ori": RMSE_humerus_ori,
        "radius_ori": RMSE_radius_ori,
        "elbow_flexion": RMSE_elbow_flexion,
        "pronation": RMSE_elbow_pronation,
        "thorax_forward_tilt": RMSE_thorax_forward_tilt,
        "thorax_lateral_tilt": RMSE_thorax_lateral_tilt,
        "thorax_rotation": RMSE_thorax_rotation,
        "HT_abd": RMSE_HT_abd,
        "HT_flexion": RMSE_HT_flexion,
        "HT_rotation": RMSE_HT_rotation,
        "HT_Y": RMSE_HT_Y_plane_of_elevation,
        "HT_Z": RMSE_HT_Z_elevation,
        "HT_YY": RMSE_HT_YY_rotation
    }

    return RMSEs



""" CREATING DATAFRAME FOR R"""

all_data = pd.DataFrame(columns=['Subject', 'Trial', 'CalibrationName', 'JA', 'RMSE'])


calibration_name_dict = {'ALL_MANUAL': None,
                         'ALL_POSE_BASED_N_self': None,
                         'ALL_POSE_BASED_Alt_self': None,
                         'METHOD_1': None,
                         'METHOD_2': None}

# Get the results for each calibration method
for calibration_name in calibration_name_dict:

    for subject_code in list_of_subjects:

        # Define the file paths
        comparison_folder = 'Comparison_' + subject_code + '_' + calibration_name + '_' + trial_name
        file_name = subject_code + '_' + calibration_name + '_' + trial_name + '_Final_RMSEs.csv'
        file_path = os.path.join(directory, subject_code, 'IMU_IK_results_' + calibration_name, trial_name,
                                 comparison_folder, file_name)

        # Read RMSEs from the CSV file
        RMSEs_dict = read_from_csv(file_path)

        for key, value in RMSEs_dict.items():
            new_row = pd.DataFrame({'Subject': [subject_code], 'Trial': [trial_name], 'CalibrationName': [calibration_name], 'JA': [key], 'RMSE': [value]})
            all_data = pd.concat([all_data, new_row], ignore_index=True)

all_data.to_csv(os.path.join(directory,'R Analysis', 'AllResults_forR.csv'))



# """ ALTERNATIVE METHOD FOR CREATING DICT FOR PLOTTING IN PYTHON """
#
# # Function to get the average and sd of the RMSEs for each JA, over all the subjects
# def get_avg_RMSEs(calibration_name):
#
#     # Create an empty dictionary to store list of RMSEs
#     all_subject_RMSEs = {"thorax_ori": [], "humerus_ori": [], "radius_ori": [],
#                          "elbow_flexion": [], "pronation": [],
#                          "thorax_forward_tilt": [], "thorax_lateral_tilt": [], "thorax_rotation": [],
#                          "HT_abd": [], "HT_flexion": [], "HT_rotation": [],
#                          "HT_Y": [], "HT_Z": [], "HT_YY": []}
#
#     # Iterate through all the subjects (get RMSE for every JA of interest)
#     for subject_code in list_of_subjects:
#
#         # Define the file paths
#         comparison_folder = 'Comparison_' + subject_code + '_' + calibration_name + '_' + trial_name
#         file_name = subject_code + '_' + calibration_name + '_' + trial_name + '_Final_RMSEs.csv'
#         file_path = os.path.join(directory, subject_code, 'IMU_IK_results_' + calibration_name, trial_name,
#                                  comparison_folder, file_name)
#
#         # Read RMSEs from the CSV file
#         RMSEs_dict = read_from_csv(file_path)
#
#         for key, value in RMSEs_dict.items():
#             all_subject_RMSEs[key].append(value)
#
#     return all_subject_RMSEs
#
# calibration_name_dict = {'ALL_MANUAL': None,
#                          'ALL_POSE_BASED_N_self': None,
#                          'ALL_POSE_BASED_Alt_self': None}
#
#
# # Get the results for each calibration method
# for calibration_name in calibration_name_dict:
#     calibration_name_dict[calibration_name] = get_avg_RMSEs(calibration_name)




# # Calculate averages and standard deviations
# averages = {key + "_avg_RMSE": np.mean(values) for key, values in all_subject_RMSEs.items()}
# standard_deviations = {key + "_sd_RMSE": np.std(values) for key, values in all_subject_RMSEs.items()}

#
# from matplotlib import pyplot as plt
#
# x = np.array([1,2,3,5,6,7])
# l = ['Manual','N_pose','Alt_pose','Manual','N_pose','Alt_pose']
# a = calibration_name_dict['ALL_MANUAL']['elbow_flexion']
# b = calibration_name_dict['ALL_POSE_BASED_N_self']['elbow_flexion']
# c = calibration_name_dict['ALL_POSE_BASED_Alt_self']['elbow_flexion']
# d = calibration_name_dict['ALL_MANUAL']['pronation']
# e = calibration_name_dict['ALL_POSE_BASED_N_self']['pronation']
# f = calibration_name_dict['ALL_POSE_BASED_Alt_self']['pronation']
#
# plt.subplot(111)
# plt.margins(0.2)
# plt.xticks(x,l)
# plt.plot([1, 1, 1], a, 'ro', label='Manual')
# plt.plot([2, 2, 2], b, 'ro', label='N_pose')
# plt.plot([3, 3, 3], c, 'ro', label='Alt_pose')
# plt.plot([5, 5, 5], d, 'ro', label='Manual')
# plt.plot([6, 6, 6], e, 'ro', label='N_pose')
# plt.plot([7, 7, 7], f, 'ro', label='Alt_pose')
# plt.tight_layout()
# plt.savefig('vertical_scatter')
# plt.close()
#
# import matplotlib.pyplot as plt
# import numpy as np
#
# # Sample data
# np.random.seed(123)
# x = np.repeat(np.arange(1, 6), 20)  # Discrete intervals on the x-axis
# y = np.random.normal(size=100)      # Random y values
#
# # Add jitter to x-coordinates
# jitter = np.random.normal(scale=0.1, size=len(x))  # Adjust scale to control spread along x-axis
# x_jittered = x
#
# # Create scatter plot with jittered points
# plt.figure(figsize=(8, 6))
# plt.scatter(x_jittered, y, alpha=0.5)
# plt.xlabel('X-axis label')
# plt.ylabel('Y-axis label')
# plt.title('Scatter Plot with Jittered Points')
# plt.grid(True)
# plt.show()