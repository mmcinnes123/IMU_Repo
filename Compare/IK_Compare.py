# This script compares the IK results from IMU and OMC analyses
# Inputs are: two .mot files
# Outputs are: .png plots of each joint of interest

# Test change made in Master

import matplotlib.pyplot as plt
import numpy as np
import os
from functions import *
import scipy

""" SETTINGS """

# Quick Settings
trial_name = 'IMU_IMU_manual_cal'    # Tag to describe this trial
parent_dir = r"C:\Users\r03mm22\Documents\Protocol_Testing\Tests\24_01_22"  # Name of the working folder
start_time = 0
end_time = 37
results_dir = parent_dir + r"\Comparison13_manual_cal_IMUIMU"
create_new_ori_csvs = False     # Set this to False if you've already run this code and csv file has been created
labelA = "OMC"  # This is the label linked to all the variables with "OMC" in the title
labelB = "IMU"  # This is the label linked to all the variables with "IMU" in the title

# Define some file names
IMU_states_file = results_dir + "\\" + trial_name + '_StatesReporter_states.sto'
OMC_states_file = results_dir + r'\OMC_StatesReporter_states.sto'
path_to_IMU_model_file = r"C:\Users\r03mm22\Documents\Protocol_Testing\IMU_Repo\das3.osim"
path_to_OMC_model_file = parent_dir + r"\OMC\das3_scaled_and_placed.osim"
figure_results_dir = results_dir + "\\TimeRange_" + str(start_time) + "_" + str(end_time) + "s"
if os.path.exists(figure_results_dir) == False:
    os.mkdir(figure_results_dir)
osim.Logger.addFileSink(results_dir + r'\opensim.log')


""" MAIN """

# Read in states for states files
OMC_table = osim.TimeSeriesTable(OMC_states_file)
IMU_table = osim.TimeSeriesTable(IMU_states_file)
print(OMC_table.getNumRows())
print(IMU_table.getNumRows())

# Check if they're the same length and remove last row from OMC table if not.
if OMC_table.getNumRows() != IMU_table.getNumRows():
    OMC_table.removeRow((OMC_table.getNumRows() - 1) / 100)

# Calculate body orientations from states table for full recording period and write to csv file
if create_new_ori_csvs == True:
    extract_body_quats(OMC_table, path_to_OMC_model_file, results_dir, tag="OMC")
    extract_body_quats(IMU_table, path_to_IMU_model_file, results_dir, tag="IMU")


# Find the heading offset between IMU model thorax and OMC model thorax (as a descriptor of global frame offset) (read in untrimmed data)
thorax_OMC_all, humerus_OMC_all, radius_OMC_all = read_in_quats(start_time, end_time, file_name=results_dir + r"\OMC_quats.csv", trim_bool=False)
thorax_IMU_all, humerus_IMU_all, radius_IMU_all = read_in_quats(start_time, end_time, file_name=results_dir + r"\IMU_quats.csv", trim_bool=False)
heading_offset = find_heading_offset(thorax_OMC_all, thorax_IMU_all)

# Read in body orientations from newly created csv files (as trimmed np arrays (Nx4))
thorax_OMC, humerus_OMC, radius_OMC = read_in_quats(start_time, end_time, file_name=results_dir + r"\OMC_quats.csv", trim_bool=True)
thorax_IMU, humerus_IMU, radius_IMU = read_in_quats(start_time, end_time, file_name=results_dir + r"\IMU_quats.csv", trim_bool=True)

# Trim tables based on time of interest
OMC_table.trim(start_time, end_time)
IMU_table.trim(start_time, end_time)

time = OMC_table.getIndependentColumn()  # Get the time data

""" PLOT """

# Plot IMU vs OMC joint angles based on OpenSim coordinates
plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                     figure_results_dir, labelA, labelB, joint_of_interest="Thorax")

plot_compare_JAs(OMC_table, IMU_table, time, start_time, end_time,
                 figure_results_dir, labelA, labelB, joint_of_interest="Elbow")

plot_compare_JAs_shoulder_eulers(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                                 time, start_time, end_time, figure_results_dir, labelA, labelB)

plot_compare_body_oris(thorax_OMC, humerus_OMC, radius_OMC, thorax_IMU, humerus_IMU, radius_IMU,
                       heading_offset, time, start_time, end_time, figure_results_dir)

plot_vector_HT_angles(thorax_OMC, humerus_OMC, thorax_IMU, humerus_IMU,
                      time, start_time, end_time, figure_results_dir, labelA, labelB)