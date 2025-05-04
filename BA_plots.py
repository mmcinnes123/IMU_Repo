# This script creates BA plots for all subjects from the peak values of the JA_Slow trial
# Inputs are: all_IK_results.csv files for IMC and OMC
# Outputs are: .png plots of each joint of interest

from helpers_compare import get_range_dict
from helpers_compare import get_peaks_or_troughs
from constants import data_dir

import pandas as pd
import os
from os.path import join
import numpy as np
import matplotlib.pyplot as plt

fig_output_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\Results\Other_Figs_From_Python\Combined_BA_Plots'

if __name__ == '__main__':

    # Create dictionaries to store all peak/trough data across subjects
    all_subjects_data = {}  # Will store data for each joint across all subjects

    # Iterate through all subjects
    for subject_num in range(1, 21):
        subject_code = f'P{str(subject_num).zfill(3)}'  # Creates P001, P002, etc.
        calibration_name = 'OSIM_N_self'
        trial_name = 'JA_Slow'

        try:
            # Get the paths for this subject
            parent_dir = data_dir + '\\' + subject_code
            IMU_type_dir = join(parent_dir, 'Perfect')
            IMU_IK_results_dir = join(IMU_type_dir, 'IMU_IK_results_' + calibration_name, trial_name)
            OMC_IK_results_dir = join(parent_dir, 'OMC', 'JA_Slow_IK_Results')
            IMU_IK_results_file = join(IMU_IK_results_dir, 'all_IMU_IK_results.csv')
            OMC_IK_results_file = join(OMC_IK_results_dir, 'all_OMC_IK_results.csv')

            # Read the data
            OMC_angles = pd.read_csv(OMC_IK_results_file, encoding='utf-8', na_values='nan', index_col=None)
            IMU_angles = pd.read_csv(IMU_IK_results_file, encoding='utf-8', na_values='nan', index_col=None)

            # Get range dictionary
            JA_range_dict_file = os.path.join(parent_dir, subject_code + '_JA_range_dict.txt')
            range_dict = get_range_dict(JA_range_dict_file, IMU_angles, OMC_angles)

            # Process each joint
            excluded_columns = ['time', 'thorax_forward_tilt', 'thorax_lateral_tilt', 'thorax_rotation']
            for joint_name in [col for col in IMU_angles.columns if col not in excluded_columns]:
                if joint_name not in all_subjects_data:
                    all_subjects_data[joint_name] = {'errors': [], 'means': []}

                # Get angle data
                IMU_angle = IMU_angles[[joint_name]].to_numpy()
                OMC_angle = OMC_angles[[joint_name]].to_numpy()
                time = IMU_angles[['time']].to_numpy()

                # Get time range for this joint
                time_start, time_end = range_dict[joint_name][0], range_dict[joint_name][1]

                # Get peaks and troughs
                OMC_peaks, _, _ = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
                IMU_peaks, _, _ = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='peak', debug=False)
                OMC_troughs, _, _ = get_peaks_or_troughs(OMC_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)
                IMU_troughs, _, _ = get_peaks_or_troughs(IMU_angles, joint_name, time_start, time_end, peak_or_trough='trough', debug=False)

                if joint_name == 'HT_flexion':
                    print(f'Subject: {subject_code}, Trough Error: {IMU_troughs - OMC_troughs})')
                # Combine peaks and troughs
                IMU_peaks_and_troughs = np.concatenate((IMU_peaks, IMU_troughs))
                OMC_peaks_and_troughs = np.concatenate((OMC_peaks, OMC_troughs))

                # Calculate errors and means
                errors = IMU_peaks_and_troughs - OMC_peaks_and_troughs
                means = (IMU_peaks_and_troughs + OMC_peaks_and_troughs) / 2

                # Store the data
                all_subjects_data[joint_name]['errors'].extend(errors)
                all_subjects_data[joint_name]['means'].extend(means)

        except Exception as e:
            print(f"Error processing subject {subject_code}: {str(e)}")
            continue

    # Create combined Bland-Altman plots for each joint
    # Create combined Bland-Altman plots for each joint
    for joint_name, data in all_subjects_data.items():
        errors = np.array(data['errors'])
        means = np.array(data['means'])

        # Calculate statistics
        bias = np.mean(errors)
        std_diff = np.std(errors)
        upper_loa = bias + (1.96 * std_diff)
        lower_loa = bias - (1.96 * std_diff)

        # Calculate regression line
        slope, intercept = np.polyfit(means, errors, 1)
        regression_line = slope * means + intercept
        r = np.corrcoef(means, errors)[0,1]  # R instead of R²

        # Create plot
        plt.rcParams.update({'font.family': 'Times New Roman', 'font.size': 18})  # Change 14 to your desired font size
        plt.figure(figsize=(10, 7))
        plt.scatter(means, errors, alpha=0.5)
        plt.plot(means, regression_line, 'b--',
                 label=f'Regression line (R = {r:.3f})')
        plt.axhline(y=bias, color='k', linestyle='-')
        plt.axhline(y=upper_loa, color='r', linestyle='--')
        plt.axhline(y=lower_loa, color='r', linestyle='--')
        plt.legend()

        # Add annotations for LOA lines
        x_min, x_max = plt.xlim()
        plt.text(x_max + 1, upper_loa, f'Upper LoA',
                 verticalalignment='center')
        plt.text(x_max + 1, lower_loa, f'Lower LoA',
                 verticalalignment='center')
        plt.text(x_max + 1, bias, f'Bias',)

        custom_labels = {
            'HT_flexion': 'Shoulder Flexion',
            'HT_abd': 'Shoulder Abduction',
            'HT_rotation': 'Shoulder Rotation',
            'elbow_flexion': 'Elbow Flexion',
            'elbow_pronation': 'Forearm Pronation'
        }

        # Use the dictionary to set the label, with a fallback to the default formatting
        label = custom_labels.get(joint_name)
        plt.xlabel('Mean Joint Angle (OMC + IMC / 2)')
        plt.ylabel('Error (IMC - OMC)')
        plt.title(f'Bland-Altman Plot\n{label}')
        plt.axhline(y=0, color='gray', linestyle='-', alpha=0.5, label='y=0 Line')

        # Set y-axis limits
        plt.ylim(-40, 50)

        # Adjust layout to accommodate annotations
        plt.tight_layout()

        # Save the figure
        os.makedirs(fig_output_dir, exist_ok=True)
        plt.savefig(join(fig_output_dir, f'combined_bland_altman_{joint_name}.png'),
                    bbox_inches='tight')  # Ensures annotations are not cut off
        plt.close()