# Functions used to run 1_preprocess.py

from constants import APDM_settings_file, APDM_template_file, sample_rate

import opensim as osim
import numpy as np
import pandas as pd
from os.path import join
import ast



# Read all data_out in from specified input file
def read_data_frame_from_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)
    return IMU1_df, IMU2_df, IMU3_df


def extract_cal_row(df, cal_time, sample_rate):
    first_index = int(cal_time*sample_rate)
    last_index = first_index + 1
    index_range = list(range(first_index, last_index))
    df_new = df.iloc[index_range, :]
    df_new_new = df_new.reset_index(drop=True)
    return df_new_new

# Write new data_out to APDM file template
def write_to_APDM(df_1, df_2, df_3, df_4, template_file, output_dir, tag):

    # Make columns of zeros
    N = len(df_1)
    zeros_25_df = pd.DataFrame(np.zeros((N, 25)))
    zeros_11_df = pd.DataFrame(np.zeros((N, 11)))
    zeros_2_df = pd.DataFrame(np.zeros((N, 2)))

    # Make a dataframe with zeros columns inbetween the data_out
    IMU_and_zeros_df = pd.concat([zeros_25_df, df_1, zeros_11_df, df_2, zeros_11_df, df_3, zeros_11_df, df_4, zeros_2_df], axis=1)

    # Read in the APDM template and save as an array
    with open(template_file, 'r') as file:
        template_df = pd.read_csv(file, header=0)
        template_array = template_df.to_numpy()

    # Concatenate the IMU_and_zeros and the APDM template headings
    IMU_and_zeros_array = IMU_and_zeros_df.to_numpy()
    new_array = np.concatenate((template_array, IMU_and_zeros_array), axis=0)
    new_df = pd.DataFrame(new_array)

    # Add the new dataframe into the template
    new_df.to_csv(output_dir + "\\" + tag + ".csv", mode='w', index=False, header=False, encoding='utf-8', na_rep='nan')


def APDM_2_sto_Converter(APDM_settings_file, input_file_name, output_file_name):

    # Build an APDM Settings Object
    # Instantiate the Reader Settings Class
    APDMSettings = osim.APDMDataReaderSettings(APDM_settings_file)
    # Instantiate an APDMDataReader
    APDM = osim.APDMDataReader(APDMSettings)

    # Read in table of movement data_out from the specified IMU file
    table = APDM.read(input_file_name)
    # Get Orientation Data as quaternions
    quatTable = APDM.getOrientationsTable(table)

    # Write to file
    osim.STOFileAdapterQuaternion.write(quatTable, output_file_name)


# Function to extract quaternion orientation data from .txt file and save as .sto file
def write_movements_and_calibration_stos(input_file_path, cal_pose_time_dict, IMU_type, trial_results_dir):

    # Read data from TMM .txt report
    IMU1_df, IMU2_df, IMU3_df = read_data_frame_from_file(input_file_path)

    # If IMU data last (or 2nd last) row has nans, remove rows
    def remove_last_row_if_nans(df):
        for _ in range(2):
            # Check if the last row has any NaNs
            if df.iloc[-1].isnull().any():
                # Remove the last row if it contains NaNs
                df = df.iloc[:-1]
        return df
    IMU1_df = remove_last_row_if_nans(IMU1_df)
    IMU2_df = remove_last_row_if_nans(IMU2_df)
    IMU3_df = remove_last_row_if_nans(IMU3_df)


    """ Write full trial to .sto """

    # Write data to APDM format .csv
    file_tag = IMU_type + '_Quats_all'
    write_to_APDM(IMU1_df, IMU2_df, IMU3_df, IMU3_df, APDM_template_file, trial_results_dir, file_tag)

    # Write data to .sto using OpenSim APDM converter tool
    APDM_2_sto_Converter(APDM_settings_file, input_file_name=trial_results_dir + "\\" + file_tag + ".csv",
                         output_file_name=trial_results_dir + "\\" + file_tag + ".sto")


    """ Write each pose to .sto, based on time_stamp """

    # Iterate through list of calibration poses and associated times to create separate .sto files
    for pose_name in cal_pose_time_dict:

        if pose_name.startswith(('N', 'Alt')) and cal_pose_time_dict[pose_name] is not None:

            cal_pose_time = cal_pose_time_dict[pose_name]   # The time at which to extract the data

            print('for pose: ', pose_name)
            print('at time: ', cal_pose_time)

            # Extract one row based on time of calibration pose
            IMU1_cal_df = extract_cal_row(IMU1_df, cal_pose_time, sample_rate)
            IMU2_cal_df = extract_cal_row(IMU2_df, cal_pose_time, sample_rate)
            IMU3_cal_df = extract_cal_row(IMU3_df, cal_pose_time, sample_rate)

            # Write data to APDM format .csv
            file_tag = IMU_type + '_Quats_' + str(pose_name)
            write_to_APDM(IMU1_cal_df, IMU2_cal_df, IMU3_cal_df, IMU3_cal_df, APDM_template_file, trial_results_dir, file_tag)

            # Write data to .sto using OpenSim APDM converter tool
            APDM_2_sto_Converter(APDM_settings_file, input_file_name=trial_results_dir + "\\" + file_tag + ".csv",
                                 output_file_name=trial_results_dir + "\\" + file_tag + ".sto")

def get_trial_pose_time_dict_from_file(directory, subject_code):
    # Read the existing txt file to get the dict
    file_obj = open(join(directory, 'SubjectEventFiles', subject_code + '_event_dict.txt'), 'r')
    content = file_obj.read()
    trial_pose_time_dict = ast.literal_eval(content)
    file_obj.close()
    return trial_pose_time_dict
