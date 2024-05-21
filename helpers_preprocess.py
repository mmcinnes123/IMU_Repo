# Functions used to run 1_preprocess.py

import opensim as osim
import numpy as np
import pandas as pd


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