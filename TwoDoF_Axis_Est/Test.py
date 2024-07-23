
from os.path import join
import pandas as pd
import os
import numpy as np

# Read all_data and change the column headers

directory = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection'

input_file_path = join(directory, 'R Analysis', 'R 2DoF Opt', 'OptResultsForR.csv')
output_file_path = join(directory, 'R Analysis', 'R 2DoF Opt', 'OptResultsForR_new.csv')

with open(input_file_path, 'r') as file:
    Results_df = pd.read_csv(file, header=0, sep=',')

all_new_results = pd.DataFrame()

for row in range(len(Results_df)):

    Subject = Results_df.loc[row, 'Subject']
    Trial = Results_df.loc[row, 'Trial']
    OptMethod = Results_df.loc[row, 'OptMethod']
    IMUtype = Results_df.loc[row, 'IMUtype']
    for metric in ['HeadingOffset', 'SD_third_DoF', 'FEOptError', 'PSOptError']:
        Metric_name = metric
        Metric_value = Results_df.loc[row, metric]

        new_df_row = pd.DataFrame({'Subject': [Subject],
                                   'Trial': [Trial],
                                   'OptMethod': [OptMethod],
                                   'IMUtype': [IMUtype],
                                   'Metric': [Metric_name],
                                   'Metric_value': [Metric_value]})

        all_new_results = pd.concat([all_new_results, new_df_row], ignore_index=True)

all_new_results.to_csv(output_file_path)
