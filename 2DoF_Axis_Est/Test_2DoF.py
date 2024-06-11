

from helpers_2DoF import get_np_quats_from_txt_file

import qmt
import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)


# Read in some IMU quaternion data from a TMM report .txt file
# input_txt_file = str(askopenfilename(title=' Choose the raw data .txt file with IMU/quat data ... '))
input_txt_file = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\P1\RawData\P1_JA_Slow - Report2 - IMU_Quats.txt'
IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(input_txt_file)

all_quats = IMU1_np

axis = [0, 1, 0]

print(all_quats[:10])

quat3 = qmt.quatProject(all_quats, axis, plot=True)

