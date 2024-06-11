
import pandas as pd

# Read quaternion orientation data from a .txt TMM report file
def get_np_quats_from_txt_file(input_file):
    with open(input_file, 'r') as file:
        df = pd.read_csv(file, header=5, sep="\t")
    # Make seperate data_out frames
    IMU1_df = df.filter(["IMU1_Q0", "IMU1_Q1", "IMU1_Q2", "IMU1_Q3"], axis=1)
    IMU2_df = df.filter(["IMU2_Q0", "IMU2_Q1", "IMU2_Q2", "IMU2_Q3"], axis=1)
    IMU3_df = df.filter(["IMU3_Q0", "IMU3_Q1", "IMU3_Q2", "IMU3_Q3"], axis=1)
    IMU1_np = IMU1_df.to_numpy()
    IMU2_np = IMU2_df.to_numpy()
    IMU3_np = IMU3_df.to_numpy()
    return IMU1_np, IMU2_np, IMU3_np