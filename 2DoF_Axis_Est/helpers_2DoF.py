
import pandas as pd
import qmt
import numpy as np
import matplotlib.pyplot as plt

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



def get_ang_vels_from_quats(quats, sample_rate, debug_plot):

    """ Function to calculate 3D angular velocity vectors (in the IMU's global frame (Ei), from IMU orientation quats.
    The following is based on equations A41 and A42 in Appendix C """

    dt = 1/sample_rate

    # Calculate quaternion changes from i - 1 to i
    q_change = qmt.qmult(quats[1:], qmt.qinv(quats[:-1]))

    # Use this function to stop the quat suddenly flipping from close to [1, 0, 0, 0] to [-1, 0, 0, 0]
    q_change_unwraped = qmt.quatUnwrap(q_change)

    # Breakdown equation A42 into steps
    q_w = q_change_unwraped[:, 0]
    q_xyz = q_change_unwraped[:, 1:]
    mult_factor = (2 / dt) * (np.arccos(np.clip(q_w, -1, 1)) / np.linalg.norm(q_xyz, axis=1))
    ang_vels = mult_factor[:, np.newaxis] * q_xyz

    if debug_plot:
        print("Animating input quaternion data...")
        visualise_quat_data(quats, sample_rate)
        print("Plotting output angular velocity vectors...")
        plot_gyr_data(ang_vels, sample_rate)

    return ang_vels

def plot_gyr_data(gyr, rate):

    # Sampling interval (in seconds)
    sampling_interval = 1 / rate  # 20Hz

    # Generate the time array
    time = np.arange(len(gyr)) * sampling_interval

    # Plotting
    plt.figure(figsize=(12, 6))

    plt.plot(time, gyr[:, 0], label='X component', color='r')
    plt.plot(time, gyr[:, 1], label='Y component', color='g')
    plt.plot(time, gyr[:, 2], label='Z component', color='b')

    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity')
    plt.title('Angular Velocity 3D Vectors')
    plt.legend()
    plt.grid(True)
    plt.show()


def visualise_quat_data(quats, rate):
    t = qmt.timeVec(N=len(quats), rate=rate)    # Make a tiem vec based on the length of the quats
    data = qmt.Struct(t=t, quat=quats)
    # run webapp
    webapp = qmt.Webapp('/view/imubox', data=data)
    webapp.run()