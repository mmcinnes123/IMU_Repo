
import pandas as pd
import qmt
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


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


def get_joint_axis_directly_from_ang_vels(quatsIMU1, quatsIMU2, rate, params, debug_plot):

    # Set the defaults, update with params
    defaults = dict(jointAxis='j1', gyrCutoff=5, downsampleRate=20)
    params = qmt.setDefaults(params, defaults)
    gyrCutoff = params['gyrCutoff']
    downsampleRate = params['downsampleRate']
    jointAxis = params['jointAxis']
    assert jointAxis in ('j1', 'j2')


    # Check the quaternion array is the correct shape
    N = quatsIMU1.shape[0]
    N = quatsIMU2.shape[0]
    assert quatsIMU1.shape == (N, 4)
    assert quatsIMU2.shape == (N, 4)

    # Down-sample the orientation data
    if rate == downsampleRate or downsampleRate is None:
        ind = slice(None)
    else:
        assert downsampleRate < rate
        M = int(round(N*downsampleRate/rate))
        ind = np.linspace(0, N-1, M, dtype=int)
    q1 = quatsIMU1[ind].copy()
    q2 = quatsIMU2[ind].copy()

    if jointAxis == 'j1':
        # First express IMU2 in IMU2 frame
        q2_in1 = qmt.qmult(qmt.qinv(q1), q2)
        # Then get angular velocities of IMU2, expressed in IMU1 frame
        angVels = get_ang_vels_from_quats(q2_in1, downsampleRate, debug_plot=False)
        # Change the sign of all elements whenever the z-element becomes negative (keeping axis always pointing pos y dir)
        angVels[angVels[:, 2] < 0] *= -1

    else:
        # Get angular velocities of IMU2 (in local IMU frame) from the down-sampled orientation data
        angVels = get_local_ang_vels_from_quats(q2, downsampleRate, debug_plot=False)
        # Change the sign of all elements whenever the y-element becomes negative (keeping axis always pointing pos y dir)
        angVels[angVels[:, 1] < 0] *= -1

    # Normalise the angVels
    angVels = angVels / np.linalg.norm(angVels, axis=1)[:, np.newaxis]
    angVels = np.nan_to_num(angVels)    # Replace any nans with 0 to allow filter to run

    # Apply a butterworth low pass filter to the angular velocity data
    # (The gyrCutoff is the cut-off frequency used to filter the angular rates (used in the rotation constraint))
    if gyrCutoff is not None:  # apply Butterworth low pass filter
        if 2 * gyrCutoff <= 0.95 * downsampleRate:  # do not filter if (almost) at Nyquist frequency
            b, a = signal.butter(2, gyrCutoff * 2 / downsampleRate)
            angVels = signal.filtfilt(b, a, angVels, axis=0)

    # Average the ang vel 3D vectors
    avg_ang_vel = np.nanmean(angVels, axis=0)

    if debug_plot:
        print("Animating first input quaternion data:")
        visualise_quat_data(quatsIMU1, rate)
        print("Animating second input quaternion data:")
        visualise_quat_data(quatsIMU2, rate)
        print("Plotting normalised/filtered angular velocity vectors:")
        plot_gyr_data(angVels, rate)

    return avg_ang_vel


def get_local_ang_vels_from_quats(quats, sample_rate, debug_plot):
    """ Function to calculate 3D angular velocity vectors (in the IMU's local frame, from IMU orientation quats."""

    dt = 1 / sample_rate

    # Calculate quaternion changes from i - 1 to i
    q_change = qmt.qmult(qmt.qinv(quats[:-1]), quats[1:])

    # Use this function to stop the quat suddenly flipping from close to [1, 0, 0, 0] to [-1, 0, 0, 0]
    q_change_unwraped = qmt.quatUnwrap(q_change)

    # Breakdown equation A42 into steps
    q_w = q_change_unwraped[:, 0]
    q_xyz = q_change_unwraped[:, 1:]
    mult_factor = (2 / dt) * (np.arccos(np.clip(q_w, -1, 1)) / np.linalg.norm(q_xyz, axis=1))
    ang_vels = mult_factor[:, np.newaxis] * q_xyz

    if debug_plot:
        print("Plotting output angular velocity vectors...")
        plot_gyr_data(ang_vels, sample_rate)

    return ang_vels





