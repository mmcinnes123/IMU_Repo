
import pandas as pd
import qmt
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import opensim as osim
from scipy.spatial.transform import Rotation as R
from os.path import join


def get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt, opt_method, subject_event_dict, sample_rate, debug):

    assert IMU_type_for_opt in ['Real', 'Perfect'], 'IMU type not Real or Perfect'

    if IMU_type_for_opt == 'Perfect':
        report_ext = ' - Report3 - Cluster_Quats.txt'
    elif IMU_type_for_opt == 'Real':
        report_ext = ' - Report2 - IMU_Quats.txt'
    else:
        report_ext = None

    # Get the .txt file with quats based on subject code, whcih trial to use, and which type of IMU
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    raw_data_dir = join(parent_dir, 'RawData')
    tmm_txt_file_name = subject_code + '_' + trial_for_opt + report_ext
    tmm_txt_file = join(raw_data_dir, tmm_txt_file_name)

    # Read in the IMU quaternion data from a TMM report .txt file
    IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(tmm_txt_file)

    # Get the start and end time for which to run the optimisation
    FE_start_time = subject_event_dict[trial_for_opt]['FE_start']
    PS_end_time = subject_event_dict[trial_for_opt]['PS_end']

    # Trim the IMU data based on the period of interest
    start_ind = FE_start_time * sample_rate
    end_ind = PS_end_time * sample_rate
    IMU2_trimmed = IMU2_np[start_ind:end_ind]
    IMU3_trimmed = IMU3_np[start_ind:end_ind]

    # Run the optimisation function
    from TwoDoF_Axis_Est.joint_axis_est_2d import jointAxisEst2D

    params = dict(method=opt_method)
    opt_results = jointAxisEst2D(IMU2_trimmed, IMU3_trimmed, None, None, sample_rate, params=params, debug=True, plot=False)
    FE = opt_results['j1']
    PS = opt_results['j2']

    # Constrain the Opt FE axis to always point laterally (z component should be positive)
    if np.sign(FE[2]) == -1:  # Constrain based on the z-component, expected to be largest
        FE = -FE
    # Constrain the Opt PS axis to always point proximally (y component should be negative)
    if np.sign(PS[1]) == 1:  # Constrain based on the z-component, expected to be largest
        PS = -PS

    if debug:
        print('get_J1_J2_from_opt() DEBUG:')
        print('\tTMM file used: ', tmm_txt_file)
        print('\tBetween times: ', start_ind/sample_rate, end_ind/sample_rate)
        print('\tUsing optimistaion method: ', opt_method)
        print('\tComplete optimisation results: ')
        print(opt_results)

        # Get variation in movement of each IMU
        def get_ori_variation(IMU_quats):
            quat_changes = np.zeros((len(IMU_quats), 4))
            for row in range(len(IMU_quats)):
                quat_changes[row] = qmt.qmult(qmt.qinv(IMU_quats[row]), IMU_quats[0])  # Get orientation change from q0, for all t
            angles = qmt.quatAngle(quat_changes, plot=False)  # Get the magnitude of the change from q0
            mean_diff = np.mean(np.array(abs(angles)))  # Get the average change in ori from q0
            return mean_diff

        IMU2_mean_diff = get_ori_variation(IMU2_trimmed)
        IMU3_mean_diff = get_ori_variation(IMU3_trimmed)
        print('\tMean variation in orientation Humerus IMU: ', IMU2_mean_diff * 180 / np.pi)
        print('\tMean variation in orientation Radius IMU: ', IMU3_mean_diff * 180 / np.pi)

    return FE, PS, opt_results


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


def get_local_ang_vels_from_quats(quats, sample_rate, debug_plot):
    """ Function to calculate 3D angular velocity vectors (in the IMU's local frame, from IMU orientation quats."""

    # Calculate quaternion changes from i - 1 to i
    q_change = qmt.qmult(qmt.qinv(quats[:-1]), quats[1:])

    # Use this function to stop the quat suddenly flipping from close to [1, 0, 0, 0] to [-1, 0, 0, 0]
    q_change = qmt.posScalar(q_change)

    # Get ang vel from array of q_change (equivalent to equation A42)
    ang_vels = qmt.quatToRotVec(q_change) * sample_rate

    if debug_plot:
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


def get_J1_J2_from_calibrated_OMC_model(model_file, debug):

    """Get the cluster frames, expressed relative to the body frames, specific to the subject's model"""

    # Read in calibrated model file to get position of humerus markers in humerus body frame
    my_model = osim.Model(model_file)
    marker_1_in_hum = my_model.getMarkerSet().get('Hum_Clus_1').get_location().to_numpy()
    marker_3_in_hum = my_model.getMarkerSet().get('Hum_Clus_3').get_location().to_numpy()
    marker_4_in_hum = my_model.getMarkerSet().get('Hum_Clus_4').get_location().to_numpy()
    # And radius markers in radius body frame
    marker_1_in_rad = my_model.getMarkerSet().get('Fore_Clus_1').get_location().to_numpy()
    marker_2_in_rad = my_model.getMarkerSet().get('Fore_Clus_2').get_location().to_numpy()
    marker_4_in_rad = my_model.getMarkerSet().get('Fore_Clus_4').get_location().to_numpy()

    # Humerus cluster CF expressed in the humerus CF, using the marker positions
    hum_clus_y_axis = marker_1_in_hum - marker_4_in_hum     # y_axis is marker 4 to marker 1 (pointing down)
    hum_clus_x_axis = marker_3_in_hum - marker_4_in_hum     # x_axis is marker 4 to marker 3 (pointing backwards)
    hum_clus_in_hum = qmt.quatFrom2Axes(hum_clus_x_axis, hum_clus_y_axis, None, plot=False)

    # Forearm cluster CF expressed in the radius CF, using the marker positions
    rad_clus_y_axis = marker_2_in_rad - marker_1_in_rad     # y_axis is marker 1 to marker 2 (pointing down)
    rad_clus_x_axis = marker_1_in_rad - marker_4_in_rad     # x_axis is marker 4 to marker 1 (pointing backwards)
    rad_clus_in_rad = qmt.quatFrom2Axes(rad_clus_x_axis, rad_clus_y_axis, None, plot=False)

    """Get the FE axis expressed in the model's humerus body frame, based on default model"""

    # Based on how the hu joint is defined in the model, the XYZ euler ori offset of the parent frame,
    # relative to humerus frame is:
    hu_parent_rel2_hum_R = R.from_euler('XYZ', [0, 0, 0.32318], degrees=False)

    # Based on how the hu joint is defined in the model, relative to the hu joint parent frame,
    # the vector of hu rotation axis (EL_x) is:
    FE_axis_rel2_hu_parent = [0.969, -0.247, 0]

    # Get the vector of hu rotation axis, relative to the humerus frame
    FE_axis_in_humerus = hu_parent_rel2_hum_R.apply(FE_axis_rel2_hu_parent)

    # Now express the FE axis in the cluster frame
    FE_in_hum_clus = qmt.rotate(hum_clus_in_hum, FE_axis_in_humerus, plot=False)

    """ Get the PS axis expressed in the model's radius body frame, based on default model """

    # PS_axis of the ur joint is defined relative to the parent/child frames, where the child frame = radius body frame
    PS_axis_in_radius = [0.182, 0.98227, -0.044946]

    # Now express the PS axis in the forearm cluster frame
    PS_in_rad_clus = qmt.rotate(rad_clus_in_rad, PS_axis_in_radius, plot=False)

    if debug:
        print('get_J1_J2_from_calibrated_OMC_model() DEBUG:')
        print('\tModel file used: ', model_file)
        print('\tHum_Clus Marker 1 position: ', np.array_str(marker_1_in_hum))
        print('\tFore_Clus Marker 1 position: ', np.array_str(marker_1_in_rad))
        print('\tFore_Clus Marker 2 position: ', np.array_str(marker_2_in_rad))
        print('')
        print('\tFE axis in humerus body frame: ', FE_axis_in_humerus)
        print('\tHumerus cluster in humerus body frame: ', hum_clus_in_hum)
        print('\tFE axis in humerus cluster frame: ', FE_in_hum_clus)
        print('')
        print('\tPS axis in radius body frame: ', PS_axis_in_radius)
        print('\tRadius cluster in radius body frame: ', rad_clus_in_rad)
        print('\tPS axis in radius cluster frame: ', PS_in_rad_clus)

    return FE_in_hum_clus, PS_in_rad_clus


def get_J1_J2_from_isolate_move(subject_code, IMU_type_for_opt, trial_for_opt, subject_time_dict, sample_rate, debug):

    assert IMU_type_for_opt in ['Real', 'Perfect'], 'IMU type not Real or Perfect'

    if IMU_type_for_opt == 'Perfect':
        report_ext = ' - Report3 - Cluster_Quats.txt'
    elif IMU_type_for_opt == 'Real':
        report_ext = ' - Report2 - IMU_Quats.txt'
    else:
        report_ext = None

    # Get the .txt file with quats based on subject code, whcih trial to use, and which type of IMU
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    raw_data_dir = join(parent_dir, 'RawData')
    tmm_txt_file_name = subject_code + '_' + trial_for_opt + report_ext
    tmm_txt_file = join(raw_data_dir, tmm_txt_file_name)

    # Read in the IMU quaternion data from a TMM report .txt file
    IMU1_np, IMU2_np, IMU3_np = get_np_quats_from_txt_file(tmm_txt_file)

    # Get the start and end time for which to run the optimisation
    FE_start_time = subject_time_dict[trial_for_opt]['FE_start']
    FE_end_time = subject_time_dict[trial_for_opt]['FE_end']
    PS_start_time = subject_time_dict[trial_for_opt]['PS_start']
    PS_end_time = subject_time_dict[trial_for_opt]['PS_end']

    # Get the FE axis based on FE movement sample
    start_ind_FE = FE_start_time * sample_rate
    end_ind_FE = FE_end_time * sample_rate
    IMU2_trimmed_FE = IMU2_np[start_ind_FE:end_ind_FE]
    IMU3_trimmed_FE = IMU3_np[start_ind_FE:end_ind_FE]
    FE_params = dict(jointAxis='j1')
    FE_axis = get_J1_J2_directly_from_ang_vels(IMU2_trimmed_FE, IMU3_trimmed_FE, sample_rate, params=FE_params,
                                               debug_plot=False)

    # Get the FE axis based on FE movement sample
    start_ind_PS = PS_start_time * sample_rate
    end_ind_PS = PS_end_time * sample_rate
    IMU2_trimmed_PS = IMU2_np[start_ind_PS:end_ind_PS]
    IMU3_trimmed_PS = IMU3_np[start_ind_PS:end_ind_PS]
    PS_params = dict(jointAxis='j1')
    PS_axis = get_J1_J2_directly_from_ang_vels(IMU2_trimmed_PS, IMU3_trimmed_PS, sample_rate, params=PS_params,
                                               debug_plot=False)

    if debug:
        print('get_J1_J2_from_isolate_move() DEBUG:')
        print('\tTMM file used: ', tmm_txt_file)
        print('\tBetween times (FE): ', start_ind_FE/sample_rate, end_ind_FE/sample_rate)
        print('\tBetween times (PS): ', start_ind_PS/sample_rate, end_ind_PS/sample_rate)

    return FE_axis, PS_axis


def get_J1_J2_directly_from_ang_vels(quatsIMU1, quatsIMU2, rate, params, debug_plot):

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
    angVels /= np.linalg.norm(angVels, axis=1)[:, np.newaxis]
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


def visulalise_opt_result_vec_on_IMU(vec1, vec2, vec3):

    # scale vec2 so it displays better
    vec2 = 6*vec2

    if vec3 is None:
        vec3 = [0, 0, 0]
    else:
        vec3 = 40*vec3

    # Test visualising vector
    N = 100
    gyr = [vec1 for _ in range(N)]
    acc = [vec2 for _ in range(N)]
    mag = [vec3 for _ in range(N)]
    quat = [[-0.70738827, 0.70682518, 0, 0] for _ in range(N)]

    t = qmt.timeVec(N=len(quat), rate=100)  # Make a tiem vec based on the length of the quats
    data = qmt.Struct(t=t, quat1=quat, gyr1=gyr, mag1=mag, acc1=acc)

    # run webapp
    webapp = qmt.Webapp('/demo/imu-raw-data', data=data)
    webapp.run()

def visulalise_3D_vecs_on_IMU(vecs1, rate):

    imu_letter = 'F'

    # Test visualising vector
    gyr = vecs1*10
    N = len(gyr)
    # acc = [vec2 for _ in range(N)]
    # mag = [vec3 for _ in range(N)]
    quat = [[-0.70738827, 0.70682518, 0, 0] for _ in range(N)]

    config = {'imus': [{'signal': 'quat1', 'letter': imu_letter}]}

    t = qmt.timeVec(N=len(gyr), rate=rate)  # Make a tiem vec based on the length of the quats
    data = qmt.Struct(t=t, quat1=quat, gyr1=gyr)

    # run webapp
    webapp = qmt.Webapp('/demo/imu-raw-data', config=config, data=data)
    webapp.run()

# Function added by MM
def get_ang_vels_from_quats(quats, sample_rate, debug_plot):

    """ Function to calculate 3D angular velocity vectors (in the IMU's global frame (Ei), from IMU orientation quats.
    The following is based on equations A41 and A42 in Appendix C """

    # Calculate quaternion changes from i - 1 to i
    q_change = qmt.qmult(quats[1:], qmt.qinv(quats[:-1]))

    # Use this function to stop the quat suddenly flipping from close to [1, 0, 0, 0] to [-1, 0, 0, 0]
    q_change = qmt.posScalar(q_change)

    # Get ang vel from array of q_change (equivalent to equation A42)
    angle = qmt.quatAngle(q_change)
    axis = qmt.quatAxis(q_change)
    rotvec = angle[..., None] * axis
    ang_vels = rotvec / (1/sample_rate)

    if debug_plot:
        print("Animating input quaternion data...")
        visualise_quat_data(quats, sample_rate)
        print("Plotting output angular velocity vectors...")
        plot_gyr_data(ang_vels, sample_rate)

    return ang_vels


def get_event_dict_from_file(subject_code):

    event_files_folder = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection\SubjectEventFiles'
    event_file_name = subject_code + '_event_dict.txt'
    event_file = join(event_files_folder, event_file_name)

    file_obj = open(event_file, 'r')
    event_dict_str = file_obj.read()
    file_obj.close()
    event_dict = eval(event_dict_str)

    return event_dict

def filter_gyr_data(gyr, cutoff, rate, plot):

    # Check cutoff freq is not near Nyquist freq (= half the sampling rate)
    if 2 * cutoff <= 0.95 * rate:  # do not filter if (almost) at Nyquist frequency

        # Design the butterworth filter parameters
        order = 2
        norm_cut_off = cutoff * 2 / rate
        b, a = signal.butter(order, norm_cut_off)

        """ If there are sections of missing data, filter the segments separately """

        valid_bool_3d = ~np.isnan(gyr)     # Check if there's nan data and create a boolean mask of valid entries
        np.set_printoptions(threshold=np.inf)
        valid_bool = valid_bool_3d.any(axis=1)

        if np.all(valid_bool):

            # If there's no nans, just filter all the data
            gyr_filt = signal.filtfilt(b, a, gyr, axis=0)

        else:

            gyr_filt = np.full_like(gyr, np.nan)   # Create an array the same size as gyr, full of nans
            interval_inds = np.where(np.diff(valid_bool))[0] + 1     # get indices where ~np.isnan() turns from True to False
            segments = np.split(gyr, interval_inds)      # Split the data into valid and non-valid sections
            valid_bool_segments = np.split(valid_bool, interval_inds)      # Split the boolean mask into valid/non-valid segments

            interval_inds = np.concatenate((np.array(([0])), interval_inds))

            # Apply the filter to each valid segment
            for segment, valid_bool_segment, interval_ind in zip(segments, valid_bool_segments, interval_inds):
                if np.all(valid_bool_segment) and len(valid_bool_segment) > 9:       # If the valid_segment is full of 'True' bools (and is longer than the filter pad)
                    filtered_segment = signal.filtfilt(b, a, segment, axis=0)
                    gyr_filt[interval_ind:(interval_ind+len(filtered_segment))] = filtered_segment     # Fill the nan array with the filtered data in the right place

    if plot:
        print('Plotting gyro data before filter:')
        plot_gyr_data(gyr, rate)
        print('Plotting gyro data after filter:')
        plot_gyr_data(gyr_filt, rate)

    return gyr_filt
