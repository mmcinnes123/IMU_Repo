

from helpers_2DoF import get_np_quats_from_txt_file
from helpers_2DoF import get_joint_axis_directly_from_ang_vels
from helpers_2DoF import get_local_ang_vels_from_quats
from helpers_2DoF import plot_gyr_data
from joint_axis_est_2d import jointAxisEst2D

import qmt
import opensim as osim
import itertools
from scipy.spatial.transform import Rotation as R
from os.path import join

import numpy as np
from tkinter.filedialog import askopenfilename, askdirectory
np.set_printoptions(suppress=True)

osim.Logger.setLevelString("Off")



def get_J1_J2_from_calibrated_OMC_model(model_file, debug):

    """Get the FE axis expressed in the model's humerus body frame, based on default model"""

    # Based on how the hu joint is defined in the model, the XYZ euler ori offset of the parent frame,
    # relative to humerus frame is:
    hu_parent_rel2_hum_R = R.from_euler('XYZ', [0, 0, 0.32318], degrees=False)

    # Based on how the hu joint is defined in the model, relative to the hu joint parent frame,
    # the vector of hu rotation axis (EL_x) is:
    EL_axis_rel2_hu_parent = [0.969, -0.247, 0]

    # Get the vector of hu rotation axis, relative to the humerus frame
    EL_axis_rel2_humerus = hu_parent_rel2_hum_R.apply(EL_axis_rel2_hu_parent)

    """Get the cluster frame, expressed in the humerus frame, specific to the subject's model"""

    # Read in calibrated model file to get position of humerus markers in humerus frame
    my_model = osim.Model(model_file)
    marker_1_in_hum = my_model.getMarkerSet().get('Hum_Clus_1').get_location().to_numpy()
    marker_3_in_hum = my_model.getMarkerSet().get('Hum_Clus_3').get_location().to_numpy()
    marker_4_in_hum = my_model.getMarkerSet().get('Hum_Clus_4').get_location().to_numpy()

    # y_axis is marker 4 to marker 1 (pointing down)
    y_axis = marker_1_in_hum - marker_4_in_hum

    # x_axis is marker 4 to marker 3 (pointing backwards)
    x_axis = marker_3_in_hum - marker_4_in_hum

    # Get the cluster CF expressed in the humerus CF, using the marker positions
    cluster_in_hum = qmt.quatFrom2Axes(x_axis, y_axis, None, plot=False)

    # Now express the FE axis in the cluster frame
    FE_in_clus = qmt.rotate(cluster_in_hum, EL_axis_rel2_humerus, plot=False)

    if debug:
        print('get_J1_J2_from_calibrated_OMC_model() DEBUG:')
        print('\tFE axis in humerus body frame: ', EL_axis_rel2_humerus)
        print('\tModel file used: ', model_file)
        print('\tMarker 1 position: ', np.array_str(marker_1_in_hum))

    # TODO: Write same code to find PS in radius frame

    return FE_in_clus



def get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt, opt_method, subject_time_dict, sample_rate, debug):

    assert IMU_type_for_opt in ['Real', 'Perfect'], 'IMU type not Real or Perfect'
    assert opt_method in ['rot', 'ori'], 'Opt method not rot or ori'

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

    # Trim the IMU data based on the period of interest
    start_ind = FE_start_time * sample_rate
    end_ind = PS_end_time * sample_rate
    IMU2_trimmed = IMU2_np[start_ind:end_ind]
    IMU3_trimmed = IMU3_np[start_ind:end_ind]

    # Run the rot method
    params = dict(method='rot')
    opt_results = jointAxisEst2D(IMU2_trimmed, IMU3_trimmed, None, None, sample_rate, params=params, debug=True, plot=False)
    FE = opt_results['j1']
    PS = opt_results['j2']

    if debug:
        print('get_J1_J2_from_opt() DEBUG:')
        print('\tTMM file used: ', tmm_txt_file)
        print('\tBetween times: ', start_ind/sample_rate, end_ind/sample_rate)
        print('\tUsing optimistaion method: ', opt_method)
        # print('\tComplete optimisation results: ')
        # print(opt_results)

        # Get variation in movement of each IMU
        def get_ori_variation(IMU_quats):
            quat_changes = np.zeros((len(IMU_quats), 4))
            for row in range(len(IMU_quats)):
                quat_changes[row] = qmt.qmult(qmt.qinv(IMU_quats[row]), IMU_quats[0])  # Get orientation change from q0, for all t
            angles = qmt.quatAngle(quat_changes, plot=True)  # Get the magnitude of the change from q0
            mean_diff = np.mean(np.array(abs(angles)))  # Get the average change in ori from q0
            return mean_diff

        IMU2_mean_diff = get_ori_variation(IMU2_trimmed)
        IMU3_mean_diff = get_ori_variation(IMU3_trimmed)
        print('\tMean variation in orientation Humerus IMU: ', IMU2_mean_diff * 180 / np.pi)
        print('\tMean variation in orientation Radius IMU: ', IMU3_mean_diff * 180 / np.pi)

    return FE, PS, opt_results



def visulalise_3D_vec_on_IMU(vec1, vec2):

    # scale vec2 so it displays better
    vec2 = 5*vec2

    # Test visualising vector
    N = 100
    gyr = [vec1 for _ in range(N)]
    acc = [vec2 for _ in range(N)]
    mag = [[0, 0, 0] for _ in range(N)]
    quat = [[-0.70738827, 0.70682518, 0, 0] for _ in range(N)]

    t = qmt.timeVec(N=len(quat), rate=100)  # Make a tiem vec based on the length of the quats
    data = qmt.Struct(t=t, quat1=quat, gyr1=gyr, mag1=mag, acc1=acc)

    # run webapp
    webapp = qmt.Webapp('/demo/imu-raw-data', data=data)
    webapp.run()

"""" Setting specific to subject """


# Data to use for the optimisation
trial_for_opt = 'JA_Slow'
IMU_type_for_opt = 'Perfect'
opt_method = 'rot'
time_dict = {
    'P1': {'JA_Slow': {'FE_start': 16, 'FE_end': 29, 'PS_start': 31, 'PS_end': 47}},
    'P2': {'JA_Slow': {'FE_start': 15, 'FE_end': 28, 'PS_start': 30, 'PS_end': 46}},
    'P3': {'JA_Slow': {'FE_start': 18, 'FE_end': 34, 'PS_start': 35, 'PS_end': 49}},
    'P4': {'JA_Slow': {'FE_start': 23, 'FE_end': 39, 'PS_start': 42, 'PS_end': 60}},
    'P5': {'JA_Slow': {'FE_start': 30, 'FE_end': 47, 'PS_start': 48, 'PS_end': 65}},
    'P6': {'JA_Slow': {'FE_start': 26, 'FE_end': 44, 'PS_start': 47, 'PS_end': 65}}
             }
sample_rate = 100          # This is the sample rate of the data going into the function



for subject_code in time_dict.keys():

    print('Results for Subject: ', subject_code)

    # Define some files
    parent_dir = r'C:\Users\r03mm22\Documents\Protocol_Testing\2024 Data Collection' + '\\' + subject_code
    OMC_dir = join(parent_dir, 'OMC')
    model_file = join(OMC_dir, 'das3_scaled_and_placed.osim')
    subject_time_dict = time_dict[subject_code]

    """ FINDING REFERENCE J1 AXIS IN HUMERUS CLUSTER FRAME """
    FE_in_clus = get_J1_J2_from_calibrated_OMC_model(model_file, debug=False)
    # print('OMC reference FE axis in humerus cluster frame:', FE_in_clus)


    """ FINDING FE AND PS FROM OPTIMISATION RESULT """
    opt_FE, opt_PS, opt_results = get_J1_J2_from_opt(subject_code, IMU_type_for_opt, trial_for_opt,
                                                     opt_method, subject_time_dict, sample_rate, debug=False)
    # print('FE axis estimation with rot method: ', opt_FE)
    # print('PS axis estimation with rot method: ', opt_PS)
    # print('and heading offset: ', opt_results['delta']*180/np.pi)
    # print('Cost: ', opt_results['debug']['cost'])
    # print('x: ', opt_results['debug']['x'])

    """ COMPARE WITH OPTIMISATION """

    # Note, optimisation based J1, J2 can point in either direction
    if np.sign(opt_FE[2]) == np.sign(-FE_in_clus[2]):
        opt_FE = -opt_FE

    error = qmt.angleBetween2Vecs(FE_in_clus, opt_FE)
    print('Error: ', error*180/np.pi)

#     # visulalise_3D_vec_on_IMU(opt_FE, FE_in_clus)



