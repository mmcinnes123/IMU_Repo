
import opensim as osim
import qmt



def get_IMU_in_body_frame(model_file):
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
    hum_clus_y_axis = marker_1_in_hum - marker_4_in_hum  # y_axis is marker 4 to marker 1 (pointing down)
    hum_clus_x_axis = marker_3_in_hum - marker_4_in_hum  # x_axis is marker 4 to marker 3 (pointing backwards)
    hum_clus_in_hum = qmt.quatFrom2Axes(hum_clus_x_axis, hum_clus_y_axis, None, plot=False)

    # Forearm cluster CF expressed in the radius CF, using the marker positions
    rad_clus_y_axis = marker_2_in_rad - marker_1_in_rad  # y_axis is marker 1 to marker 2 (pointing down)
    rad_clus_x_axis = marker_1_in_rad - marker_4_in_rad  # x_axis is marker 4 to marker 1 (pointing backwards)
    rad_clus_in_rad = qmt.quatFrom2Axes(rad_clus_x_axis, rad_clus_y_axis, None, plot=False)

    return hum_clus_in_hum, rad_clus_in_rad