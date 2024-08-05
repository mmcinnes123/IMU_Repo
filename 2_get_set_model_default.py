# This script is used to update the default coordinates of the model
# Primarily, to change elbow angle quickly, but teh second part can also to set the HT angles to zero.

import opensim as osim
import numpy as np
from scipy.spatial.transform import Rotation as R
import numpy as np


""" SET ELBOW COORD """

# EL_X_new = 90  # Specify the default elbow flexion angle in degrees
#
# osim.Model.setDebugLevel(-2)  # Stop warnings about missing geometry vtp files
# model_file = 'das3.osim'
# model = osim.Model(model_file)
# model.getCoordinateSet().get('EL_x').setDefaultValue(EL_X_new * np.pi/180 )
# model.printToXML(model_file)

# print(f'\nIMU das3.osim default elbow angle has been updated to {EL_X_new} degrees.')


""" SET GH MODEL COORDINATES """

def set_GH_coords_for_given_HT_angles(target_abd_angle):

    """
      Calculate the GH euler angles required to achieve 0 degree HT angles
      Because HT angles aren't coordinates in this model, we can't simply set HT1 = 0 etc
      Instead, we can calculate the GH angles which are required to align the humerus frame with the thorax frame
      Theory:
          R_humerus_in_thorax_frame = R_scap_in_thorax_frame * R_humerus_in_scap_frame
          Therefore: R_humerus_in_scap_frame = R_scap_in_thorax_frame.inv() * R_humerus_in_thorax_frame
          And: R_humerus_in_scap_frame = R_scap_in_thorax_frame.inv() * R_humerus_in_thorax_frame
          R_humerus_in_scap can then be decomposed into 'YZX' (or 'YZY') eulers, which are used to set the GH coordinates
          in the model

      Note: The default orientation of the scapula and clavicle should remain the same as original model
      """

    # Set the HT abduction angle
    abd = target_abd_angle

    # Get the default state of the model
    model_file = 'das3.osim'
    model = osim.Model(model_file)
    default_state = model.initSystem()

    # Get each body
    thorax = model.getBodySet().get('thorax')
    humerus = model.getBodySet().get('humerus_r')
    scapula = model.getBodySet().get('scapula_r')

    # Get the relative orientation of the scapula in the thorax frame
    R_s_in_t = get_scipy_rel_ori(thorax, scapula, default_state)

    # Find a rotation which is equivalent to rotating only in abduction plane
    R_h_in_t = R.from_euler('z', abd, degrees=True)

    # Calculate the required rotational offset of the humerus relative to the scapula frame
    R_h_in_s = R_s_in_t.inv() * R_h_in_t

    # Decompose into 'YZY' Eulers, used to update the coordinates of the model.
    GH_Y_new, GH_Z_new, GH_X_new = R_h_in_s.as_euler('YZX')
    print('\nThe GH coordinates will be set as follows: (GH_y, GH_z, GH_x)')
    print('(Remember these are subject to gimbal lock, so may look random)')
    print(GH_Y_new * 180 / np.pi, GH_Z_new * 180 / np.pi, GH_X_new * 180 / np.pi)

    # Update the model coordinates
    model.getCoordinateSet().get('GH_y').setDefaultValue(GH_Y_new)
    model.getCoordinateSet().get('GH_z').setDefaultValue(GH_Z_new)
    model.getCoordinateSet().get('GH_x').setDefaultValue(GH_X_new)

    # Check the resultant HT angles
    state1 = model.initSystem()      # Reset the state of the model to the default state
    HT_R = get_scipy_rel_ori(thorax, humerus, state1)
    HT_1, HT_2, HT_3 = HT_R.as_euler('YZX', degrees=True)
    print('\nThe HT angles of the updated model are: (HT_y, HT_x, HT_z)')
    print("%.2f" % HT_1, "%.2f" % HT_2, "%.2f" % HT_3)

    model.printToXML(model_file)
    print('Saved to model file: ', model_file)


# Function to set the clamp range of a group of coords
def set_coord_clamps():

    model_file = 'das3.osim'
    model = osim.Model(model_file)

    range_max = np.deg2rad(200)
    range_min = np.deg2rad(-200)

    coord_set = ['GH_y', 'GH_z', 'GH_x', 'EL_x', 'PS_y']

    for coord in coord_set:
        # Update the model coordinates
        model.getCoordinateSet().get(coord).setRangeMax(range_max)
        model.getCoordinateSet().get(coord).setRangeMin(range_min)

    model.printToXML(model_file)


# Get scipy orientation of body 2, expressed in body 1 frame
def get_scipy_rel_ori(body1, body2, state):
    Rot = body2.findTransformBetween(state, body1).R()
    quat = Rot.convertRotationToQuaternion()
    quat_arr = np.array([quat.get(0), quat.get(1), quat.get(2), quat.get(3)])
    scipy_R = R.from_quat(quat_arr[[1, 2, 3, 0]])
    return scipy_R


set_GH_coords_for_given_HT_angles(target_abd_angle=0)