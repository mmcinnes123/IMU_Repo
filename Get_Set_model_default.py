# This script is used to update the default coordinates of the model
# Primarily, to change elbow angle quickly, but teh second part can also to set the HT angles to zero.

import opensim as osim
import numpy as np
from scipy.spatial.transform import Rotation as R

""" Set the elbow angle to 0 or 90 degrees """

EL_X_new = 0  # Specify the default elbow flexion angle in degrees

model_file = 'das3.osim'
model = osim.Model(model_file)
model.getCoordinateSet().get('EL_x').setDefaultValue(EL_X_new * np.pi/180 )
model.printToXML(model_file)


""" 
Calculate the GH euler angles required to achieve 0 degree HT angles
Because HT angles aren't coordinates in this model, we can't simply set HT1 = 0 etc
Instead, we can calculate the GH angles which are required to align the humerus frame with the thorax frame
Theory: 
    R_humerus_in_thorax_frame = R_scap_in_thorax_frame * R_humerus_in_scap_frame
    Therefore: R_humerus_in_scap_frame = R_scap_in_thorax_frame.inv() * R_humerus_in_thorax_frame
    If we want the humerus in alignment with the throax frame, then R_humerus_in_thorax_frame = R_identitiy
    And: R_humerus_in_scap_frame = R_scap_in_thorax_frame.inv()
    R_humerus_in_scap can then be decomposed into 'YZY' eulers, which are used to set the GH coordinates
    in the model

Note: The default orientation of the scapula and clavicle should remain the same as original model
"""

# # Get scipy orientation of body 2, expressed in body 1 frame
# def get_scipy_rel_ori(body1, body2):
#     Rot = body2.findTransformBetween(default_state, body1).R()
#     quat = Rot.convertRotationToQuaternion()
#     quat_arr = np.array([quat.get(0), quat.get(1), quat.get(2), quat.get(3)])
#     scipy_R = R.from_quat(quat_arr[[1, 2, 3, 0]])
#     return scipy_R
#
#
# # Get the default state of the model
# default_state = model.initSystem()
#
# # Get each body
# thorax = model.getBodySet().get('thorax')
# humerus = model.getBodySet().get('humerus_r')
# scapula = model.getBodySet().get('scapula_r')
#
# # Get the relative orientation of the scapula in the thorax frame
# R_s_in_t = get_scipy_rel_ori(thorax, scapula)
#
# # Calculate the required rotational offset of the humerus relative to the scapula frame
# R_h_in_s = R_s_in_t.inv()
#
# # Decompose into 'YZY' Eulers, used to update the coordinates of the model.
# GH_Y_new, GH_Z_new, GH_YY_new = R_h_in_s.as_euler('YZY')
# print('\nThe GH coordinates will be set as follows: (GH_y, GH_z, GH_yy)')
# print('(Remember these are subject to gimbal lock, so may look random)')
# print(GH_Y_new*180/np.pi, GH_Z_new*180/np.pi, GH_YY_new*180/np.pi)
#
# # Update the model coordinates
# model.getCoordinateSet().get('GH_y').setDefaultValue(GH_Y_new)
# model.getCoordinateSet().get('GH_z').setDefaultValue(GH_Z_new)
# model.getCoordinateSet().get('GH_yy').setDefaultValue(GH_YY_new)
#
#
# # Check the resultant HT angles
# model.initSystem()      # Reset the state of the model to the default state
# HT_R = get_scipy_rel_ori(thorax, humerus)
# HT_1, HT_2, HT_3 = HT_R.as_euler('YXZ', degrees=True)
# print('\nThe HT angles of the updated model are: (HT_y, HT_x, HT_z)')
# print("%.2f" % HT_1, "%.2f" % HT_2, "%.2f" % HT_3)
#
# model.printToXML(model_file)


