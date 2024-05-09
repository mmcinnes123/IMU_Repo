
import opensim as osim
import os
from scipy.spatial.transform import Rotation as R
import numpy as np
import pandas as pd

IMU_IK_settings_file = 'IMU_IK_Settings.xml'
OMC_IK_settings_file = 'OMC_IK_Settings.xml'

# Instantiate an InverseKinematicsTool
imuIK = osim.IMUInverseKinematicsTool(IMU_IK_settings_file)

imuIK.set_time_range(0, 1)
imuIK.set_time_range(1, 2)

thorax_imu_weight = osim.OrientationWeight('thorax_imu', 1.0)
humerus_imu_weight = osim.OrientationWeight('humerus_imu', 1.0)
radius_imu_weight = osim.OrientationWeight('radius_imu', 10.0)

imuIK.upd_orientation_weights().cloneAndAppend(thorax_imu_weight)
imuIK.upd_orientation_weights().cloneAndAppend(humerus_imu_weight)
imuIK.upd_orientation_weights().cloneAndAppend(radius_imu_weight)

imuIK.printToXML('New_IMU_IK_Settings.xml')


