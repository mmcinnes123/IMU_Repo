import itertools
from abc import ABC

import numpy as np
from scipy import signal
# from numpy.core.umath_tests import inner1d

import qmt

from optimize import AbstractObjectiveFunction, GNSolver
from helpers_2DoF import plot_gyr_data
from helpers_2DoF import visualise_quat_data
from helpers_2DoF import get_ang_vels_from_quats
from helpers_2DoF import visulalise_3D_vecs_on_IMU


def inner1d(a, b):  # avoid deprecation, cf. https://stackoverflow.com/a/15622926
    return np.einsum('ij,ij->i', np.atleast_2d(a), np.atleast_2d(b))


def jointAxisEst2D(quat1, quat2, gyr1, gyr2, rate, params=None, debug=False, plot=False):
    # TODO: docs, debug plot

    # Update the parameters (settings) with default values if they haven't been specified in the input
    defaults = dict(method='rot', gyrCutoff=5, downsampleRate=20)
    params = qmt.setDefaults(params, defaults)

    # Check the quaternion and gyro arrays are the correct shape
    N = quat1.shape[0]
    assert quat1.shape == (N, 4)
    assert quat2.shape == (N, 4)

    # Define each setting from the params dict
    method = params['method']
    gyrCutoff = params['gyrCutoff']
    downsampleRate = params['downsampleRate']
    assert method in ('rot', 'ori', 'rot_noDelta')

    # Downsample the orientation data
    if rate == downsampleRate or downsampleRate is None:
        ind = slice(None)
    else:
        assert downsampleRate < rate
        M = int(round(N*downsampleRate/rate))
        ind = np.linspace(0, N-1, M, dtype=int)
    q1 = quat1[ind].copy()
    q2 = quat2[ind].copy()

    # If no gyroscope info is provided, create synthesised gyro data from quaternion data
    if gyr1 is None or gyr2 is None:
        # Use the down-sampled orientation data to calculate angular velocities
        # Note: these are already in the IMUs reference frame, not in the local frame as real gyro data would be
        gyr1_E1 = get_ang_vels_from_quats(q1, downsampleRate, debug_plot=False)
        gyr2_E2 = get_ang_vels_from_quats(q2, downsampleRate, debug_plot=False)

        # And remove the last row from the ori data to match the size of the gyro data
        q1 = q1[:-1]
        q2 = q2[:-1]

    # If gyro data is provided, it must be converted from the local frame to the reference frame
    else:
        assert gyr1.shape == (N, 3)
        assert gyr2.shape == (N, 3)
        gyr1_E1 = qmt.rotate(q1, gyr1[ind])
        gyr2_E2 = qmt.rotate(q2, gyr2[ind])

    # Apply a butterworth low pass filter to the angular velocity data
    # (The gyrCutoff is the cut-off frequency used to filter the angular rates (used in the rotation constraint))
    if gyrCutoff is not None:  # apply Butterworth low pass filter
        if 2 * gyrCutoff <= 0.95 * downsampleRate:  # do not filter if (almost) at Nyquist frequency
            b, a = signal.butter(2, gyrCutoff * 2 / downsampleRate)
            gyr1_E1 = signal.filtfilt(b, a, gyr1_E1, axis=0)
            gyr2_E2 = signal.filtfilt(b, a, gyr2_E2, axis=0)

    # Define the dict of data to be used in the optimisation function
    d = dict(quat1=q1, quat2=q2, gyr1_E1=gyr1_E1, gyr2_E2=gyr2_E2)

    # Specify which constraint (Class) to use based on the method specified in params
    objFnCls = dict(rot=AxisEst2DRotConstraint, rot_noDelta=AxisEst2DRotConstraint_noDelta, ori=AxisEst2DOriConstraint)[method]
    if method == 'rot_noDelta':
        initVals_variant = 'rot_noDelta'
    else:
        initVals_variant = 'default'
    initVals = objFnCls.getInitVals(variant=initVals_variant)

    # Run the solver
    x = None
    parameters = None
    cost = None
    for initVal in initVals:
        objFn = objFnCls(initVal)
        objFn.setData(d)
        solver = GNSolver(objFn)
        for i in range(200):
            deltaX = solver.step()
            if i >= 10 and np.linalg.norm(deltaX) < 1e-10:
                break
        if cost is None or solver.objFn.cost() < cost:  # Update the cost, x, and parameters if cost is less than for previous initVal
            cost = solver.objFn.cost()
            x = solver.objFn.getX()
            parameters = solver.objFn.unpackX()     # The outputs j1, j2 and delta are stored in here

    # Extract the outputs of the optimisation: j1, j2, delta (heading offset), and for 'ori' method, beta (carry angle)
    out = dict(
        j1=parameters['j1'],
        j2=parameters['j2'],
    )

    if 'delta' in parameters:
        out['delta'] = qmt.wrapToPi(parameters['delta'])

    if 'beta' in parameters:
        out['beta'] = qmt.wrapToPi(parameters['beta'])

    if debug:
        # Calculate the variation in the 3rd degree of freedom over the sample period, as a measure of error

        z_ax = np.array([0, 0, 1])
        b1_s1_ang = np.arccos(np.dot(z_ax, parameters['j1']))
        b1_s1_ax = _cross1d(z_ax, parameters['j1'])
        q_b1_s1 = qmt.quatFromAngleAxis(b1_s1_ang, b1_s1_ax)    # Get ori of a body which has a z axis aligned with J1

        y_ax = np.array([0, 1, 0])
        b2_s2_ang = np.arccos(np.dot(y_ax, parameters['j2']))
        b2_s2_ax = _cross1d(y_ax, parameters['j2'])
        q_b2_s2 = qmt.quatFromAngleAxis(b2_s2_ang, b2_s2_ax)    # Get ori of a body which has a 2 axis aligned with J2

        q2_1 = _qmult(_qinv(q1), q2)     # Relative sensor ori
        q_joint = _qmult(_qmult(_qinv(q_b1_s1), q2_1), q_b2_s2)    # Relative body ori

        third_DoF_angle_eul = qmt.eulerAngles(q_joint, 'zxy', intrinsic=True, plot=False)
        SD_third_DoF = np.nanstd(np.rad2deg(third_DoF_angle_eul[:,1]))

        out['debug'] = dict(cost=cost, x=x, SD_third_DoF=SD_third_DoF)

    if plot:

        if gyr1 is None or gyr2 is None:
            # Express input ang vels in local frames
            gyr2_2 = qmt.rotate(qmt.qinv(q2), gyr2_E2)
            gyr2_1 = qmt.rotate(qmt.qinv(q1), gyr2_E2)

        else:
            # If gyro data is provided, gyr1, gyr2 are in local frames
            gyr2_2 = gyr2
            gyr2_1 = qmt.rotate(qmt.qmult(qmt.qinv(q2), q1), gyr2)

        print('Visualising ang vel of IMU2 in IMU2 frame')
        visulalise_3D_vecs_on_IMU(gyr2_2, downsampleRate)
        print('Visualising ang vel of IMU2 in IMU1 frame')
        visulalise_3D_vecs_on_IMU(gyr2_1, downsampleRate)



    return out


def _cross(a, b):
    """Row-wise _cross product, faster than np._cross"""
    c = np.empty((max(a.shape[0], b.shape[0]), 3))
    c[:, 0] = a[:, 1]*b[:, 2] - a[:, 2]*b[:, 1]
    c[:, 1] = a[:, 2]*b[:, 0] - a[:, 0]*b[:, 2]
    c[:, 2] = a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0]
    return c


def _cross1d(a, b):
    return np.array([
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]], dtype=float)


def _qmult(q1, q2):  # quaternion multiplication without much type checking
    q3 = np.zeros(q1.shape if q1.ndim == 2 else q2.shape, float)
    q3[..., 0] = q1[..., 0] * q2[..., 0] - q1[..., 1] * q2[..., 1] - q1[..., 2] * q2[..., 2] - q1[..., 3] * q2[..., 3]
    q3[..., 1] = q1[..., 0] * q2[..., 1] + q1[..., 1] * q2[..., 0] + q1[..., 2] * q2[..., 3] - q1[..., 3] * q2[..., 2]
    q3[..., 2] = q1[..., 0] * q2[..., 2] - q1[..., 1] * q2[..., 3] + q1[..., 2] * q2[..., 0] + q1[..., 3] * q2[..., 1]
    q3[..., 3] = q1[..., 0] * q2[..., 3] + q1[..., 1] * q2[..., 2] - q1[..., 2] * q2[..., 1] + q1[..., 3] * q2[..., 0]
    return q3


def _qinv(q):  # invert quaternion without much type checking
    out = q.copy()
    out[..., 1:] *= -1
    return out


def _rotate(q, v):  # quaternion rotation without much type checking
    out = np.zeros((q.shape[0], 3) if q.ndim == 2 else v.shape, float)
    out[..., 0] = (1 - 2 * q[..., 2] ** 2 - 2 * q[..., 3] ** 2) * v[..., 0] \
        + 2 * v[..., 1] * (q[..., 2] * q[..., 1] - q[..., 0] * q[..., 3]) \
        + 2 * v[..., 2] * (q[..., 0] * q[..., 2] + q[..., 3] * q[..., 1])

    out[..., 1] = 2 * v[..., 0] * (q[..., 0] * q[..., 3] + q[..., 2] * q[..., 1]) \
        + v[..., 1] * (1 - 2 * q[..., 1] ** 2 - 2 * q[..., 3] ** 2) \
        + 2 * v[..., 2] * (q[..., 2] * q[..., 3] - q[..., 1] * q[..., 0])

    out[..., 2] = 2 * v[..., 0] * (q[..., 3] * q[..., 1] - q[..., 0] * q[..., 2]) \
        + 2 * v[..., 1] * (q[..., 0] * q[..., 1] + q[..., 3] * q[..., 2]) \
        + v[..., 2] * (1 - 2 * q[..., 1] ** 2 - 2 * q[..., 2] ** 2)
    return out


def axisFromThetaPhi(theta, phi, var):
    if var == 1:
        j = np.array([np.sin(theta)*np.cos(phi), np.sin(theta)*np.sin(phi), np.cos(theta)], float)
    elif var == 2:
        j = np.array([np.cos(theta), np.sin(theta)*np.sin(phi), np.sin(theta)*np.cos(phi)], float)
    else:
        raise ValueError('invalid axis var')
    return j


def axisToThetaPhi(j, var):
    if var == 1:
        theta = np.arccos(j[2])
        phi = np.arctan2(j[1], j[0])
    elif var == 2:
        theta = np.arccos(j[0])
        phi = np.arctan2(j[1], j[2])
    else:
        raise ValueError('invalid axis var')
    return theta, phi


def checkAndSwitchAxisVar(x, axisCount, switchThreshold=0.5):
    """
    Switches between two spherical axis represenations if neccessary, as the derivative becomes zero when the
    "inclination" of the axis becomes close to vertical. The parameter x is modified in-place.
    :param x: state vector, the axes must be first and the axis variation identifiers (1 or 2) must be last,
     i.e. x = [theta, phi]*N + other_states + [axisVar]*N
    :param axisCount: Number of axes (N).
    :param switchThreshold: If sin(theta) is below the threshold, the parametrization will be switched. 0.5 corresponds
    to 30° from vertical.
    """

    changed = False
    for i in range(axisCount):
        theta = x[2*i]
        if np.abs(np.sin(theta)) < switchThreshold:
            phi = x[2*i+1]
            var = x[-(axisCount-i)]
            j = axisFromThetaPhi(theta, phi, var)
            newVar = 1 if var == 2 else 2
            newTheta, newPhi = axisToThetaPhi(j, newVar)
            x[2*i] = newTheta
            x[2*i+1] = newPhi
            x[-(axisCount-i)] = newVar
            changed = True
    return changed


def axisGradient(theta, phi, var):
    assert var in (1, 2)
    ct = np.cos(theta)
    st = np.sin(theta)
    cp = np.cos(phi)
    sp = np.sin(phi)
    if var == 1:
        dj_theta = np.array([ct*cp, ct*sp, -st], float)
        dj_phi = np.array([-st*sp, st*cp, 0], float)
    else:
        dj_theta = np.array([-st, ct*sp, ct*cp], float)
        dj_phi = np.array([0, st*cp, -st*sp], float)
    return dj_theta, dj_phi


class AbstractAxisEst2DObjectiveFunction(AbstractObjectiveFunction, ABC):
    def __init__(self, init):
        super().__init__(init)

    def err(self):
        return self.errAndJac()[0]

    def checkAndSwitchParametrization(self):
        changed = checkAndSwitchAxisVar(self._x, 2)
        if changed:
            self.clearCache()

    def calculateParameterDistance(self, other, strict=False):
        param1 = self.unpackX()
        param2 = other.unpackX()
        if strict:
            axisDiff1 = np.arccos(np.clip(np.dot(param1['j1'], param2['j1']), -1, 1))
            axisDiff2 = np.arccos(np.clip(np.dot(param1['j2'], param2['j2']), -1, 1))
        else:
            # abs of dot product since j and -j are considered to be the same axis!
            axisDiff1 = np.arccos(min(1, np.abs(np.dot(param1['j1'], param2['j1']))))
            axisDiff2 = np.arccos(min(1, np.abs(np.dot(param1['j2'], param2['j2']))))
        return np.rad2deg(axisDiff1 + axisDiff2 + abs(qmt.wrapToPi(param1['delta']-param2['delta']))
                          + abs(qmt.wrapToPi(param1.get('beta', 0)-param2.get('beta', 0))))

    @staticmethod
    def getInitVals(variant='default', seed=None):
        assert variant in ['default', 'rot_noDelta'] or variant.startswith('rand')
        if variant == 'default':
            e_x = np.array([1, 0.01, 0.01], float)
            e_y = np.array([0.01, 1, 0.01], float)
            e_z = np.array([0.01, 0.01, 1], float)
            e_x /= np.linalg.norm(e_x)
            e_y /= np.linalg.norm(e_y)
            e_z /= np.linalg.norm(e_z)
            init = []
            # Build up, row by row, every combination of options for j1, j2 and delta, from the options of ex, ey, ez
            # (above) for j1 and j2, and the options of -90, 0, 90, 180 for delta
            for j1, j2, delta in itertools.product([e_x, e_y, e_z], [e_x, e_y, e_z], np.deg2rad([-90, 0, 90, 180])):
                # Build the row by concatenating along the row (np.r_), J1, J2 (2 params each), delta, and the ints)
                init.append(np.r_[axisToThetaPhi(j1, 1), axisToThetaPhi(j2, 1), delta, 1, 1])
            return np.array(init, float)
        if variant == 'rot_noDelta':
            e_x = np.array([1, 0.01, 0.01], float)
            e_y = np.array([0.01, 1, 0.01], float)
            e_z = np.array([0.01, 0.01, 1], float)
            e_x /= np.linalg.norm(e_x)
            e_y /= np.linalg.norm(e_y)
            e_z /= np.linalg.norm(e_z)
            init = []
            # Build up, row by row, every combination of options for j1, j2, from the options of ex, ey, ez
            for j1, j2 in itertools.product([e_x, e_y, e_z], [e_x, e_y, e_z]):
                # Build the row by concatenating along the row (np.r_), J1, J2 (2 params each), delta, and the ints)
                init.append(np.r_[axisToThetaPhi(j1, 1), axisToThetaPhi(j2, 1), 1, 1])
            return np.array(init, float)
        # An alternative set of init values using random numbers, or setting delta to 0
        elif variant.startswith('rand'):  # e.g. 'rand100_delta0', 'rand100')
            if seed is None:
                r = np.random
            else:
                r = np.random.RandomState(seed)
            # Get the number after 'rand', e.g. 100
            N = variant[len('rand'):-len('_delta0')] if variant.endswith('_delta0') else variant[len('rand'):]
            assert N.isdigit()
            # Create the init array using random values between pi and -pi for the first 6 params, then rand integers for the last two
            init = np.c_[r.uniform(-np.pi, np.pi, (int(N), 6)), r.randint(1, 3, (int(N), 2))]
            # Set delta to 0
            if variant.endswith('_delta0'):
                init[:, 4] = 0
            return init


class AxisEst2DRotConstraint(AbstractAxisEst2DObjectiveFunction):
    def __init__(self, init):
        super().__init__(init)
        self.updateIndices = slice(0, 5)

    def errAndJac(self):
        if 'err' in self._cache and 'jac' in self._cache:
            return self._cache['err'], self._cache['jac']

        d = self._data
        theta1, phi1, theta2, phi2, delta, var1, var2 = self._x

        q1 = d['quat1']
        q2 = d['quat2']
        w1_e1 = d['gyr1_E1']  # gyr1_E1 = qmt.rotate(quat1, gyr1)
        w2_e2 = d['gyr2_E2']  # gyr2_E2 = qmt.rotate(quat2, gyr2)
        N = q1.shape[0]
        assert q1.shape == q2.shape == (N, 4)
        assert w1_e1.shape == w2_e2.shape == (N, 3)

        j1_est = axisFromThetaPhi(theta1, phi1, var1)
        j2_est = axisFromThetaPhi(theta2, phi2, var2)

        q_E2_E1 = np.array([np.cos(delta/2), 0, 0, np.sin(delta/2)], float)

        q2_e1_est = _qmult(q_E2_E1, q2)
        j1_e1 = _rotate(q1, j1_est)
        j2_e2 = _rotate(q2, j2_est)
        j2_e1 = _rotate(q2_e1_est, j2_est)
        w2_e1 = _rotate(q_E2_E1, w2_e2)

        ax_orig = _cross(j1_e1, j2_e1)
        ax_norm = np.linalg.norm(ax_orig, axis=1)[:, None]
        ax = ax_orig / ax_norm
        w_d = w1_e1 - w2_e1
        err = inner1d(w_d, ax)

        dj1_theta, dj1_phi = axisGradient(theta1, phi1, var1)
        dj2_theta, dj2_phi = axisGradient(theta2, phi2, var2)

        e_z = np.array([0, 0, 1], float)
        dj2_delta = (-j2_e2 * np.sin(delta)
                     + _cross(e_z[None, :], j2_e2) * np.cos(delta)
                     + e_z[None, :] * (inner1d(e_z, j2_e2)*np.sin(delta))[:, None])
        dwd_delta = -(-w2_e2 * np.sin(delta)
                      + _cross(e_z[None, :], w2_e2) * np.cos(delta)
                      + e_z[None, :] * (inner1d(e_z, w2_e2) * np.sin(delta))[:, None])
        d_ax_orig_delta = _cross(j1_e1, dj2_delta)
        d_ax_delta = d_ax_orig_delta / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_delta)[:, None] / ax_norm ** 3
        d_delta = inner1d(dwd_delta, ax) + inner1d(w_d, d_ax_delta)

        d_ax_orig_theta1 = _cross(_rotate(q1, dj1_theta), j2_e1)
        d_ax_theta1 = d_ax_orig_theta1/ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_theta1)[:, None] / ax_norm**3
        d_theta1 = inner1d(w_d, d_ax_theta1)
        d_ax_orig_phi1 = _cross(_rotate(q1, dj1_phi), j2_e1)
        d_ax_phi1 = d_ax_orig_phi1 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_phi1)[:, None] / ax_norm ** 3
        d_phi1 = inner1d(w_d, d_ax_phi1)
        d_ax_orig_theta2 = _cross(j1_e1, _rotate(q2_e1_est, dj2_theta))
        d_ax_theta2 = d_ax_orig_theta2 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_theta2)[:, None] / ax_norm ** 3
        d_theta2 = inner1d(w_d, d_ax_theta2)
        d_ax_orig_phi2 = _cross(j1_e1, _rotate(q2_e1_est, dj2_phi))
        d_ax_phi2 = d_ax_orig_phi2 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_phi2)[:, None] / ax_norm ** 3
        d_phi2 = inner1d(w_d, d_ax_phi2)

        jac = np.column_stack([d_theta1, d_phi1, d_theta2, d_phi2, d_delta])

        self._cache['err'] = err
        self._cache['jac'] = jac
        return err, jac

    def unpackX(self):
        assert self._x.shape == (7,)
        return {
            'j1': axisFromThetaPhi(self._x[0], self._x[1], self._x[5]),
            'j2': axisFromThetaPhi(self._x[2], self._x[3], self._x[6]),
            'delta': self._x[4],
        }


class AxisEst2DRotConstraint_noDelta(AbstractAxisEst2DObjectiveFunction):
    def __init__(self, init):
        super().__init__(init)
        self.updateIndices = slice(0, 4)

    def errAndJac(self):
        if 'err' in self._cache and 'jac' in self._cache:
            return self._cache['err'], self._cache['jac']

        d = self._data
        theta1, phi1, theta2, phi2, var1, var2 = self._x

        q1 = d['quat1']
        q2 = d['quat2']
        w1_e = d['gyr1_E1']  # gyr1_E1 = qmt.rotate(quat1, gyr1)
        w2_e = d['gyr2_E2']  # gyr2_E2 = qmt.rotate(quat2, gyr2)
        N = q1.shape[0]
        assert q1.shape == q2.shape == (N, 4)
        assert w1_e.shape == w2_e.shape == (N, 3)

        # Get j1 and j2 in axis form, from our latest estimates of theta1, phi1, etc
        j1_est = axisFromThetaPhi(theta1, phi1, var1)
        j2_est = axisFromThetaPhi(theta2, phi2, var2)

        # Get j1 and j2 in the (shared) inertial reference frame
        j1_e = _rotate(q1, j1_est)
        j2_e = _rotate(q2, j2_est)

        # Calculate the elements for our cost function
        ax_orig = _cross(j1_e, j2_e)
        ax_norm = np.linalg.norm(ax_orig, axis=1)[:, None]
        ax = ax_orig / ax_norm
        w_d = w1_e - w2_e   # The relative angular velocity term

        # Calculate the current error from our cost function
        err = inner1d(w_d, ax)

        ### Calculate the jacobian by finding every partial derivative

        # Find the partial derivatives of j1 and j2, wrt theta1, phi1, theta2, phi2, from the basic definition of
        # spherical coords (theta, phi) from a 3D vector
        dj1_theta, dj1_phi = axisGradient(theta1, phi1, var1)
        dj2_theta, dj2_phi = axisGradient(theta2, phi2, var2)

        # Find partial derivatives of the error function wrt the joint axes (theta1, phi1, theta2, phi2)
        d_ax_orig_theta1 = _cross(_rotate(q1, dj1_theta), j2_e)
        d_ax_theta1 = d_ax_orig_theta1/ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_theta1)[:, None] / ax_norm**3
        d_theta1 = inner1d(w_d, d_ax_theta1)
        d_ax_orig_phi1 = _cross(_rotate(q1, dj1_phi), j2_e)
        d_ax_phi1 = d_ax_orig_phi1 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_phi1)[:, None] / ax_norm ** 3
        d_phi1 = inner1d(w_d, d_ax_phi1)
        d_ax_orig_theta2 = _cross(j1_e, _rotate(q2, dj2_theta))
        d_ax_theta2 = d_ax_orig_theta2 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_theta2)[:, None] / ax_norm ** 3
        d_theta2 = inner1d(w_d, d_ax_theta2)
        d_ax_orig_phi2 = _cross(j1_e, _rotate(q2, dj2_phi))
        d_ax_phi2 = d_ax_orig_phi2 / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_phi2)[:, None] / ax_norm ** 3
        d_phi2 = inner1d(w_d, d_ax_phi2)

        jac = np.column_stack([d_theta1, d_phi1, d_theta2, d_phi2])

        self._cache['err'] = err
        self._cache['jac'] = jac

        return err, jac

    def unpackX(self):
        assert self._x.shape == (6,)
        return {
            'j1': axisFromThetaPhi(self._x[0], self._x[1], self._x[4]),
            'j2': axisFromThetaPhi(self._x[2], self._x[3], self._x[5]),
        }


class AxisEst2DOriConstraint(AbstractAxisEst2DObjectiveFunction):
    def __init__(self, init):
        super().__init__(init)
        self.updateIndices = slice(0, 6)

    def errAndJac(self):
        if 'err' in self._cache and 'jac' in self._cache:
            return self._cache['err'], self._cache['jac']

        theta1, phi1, theta2, phi2, delta, beta, var1, var2 = self._x
        q1 = self._data['quat1']
        q2 = self._data['quat2']
        N = len(q1)
        assert q1.shape == q2.shape == (N, 4)

        j1_est = axisFromThetaPhi(theta1, phi1, var1)
        j2_est = axisFromThetaPhi(theta2, phi2, var2)

        q_E2_E1 = np.array([np.cos(delta/2), 0, 0, np.sin(delta/2)], float)

        q1_angle = np.arccos(np.dot([0, 0, 1], j1_est))
        q1_axis = _cross1d([0, 0, 1], j1_est)
        q1_axis_norm = np.linalg.norm(q1_axis)
        q1_axis_n = q1_axis / q1_axis_norm
        q1_sin = np.sin(q1_angle/2)
        q1_cos = np.cos(q1_angle/2)
        q_B1_S1 = np.array([q1_cos, q1_sin*q1_axis_n[0], q1_sin*q1_axis_n[1], q1_sin*q1_axis_n[2]], float)

        q2_angle = np.arccos(np.dot([0, 1, 0], j2_est))
        q2_axis = _cross1d([0, 1, 0], j2_est)
        q2_axis_norm = np.linalg.norm(q2_axis)
        q2_axis_n = q2_axis / q2_axis_norm
        q2_sin = np.sin(q2_angle/2)
        q2_cos = np.cos(q2_angle/2)
        q_B2_S2 = np.array([q2_cos, q2_sin*q2_axis_n[0], q2_sin*q2_axis_n[1], q2_sin*q2_axis_n[2]], float)

        q_E1_B1 = _qmult(_qinv(q_B1_S1), _qinv(q1))  # q12
        q_E2_B1 = _qmult(q_E1_B1, q_E2_E1)  # q123
        q_S2_B1 = _qmult(q_E2_B1, q2)  # q1234
        q_B2_B1 = _qmult(q_S2_B1, q_B2_S2)  # q12345

        arcsin_arg = 2*(q_B2_B1[:, 1]*q_B2_B1[:, 0] + q_B2_B1[:, 2]*q_B2_B1[:, 3])
        euler_beta = np.arcsin(np.clip(arcsin_arg, -1, 1))
        err = euler_beta - beta

        # Jacobian calculation
        # big array for derivatives of the final quaternion
        d_q = np.empty((N, 4, 5))

        # necessary quaternion multiplications
        q_B2_E2 = _qmult(q2, q_B2_S2)  # q45
        q_B2_E1 = _qmult(q_E2_E1, q_B2_E2)  # q345
        q_B2_S1 = _qmult(_qinv(q1), q_B2_E1)  # q2345

        # derivative of inv(q_B1_S1) wrt j1
        dj1_theta, dj1_phi = axisGradient(theta1, phi1, var1)

        den = np.sqrt(1 - j1_est[2]**2)
        if den == 0:
            den = 1e-8
        d_q1angle_theta = - dj1_theta[2] / den
        d_q1angle_phi = - dj1_phi[2] / den

        d_q1cos_theta = -0.5 * d_q1angle_theta * q1_sin
        d_q1cos_phi = -0.5 * d_q1angle_phi * q1_sin
        d_q1sin_theta = 0.5 * d_q1angle_theta * q1_cos
        d_q1sin_phi = 0.5 * d_q1angle_phi * q1_cos

        d_q1axisnorm_theta = (j1_est[1] * dj1_theta[1] + j1_est[0] * dj1_theta[0]) / q1_axis_norm
        d_q1axisnorm_phi = (j1_est[1] * dj1_phi[1] + j1_est[0] * dj1_phi[0]) / q1_axis_norm

        d_q1axis_theta = np.array([-dj1_theta[1], dj1_theta[0], 0], dtype=float)
        d_q1axis_phi = np.array([-dj1_phi[1], dj1_phi[0], 0], dtype=float)

        # note the minus here to calculate the derivative of inv(q_B1_S1) instead of q_B1_S1
        d_q1xyz_theta = - (q1_axis * q1_axis_norm * d_q1sin_theta + q1_sin * q1_axis_norm * d_q1axis_theta
                           - q1_sin * q1_axis * d_q1axisnorm_theta) / q1_axis_norm**2
        d_q1xyz_phi = - (q1_axis * q1_axis_norm * d_q1sin_phi + q1_sin * q1_axis_norm * d_q1axis_phi
                         - q1_sin * q1_axis * d_q1axisnorm_phi) / q1_axis_norm**2

        d_q1_theta = np.zeros((N, 4))
        d_q1_theta[:, 0] = d_q1cos_theta
        d_q1_theta[:, 1:4] = d_q1xyz_theta

        d_q1_phi = np.zeros((N, 4))
        d_q1_phi[:, 0] = d_q1cos_phi
        d_q1_phi[:, 1:4] = d_q1xyz_phi

        d_q[:, :, 0] = _qmult(d_q1_theta, q_B2_S1)  # d_q_theta1
        d_q[:, :, 1] = _qmult(d_q1_phi, q_B2_S1)  # d_q_phi1

        # derivative of q_B2_S2 wrt j2
        dj2_theta, dj2_phi = axisGradient(theta2, phi2, var2)

        den = np.sqrt(1 - j2_est[1]**2)
        if den == 0:
            den = 1e-8
        d_q2angle_theta = - dj2_theta[1] / den
        d_q2angle_phi = - dj2_phi[1] / den

        d_q2cos_theta = -0.5 * d_q2angle_theta * q2_sin
        d_q2cos_phi = -0.5 * d_q2angle_phi * q2_sin
        d_q2sin_theta = 0.5 * d_q2angle_theta * q2_cos
        d_q2sin_phi = 0.5 * d_q2angle_phi * q2_cos

        d_q2axisnorm_theta = (j2_est[2] * dj2_theta[2] + j2_est[0] * dj2_theta[0]) / q2_axis_norm
        d_q2axisnorm_phi = (j2_est[2] * dj2_phi[2] + j2_est[0] * dj2_phi[0]) / q2_axis_norm

        d_q2axis_theta = np.array([dj2_theta[2], 0, -dj2_theta[0]], dtype=float)
        d_q2axis_phi = np.array([dj2_phi[2], 0, -dj2_phi[0]], dtype=float)

        d_q2xyz_theta = (q2_axis * q2_axis_norm * d_q2sin_theta + q2_sin * q2_axis_norm * d_q2axis_theta
                         - q2_sin * q2_axis * d_q2axisnorm_theta) / q2_axis_norm**2
        d_q2xyz_phi = (q2_axis * q2_axis_norm * d_q2sin_phi + q2_sin * q2_axis_norm * d_q2axis_phi
                       - q2_sin * q2_axis * d_q2axisnorm_phi) / q2_axis_norm**2

        d_q2_theta = np.zeros((N, 4))
        d_q2_theta[:, 0] = d_q2cos_theta
        d_q2_theta[:, 1:4] = d_q2xyz_theta

        d_q2_phi = np.zeros((N, 4))
        d_q2_phi[:, 0] = d_q2cos_phi
        d_q2_phi[:, 1:4] = d_q2xyz_phi

        d_q[:, :, 2] = _qmult(q_S2_B1, d_q2_theta)  # d_q_theta2
        d_q[:, :, 3] = _qmult(q_S2_B1, d_q2_phi)  # d_q_phi2

        # derivative of q_E2_E1 wrt delta
        d_q3_delta = np.zeros((N, 4))
        d_q3_delta[:, 0] = -0.5*np.sin(delta/2)
        d_q3_delta[:, 3] = 0.5*np.cos(delta/2)
        d_q[:, :, 4] = _qmult(_qmult(q_E1_B1, d_q3_delta), q_B2_E2)

        jac = np.empty((N, 6))
        jac[:, 0:5] = 2*(d_q[:, 1]*q_B2_B1[:, 0, None] + q_B2_B1[:, 1, None]*d_q[:, 0] + d_q[:, 2]*q_B2_B1[:, 3, None]
                         + q_B2_B1[:, 2, None]*d_q[:, 3])

        # d/dx arcsin((f(x)) = f'(x) / sqrt(1-f(x)²)
        jac[:, 0:5] = jac[:, 0:5] / np.sqrt(1 - arcsin_arg**2)[:, None]

        # derivative wrt carrying angle
        jac[:, 5] = -1  # d_ca

        self._cache['err'] = err
        self._cache['jac'] = jac
        return err, jac

    def unpackX(self):
        assert self._x.shape == (8,)
        return {
            'j1': axisFromThetaPhi(self._x[0], self._x[1], self._x[6]),
            'j2': axisFromThetaPhi(self._x[2], self._x[3], self._x[7]),
            'delta': self._x[4],
            'beta': self._x[5],
        }

    @staticmethod
    def getInitVals(variant='default', seed=None):
        init = AbstractAxisEst2DObjectiveFunction.getInitVals(variant, seed)
        # insert zero column for beta angle
        return np.hstack([init[:, :5], np.zeros((init.shape[0], 1)), init[:, -2:]])



