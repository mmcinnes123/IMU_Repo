
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

    q_E2_E1 = np.array([np.cos(delt a /2), 0, 0, np.sin(delt a /2)], float)

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
                 + e_z[None, :] * (inner1d(e_z, j2_e2 ) *np.sin(delta))[:, None])
    dwd_delta = -(-w2_e2 * np.sin(delta)
                  + _cross(e_z[None, :], w2_e2) * np.cos(delta)
                  + e_z[None, :] * (inner1d(e_z, w2_e2) * np.sin(delta))[:, None])
    d_ax_orig_delta = _cross(j1_e1, dj2_delta)
    d_ax_delta = d_ax_orig_delta / ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_delta)[:, None] / ax_norm ** 3
    d_delta = inner1d(dwd_delta, ax) + inner1d(w_d, d_ax_delta)

    # Find partial derivatives wrt the joint axes (theta1, phi1, theta2, phi2)
    d_ax_orig_theta1 = _cross(_rotate(q1, dj1_theta), j2_e1)
    d_ax_theta1 = d_ax_orig_theta 1 /ax_norm - ax_orig * inner1d(ax_orig, d_ax_orig_theta1)[:, None] / ax_nor m* *3
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