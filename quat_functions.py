### Functions for application to quaternions

import numpy as np


# Find average quaternion from an array of quaternions. Note: be aware of different quaternion averaging methods
def average_quaternions(quaternions):
    """
    Calculate average quaternion

    :params quaternions: is a Nx4 numpy matrix and contains the quaternions
        to average in the rows.
        The quaternions are arranged as (w,x,y,z), with w being the scalar

    :returns: the average quaternion of the input. Note that the signs
        of the output quaternion can be reversed, since q and -q
        describe the same orientation
    """

    # Number of quaternions to average
    samples = quaternions.shape[0]
    mat_a = np.zeros(shape=(4, 4), dtype=np.float64)

    for i in range(0, samples):
        quat = quaternions[i, :]
        # multiply quat with its transposed version quat' and add mat_a
        mat_a = np.outer(quat, quat) + mat_a

    # scale
    mat_a = (1.0/ samples)*mat_a
    # compute eigenvalues and -vectors
    eigen_values, eigen_vectors = np.linalg.eig(mat_a)
    # Sort by largest eigenvalue
    eigen_vectors = eigen_vectors[:, eigen_values.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(np.ravel(eigen_vectors[:, 0]))



def average_quats_Gramkow(quaternions):
    '''
    This quaternion averaging method, from Gramkow, 2001,
    simply takes sum of each q0, q1, q2, q3 component, for every quaternion
    in the array, then normalises by dividing by the resultant magnitude.
    Appropriate for averaging quaternions which are relatively close.
    '''

    quat0 = sum(quaternions[i, 0] for i in range(len(quaternions)))
    quat1 = sum(quaternions[i, 1] for i in range(len(quaternions)))
    quat2 = sum(quaternions[i, 2] for i in range(len(quaternions)))
    quat3 = sum(quaternions[i, 3] for i in range(len(quaternions)))
    quat_avg = [quat0, quat1, quat2, quat3]
    quat_avg_norm = quat_avg / np.linalg.norm(quat_avg)
    return quat_avg_norm

# Calculate angular velocity vectors from quaternion data_out
def ang_vel_from_quats(q1, q2, dt):
    vel = (2 / dt) * np.array([
        q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2],
        q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1],
        q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]])
    return vel


# Define a function for quaternion multiplication
def quat_mul(Q0, Q1):
    """
    Multiplies two quaternions.
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31)
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32)
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)
    """
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
    return final_quaternion


def quat_dot_prod(Q0, Q1):
    """
    Multiplies two quaternions by dot product.
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31)
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32)
    """

    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]

    prod = w0*w1 + x0*x1 + y0*y1 + z0*z1

    return prod

# Calculate quaternion conjugate
def quat_conj(Q0):
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
    output_quaternion = np.array([w0, -x0, -y0, -z0])
    return output_quaternion