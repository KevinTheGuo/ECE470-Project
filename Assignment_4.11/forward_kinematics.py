# Kuka Forward Kinematics
# All measurements are in meters and radians
import math
import numpy as np
import scipy.linalg as syl

def skew_sym(arr):
    return np.array([[0, -arr[2], arr[1]], [arr[2], 0, -arr[0]], [-arr[1], arr[0], 0]])


def matrix_rep(arr):
    matrix = np.zeros((4, 4))
    matrix[0:3, 0:3] = skew_sym(arr[0:3, 0:1])
    matrix[0:3, 3:4] = arr[3:6, 0:1]
    return matrix


def forwardKinematics(theta):
    M = np.array([[1., 0., 0., 0.],
                  [0., 1., 0., 0.],
                  [0., 0., 1., 1.266],
                  [0., 0., 0., 1.]])

    S = np.array([[0., 0., 0., 0., 0., 0., 0.],
                  [0., 1., 0., -1., 0., 1., 0.],
                  [1., 0., 1., 0., 1., 0., 1.],
                  [0., -0.34, 0., 0.74, 0., -1.14, 0.],
                  [0., 0., 0., 0., 0., 0., 0.],
                  [0., 0., 0., 0., 0., 0., 0.]])

    T = (syl.expm(matrix_rep(S[0:6, 0:1]) * theta[0]) @
         syl.expm(matrix_rep(S[0:6, 1:2]) * theta[1]) @
         syl.expm(matrix_rep(S[0:6, 2:3]) * theta[2]) @
         syl.expm(matrix_rep(S[0:6, 3:4]) * theta[3]) @
         syl.expm(matrix_rep(S[0:6, 4:5]) * theta[4]) @
         syl.expm(matrix_rep(S[0:6, 5:6]) * theta[5]) @
         syl.expm(matrix_rep(S[0:6, 6:7]) * theta[6]) @ M)

    return T


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)


def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2,0],-1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0,1],R[0,2])
    elif isclose(R[2,0],1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0,1],-R[0,2])
    else:
        theta = -math.asin(R[2,0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
        phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
    return (psi, theta, phi)


def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix."""
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    if isprecise:
        q = np.empty((4, ))
        t = np.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
            q = q[[3, 0, 1, 2]]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = np.linalg.eigh(K)
        q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q
