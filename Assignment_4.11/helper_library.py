# File with all of our helper functions
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm
import scipy.linalg as sl

def bracket(v):
    """
    Returns the 'bracket' operator of a 3x1 vector or 6x1 twist
    :param v: the 3x1 vector or 6x1 twist, can be of type list or numpy.ndarray - Must be convertible to a numpy array!
    :returns: a 3x3 or 4x4 numpy array based on the input matrix or an empty list otherwise
    """
    v = np.asarray(v)
    rtn = []
    if(v.shape == (6,1)):
        rtn = np.block([[ bracket(v[:3]),  v[3:]   ],
                        [ np.zeros((1,4))          ]])
    elif(v.shape == (3,1)):
        rtn = np.zeros((3,3))
        rtn[0][1] = - v[2]
        rtn[0][2] =   v[1]
        rtn[1][2] = - v[0]
        rtn = rtn - rtn.transpose()
    return rtn

def toTs(S, theta):
    """
    Generates a list of HCT matricies from a list of screw axes and joint variables. Not that useful for general work,
    but used by other functions. Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix)
    :param S: A python list of 6x1 screw axes
    :param theta: A list/numpy array of joint vars. Should have the same number of elements as S
    :returns: A python list of 4x4 HCT matricies representing a transformation by each of the screw axes
    """
    return [expm(bracket(s) * t) for s, t in zip(S, theta)]

def evalT(S, theta, M):
    """
    Basically Forward Kinematics
    Finds the end position of a robot based on space screw axes, joint vars and the space 'zero HCT'
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix)
    :param S: A python list of 6x1 screw axes from the base to the manipulator
    :param theta: A python list/numpy array of joint vars in the same order as S.
    :param M: A 4x4 HCT transformation matrix representing the pose of the end effector when theta = 0 for all joint vars
    :returns: A numpy 4x4 HCT transformation matrix representing the pose of the end effector at the given joint vars
    """
    ret = np.identity(4)
    for t in toTs(S, theta):
        ret = ret.dot(t)
    return ret.dot(M)
