# The default helper library
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm

"""
ece470_lib.py

Created for ECE 470 CBTF Exams

Portions of this library will be included with the exams.
Import the library into Jupyter Notebook by placing this file in the same directory as your notebook.
Then run:
import ece470_lib as ece470
and then you can use the functions as
ece470.bracket(np.ones(6,1))

Hosted at https://github.com/namanjindal/ECE-470-Library
If you would like to contribute, reply to @289 on Piazza with your github username

------------
Created by Naman Jindal
March 2018

"""
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

def inv_bracket(m):
    """
    Performs the inverse 'bracket' operation on a 3x3 or 4x4 matrix
    :param m: the 3x3 skew-symmetric matrix or 4x4 bracket of a twist - Must be convertible to a numpy array!
    :returns: the vector or twist representation of the input matrix or an empty list otherwise
    """
    rtn = []
    m = np.asarray(m)
    if(m.shape == (4,4)):
        rtn = np.block([[ inv_bracket(m[:3,:3])],
                        [ m[:3,3:]             ]])
    elif(m.shape == (3,3)):
        m = m - m.transpose()
        rtn = np.zeros((3,1))
        rtn[2] = - m[0][1]/2
        rtn[1] =   m[0][2]/2
        rtn[0] = - m[1][2]/2
    return rtn

def adj_T(T):
    """
    Returns the adjoint transformation matrix of T
    :param T: the pose whose 6x6 adjoint matrix to return
    """
    rot, pos = fromPose(T)
    return np.block([[ rot,                   np.zeros((3,3)) ],
                     [ bracket(pos).dot(rot), rot             ]])

def toPose(rot, pos):
    """
    Returns a 4x4 HCT matrix given by the 3x3 rotation matrix and 3x1 postion vector
    :param rot: A 3x3 Rotation Matrix
    :param pos: A 3x1 Position Vector
    :returns: A 4x4 HTC matrix as a numpy array
    """
    return np.block([[ rot, pos  ],
                     [ [0,0,0,1] ]])

def fromPose(T):
    """
    Returns a rotation matrix and position vector from a 4x4 HCT matrix
    :param T: The 4x4 HCT matrix as either python lists or numpy array
    :returns: a tuple with the first element being a 3x3 numpy array representing the rotation matrix
              and the second element being a 3x1 numpy array position vector
    """
    T = np.asarray(T)
    return (T[:3,:3], T[:3, 3:4])

def toScrew(a, q=None):
    """
    Returns the space screw of some prismatic or revolute joint as a 6x1 numpy array.
    If a q is supplied, the returned screw will be revolute; if no q, screw will be prismatic.
    Can use either python list, list of lists, or numpy array as inputs in XYZ order
    :param a: The axis of motion for a prismatic screw or axis of revolution. Should have norm 1 (not checked)
    :param q: A point passing through the axis if a revolute joint
    :returns: A 6x1 numpy matrix representing the screw axis
    """
    a = np.atleast_2d(a).reshape((3,1))
    # Revolute Screw
    if q is not None:
        q = np.atleast_2d(q).reshape((3,1))
        return np.block([[ a                 ],
                         [ bracket(q).dot(a) ]])
    # Prismatic Screw
    return np.block([[ np.zeros((3,1)) ],
                     [ a               ]])

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

def evalJ(S, theta):
    """
    Finds the space jacobian of a robot with given screw axes at a given joint positions:
    Note that numpy arrays of screw axes are not supported, only python lists of screw axes.
    Use np.hsplit(S, N) to generate a list of screw axes given a numpy array S where N is the number of joints (cols in the matrix)
    TODO: Improve efficeny by removing the need to recompute the transformation for each screw
    :param S: a python list of 6x1 screw axes
    :param theta: a python list/numpy array of joint vars. Should be same number of elements as S
    :returns: A 6xN matrix representing the space Jacobian of the robot with the given screw axes at the given joint vars
    """
    T = toTs(S, theta)
    J = [S[0]]
    for i in range(1, len(S)):
        col = T[0]
        for j in range(1, i):
            col = col.dot(T[j])
        J.append(adj_T(col).dot(S[i]))
    return np.hstack(J)
