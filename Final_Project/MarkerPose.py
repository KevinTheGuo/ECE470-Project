# This file contains code which uses the Aruco library to grab marker corners and IDs, and then
# uses that data to derive marker poses, using our own ECE470 code! Runs quite well in realtime.
#/home/lfsony/.local/lib/python3.5/site-packages/cv2"
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm
from scipy.stats import special_ortho_group

import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/home/lfsony/.local/lib/python3.5/site-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)
# foo.MyClass()
#
# import cv2
import cv2.aruco

# DEFAULT ECE470 LIBRARY CODE-----------------------------------------------------------------------------------------------
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

standard_projection = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0]])

# HW6.2 MARKER POSE ESTIMATION----------------------------------------------------------------------------------------------
def eta(array):
    result = np.ones((3,1))
    result[0] = array[0]/array[2]
    result[1] = array[1]/array[2]
    return result.flatten()

def eta_d(array):
    d = np.matrix([[1/array[2],0,-array[0]/(array[2])**2],
                   [0,1/array[2],-array[1]/(array[2])**2],
                   [0,0         ,0                      ]])
    return d

# Condensed HW6.2 code to a function that finds a marker pose given a marker length (w), a matrix of camera
# calibration numbers (k), and a matrix of corners (q).
# returns two variables- an integer num_iterations tracking the number of iterations to converage,
#                        and a pose matrix if numIterations is not -1
def markerPoseFinder(w, K, q):
    MIN_ERR = 1e-2 # Defined constants
    MU = 1e-3

    p_intag = np.array([[-w/2,w/2,w/2,-w/2],[w/2,w/2,-w/2,-w/2],[0,0,0,0],[1,1,1,1]])  # Get p of the tag frame, based on w
    q_inimage = np.ones((3,4))  # Put q in homogenous coordinates
    q_inimage[0:2,0:4] = q

    x = np.eye(4,4)  # Make first pose guess
    x[0:3,3:4] = np.array([[0],[0],[0.5]])

    iterations = 0
    while(True):  # Levenberg-Marquardt
        iterations += 1

        # Initialize variables
        u_sum1, u_sum2 = 0,0
        b = np.zeros((4,1))
        J = np.zeros((4,6))
        u = np.zeros((6,1))

        for i in range(4):  # Iterate over each point and the corresponding pair, adding to our sum variables
            q_point = q_inimage[0:3,i]
            p_point = p_intag[0:4,i]

            b[0:3,0] = q_point - eta(K @ standard_projection @ x @ p_point)   # Calculate b
            for j in range(6):                       # Calculate J (df/du)
                basis = np.eye(6,6)[0:6,j:j+1]
                J[0:3,j] = eta_d(K @ standard_projection @ x @ p_point) @ K @ standard_projection @ bracket(basis) @ x @ p_point
            u_sum1 += np.transpose(J)@J  # Add our contribution to the two sums that help build u
            u_sum2 += np.transpose(J)@b

        u[0:6,0:1] = inv(u_sum1 + MU*np.eye(6,6)) @ u_sum2  # Finish calculating u from its sums
        if norm(u) < MIN_ERR:  # Check to see if u is within our error tolerance
            break

        x = expm(bracket(u)) @ x   # If u isn't within error tolerance, update our guess x
        if iterations > 15:   # If we exceed a set number of iterations, just end
            return -1, None

    return iterations, x

# CODE FOR ECE470 FINAL PROJECT- takes in frame, draws on it and returns----------------------------------------------------

# Add our calculated camera calibration variables
camera_matrix = np.array( [[ 612.28318156,    0.        ,  377.95149893],
                           [   0.        ,  603.72201353,  273.9850609 ],
                           [   0.        ,    0.        ,    1.        ]])

dist_coeffs = np.array(   [[ 0.45710893, -0.68904653,  0.05444916, -0.01899903, -1.79491808]])

# Generate our specific aruco dict
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

# Function for the ECE470 final project that makes use of Aruco to find marker IDs and corners, but utilizes our custom
# ECE470 code to find the marker pose. Only works with one marker on screen currently.
# Returns:  frame - modified frame with information, marker and pose drawn
#           isValid - -1 if marker pose not found (marker may be visible), 1 if valid marker pose found
#           pose - the pose of the marker, if valid.
def findAndDrawMarkers(frame):
    # Detect and display markers and their poses in our video frame
    corners, ids, rejected_corners = cv2.aruco.detectMarkers(frame, aruco_dict)
    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    if (len(corners) > 0):
        num_iterations, custom_pose = markerPoseFinder(.066, camera_matrix, np.transpose(corners[0][0]))
        # print("num iterations is {}".format(num_iterations))
        if num_iterations != -1:
            custom_pose = np.round(custom_pose, decimals=3)
            custom_rotation = custom_pose[0:3,0:3]
            custom_translation = custom_pose[0:3,3]
            custom_rvec, jacobian = cv2.Rodrigues(custom_rotation)
            #print("Custom pose is rotation: \n{}, translation: {}".format(custom_rotation, custom_translation))
            #print("Custom rvec is: {}".format(custom_rvec))
            frame = cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, custom_rvec, custom_translation, 0.05)
            cv2.putText(frame, "Translation: {}".format(custom_translation), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 2)
            cv2.putText(frame, "Rotation: {}".format(custom_rotation[0]), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 2)
            cv2.putText(frame, "{}".format(custom_rotation[1]), (75, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 2)
            cv2.putText(frame, "{}".format(custom_rotation[2]), (75, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 2)
            return frame, 1, custom_pose

    return frame, -1, None

# Comparison Code: What we'd get if we used Aruco's pose
#             rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 1, camera_matrix, dist_coeffs)
#             rotation_matrix, jacobian = cv2.Rodrigues(rvecs)
#             print("OpenCV pose is rotation: \n{}, translation: {}".format(rotation_matrix, tvecs[0][0]))
#             print("OpenCV rvec is: {}".format(rvecs[0][0]))
