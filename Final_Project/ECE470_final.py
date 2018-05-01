# This file, written in Python3, opens a webcam and analyzes the input frame-by-frame, calculating the pose
# of any Aruco marker detected. Currently it can only check the pose of one at a time. If it finds a pose, it will
# use inverse kinematics to calculate the joint angles required to move the Kuka to there. If the inverse kinematics
# calculation converges, this code will then open robot.py as a subprocesses, using that code, written in Python2, to
# move the robot joints by passing it command-line arguments.

# Import CV2 with importlib magic
import math #we need this to get math.pi for rad -> deg conversion
import importlib.util
spec = importlib.util.spec_from_file_location("cv2", "/home/lfsony/.local/lib/python3.5/site-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so")
cv2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cv2)

import subprocess        # Import subprocess to call python2 code robot.py
import numpy as np       # Numpy for transformation stuff
from scipy.stats import special_ortho_group

# Import our other helper files
import MarkerPose
import inverse_kinematics
import sys

# Transformation matrix T_camInBot
T_camInBot = np.array([[-1, 0, 0, -.07 ],
                       [ 0, 0,-1,-0.07],
                       [ 0,-1, 0, 0.05],
                       [ 0, 0, 0, 1   ]])

print('Starting up!')

# Try to grab video input
video = cv2.VideoCapture(1)
# Exit if video not opened.
if not video.isOpened():
    print('Could not open video!')
    sys.exit()

try:
    while(True):
        # Check for user input
        key = cv2.waitKey(1) & 0xff
        if key == 27:
            print('Exiting program!')
            break

        # Grab another frame
        read_success, frame = video.read()
        if not read_success:
            print('Cannot read video file!')
            break

        # Find the marker pose and draw stuff.
        frame, isValid, T_markInCam = MarkerPose.findAndDrawMarkers(frame)
        cv2.imshow("ECE470 Final Project", frame)
        # If we found a marker pose and converged to it, then move the robot there!
        if (isValid != -1):
            # First, transform the pose in camera frame to be in the robot frame
            print("----------------------------------------------------------------------")
            # Also, make the end effector pose straight, and not dependent on the marker pose
            T_markInCam[0:3,0:3] = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
            T_markInBot = T_camInBot @ T_markInCam
            print(T_markInBot)
            # Also, make the end effector pose straight, and not dependent on the marker pose
            # T_markInBot[0:3,0:3] = np.array([[-1,0,0],[0,-1,0],[0,0,1]])

            thetas = inverse_kinematics.inverse_kinematics(T_markInBot, iterationMax=15)  # inverse kinematics
            if thetas is not None:          # Make sure that inverse kinematics has converged.
                print("Inverse kinematics has converged! First theta list: ")
                for i in range(7):
                    thetas[i] = thetas[i] * (180 / math.pi)  # Convert thetas (rad -> degrees)
                    while thetas[i] <= -180:
                        thetas[i] += 360
                    while thetas[i] > 180:
                        thetas[i] -= 360
                    print("theta_", i+1, "is", thetas[i])

            #     # Call robot.py with specific command-line arguments, to move the robot to those joint angles
            #     subprocess.call("python robot.py {} {} {} {} {} {} {}"
            #                     .format(thetas[0],thetas[1],thetas[2],thetas[3],thetas[4],thetas[5],thetas[6]), shell=True)
            #
            #     sleep(20)   # sleep for a while

except (Exception, KeyboardInterrupt) as e:
    print('Exception encountered!')
    video.release()
    cv2.destroyAllWindows()
    raise e

video.release()
cv2.destroyAllWindows()
