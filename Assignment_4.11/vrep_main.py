# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import sys
import math
import numpy as np
from time import sleep

import vrep
import inverse_kinematics
import quaternion
import collision_detection
import path_planner

import time

print ('KUKA MOVER started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connected to KUKA Simulator')

    # Get the ID of the user-movable dummy to indicate desired position
    movable_dummy_handle = 420

    # Initial thetas:       1    2     3    4     5    6     7
    theta_goal = np.array([1.0, 1.57, 1.0, 1.57, 0.0, 1.57, 0.0])

    # Loop through all the robot's joints and get unique ID's for each
    joint_handles = []
    for i in range(7):
        joint_name = 'LBR_iiwa_7_R800_joint' + str(i + 1)
        errorCode, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_oneshot_wait)
        joint_handles.append(handle)

    # Get a handle for the movable inverse kinematics dummy
    errorCode, target_dummy_handle = vrep.simxGetObjectHandle(clientID, "TargetDummy", vrep.simx_opmode_oneshot_wait)

    # Get handles for all the movable collision detection dummies
    # Set number of obstacles
    NUM_OBSTACLES = 5

    dummy_obstacle_handles = []
    for i in range(0, NUM_OBSTACLES):
        dummy_name = 'ObstacleDummy' + str(i + 1)
        errorCode, dummy_obstacle_handles[i] = vrep.simxGetObjectHandle(clientID, dummy_name, vrep.simx_opmode_oneshot_wait)

    # Create bounding volume dummies for the joints
    BOUNDING_VOL_RADIUS = 0.15
    END_RADIUS = 0.05

    # Initialize variable which let us check if the dummy has been moved
    prev_dummy_pos = None

    try:
        while True:
            # Get the desired pose by checking the pose of the user-movable dummy
            errorCode, dummy_pos = vrep.simxGetObjectPosition(clientID, movable_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
            #print("Dummy pos: {}".format(dummy_pos))
            errorCode, dummy_quaternion = vrep.simxGetObjectQuaternion(clientID, movable_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
            #print("Dummy qua: {}".format(dummy_quaternion))

            # Make sure we don't recalculate if the inverse kinematics dummy is still in the same position
            if (dummy_pos == prev_dummy_pos):
                continue
            prev_dummy_pos = dummy_pos

            # Mush the quaternion and pos into a pose matrix!
            # Our function takes care of the V-REP quaternion mismatch
            pose = quaternion.matrix_from_quaternion(dummy_quaternion)
            pose[0,3] = dummy_pos[0]
            pose[1,3] = dummy_pos[1]
            pose[2,3] = dummy_pos[2]
            print("Desired Pose: \n{}".format(pose))

            # Get theta start (current thetas of robot joints)
            theta_start = np.zeros((7,1))
            for i in range(7):
                theta[i] = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)

            # Get theta goal
            theta_goal = inverse_kinematics.inverse_kinematics(pose)
            print("Theta goal: {}".format(theta_goal))

            # Get p_robot (position of each robot's collision spheres)
            p_robot = np.zeros((3,8))
            for i in range(1,8):
                p_robot[0:3,i] = vrep.simxGetObjectPosition(clientID, joint_handles[i], -1, vrep.simx_opmode_oneshot_wait)

            # Get r_robot (radius of robot's joints)
            r_robot = np.zeros((1,8)).fill(BOUNDING_VOL_RADIUS)
            r_robot[0,7] = END_RADIUS

            # Get p_obstacle (position of external obstacles)
            p_obstacle = np.zeros((3,NUM_OBSTACLES))
            for i in range(0, NUM_OBSTACLES):
                p_obstacle[:,i] = vrep.simxGetObjectPosition(clientID, dummy_obstacle_handles[i], -1, vrep.simx_opmode_oneshot_wait)

            # Get r_obstacle (radius of each external obstacle)
            r_obstacle = np.zeros((1,NUM_OBSTACLES)).fill(BOUNDING_VOL_RADIUS)

            # Plan a path!
            max_iterations = 420
            final_path = path_planner.plan_my_path(p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal, max_iterations)

            # If we generated a valid path, iterate through it and move our robot!
            print(len(final_path[0]))
            if final_path is not False:
                print("Moving robot to goal theta!")
                for i in range(len(final_path[0])):   # Iterate through each point in our path
                    print("Waypoint {}".format(i))
                    for j in range(7):                     # Iterate through every joint on our robot
                        vrep.simxSetJointPosition(clientID, joint_handles[j], final_path[j:i], vrep.simx_opmode_oneshot_wait)
                        sleep(0.025)
            else:
                print("Viable path not found in {} iterations".formatmax_iterations())

    except KeyboardInterrupt:
        for i in range(7):
            vrep.simxSetJointPosition(clientID, joint_handles[i], 0, vrep.simx_opmode_oneshot_wait)

    sleep(0.5)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection Failed")
print ('Program ended')
