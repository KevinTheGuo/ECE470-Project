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


import time

print ('KUKA MOVER started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connected to KUKA Simulator')

    # Make a Dummy
    errorCode, dummy_handle_0 = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)

    # Get the ID of the user-movable dummy to indicate desired position
    movable_dummy_handle = 420

    # Initial thetas:       1    2     3    4     5    6     7
    theta_list = np.array([1.0, 1.57, 1.0, 1.57, 0.0, 1.57, 0.0])

    # Loop through all the robot's joints and get unique ID's for each
    joint_handles = []
    for i in range(7):
        joint_name = 'LBR_iiwa_7_R800_joint' + str(i + 1)
        errorCode, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_oneshot_wait)
        joint_handles.append(handle)

    errorCode, movable_dummy_handle = vrep.simxGetObjectHandle(clientID, "Dummy", vrep.simx_opmode_oneshot_wait)

    # Set the dummy to tool end effector
    vrep.simxSetObjectPosition(clientID, dummy_handle_0, joint_handles[6], (0, 0, 0), vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(clientID, dummy_handle_0, -1, (0, 0, 0), vrep.simx_opmode_oneshot_wait)

    errorCode, pos = vrep.simxGetObjectPosition(clientID, joint_handles[6], -1, vrep.simx_opmode_oneshot_wait)
    print("Initial end effector position:", pos)

    input("Press any key to move robot to desired location")

    # Get the desired pose by checking the pose of the user-movable dummy
    errorCode, dummy_pos = vrep.simxGetObjectPosition(clientID, movable_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
    print("Dummy pos: {}".format(dummy_pos))
    errorCode, dummy_quaternion = vrep.simxGetObjectQuaternion(clientID, movable_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
    print("Dummy qua: {}".format(dummy_quaternion))

    # Mush the quaternion and pos into a pose matrix!
    # Our function takes care of the V-REP quaternion mismatch
    pose = quaternion.matrix_from_quaternion(dummy_quaternion)
    pose[0,3] = dummy_pos[0]
    pose[1,3] = dummy_pos[1]
    pose[2,3] = dummy_pos[2]
    print("Pose is: {}".format(pose))

    # Get the thetas required to move the robot to the desired position
    theta_list = inverse_kinematics.inverse_kinematics(pose)
    print("Thetas are: {}".format(theta_list))
    if theta_list is not None:
        print("Moving robot")
        for i in range(7):              # Set the position of each joint
            vrep.simxSetJointTargetPosition(clientID, joint_handles[i], theta_list[i], vrep.simx_opmode_oneshot_wait)
            # print("Setting joint", i+1, "to", theta_list[i])
            sleep(0.5)

    # Add a new dummy on the moved robot end effector so we can compare the desired and actual positions
    errorCode, dummy_handle_1 = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectPosition(clientID, dummy_handle_1, joint_handles[6], (0, 0, 0), vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(clientID, dummy_handle_1, joint_handles[6], (0, 0, 0), vrep.simx_opmode_oneshot_wait)

    # Grab the actual pose
    errorCode, pos = vrep.simxGetObjectPosition(clientID, joint_handles[6], joint_handles[0],
                                                vrep.simx_opmode_oneshot_wait)
    errorCode, angles = vrep.simxGetObjectQuaternion(clientID, joint_handles[6], joint_handles[0],
                                                     vrep.simx_opmode_oneshot_wait)
    print("Actual pos:", pos)
    print("Actual qua:", angles)

    for i in range(7):
        theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)
        print("Theta", i, "is", theta)

    input("Press any key to revert to origin...")
    print("")

    for i in range(7):
        vrep.simxSetJointTargetPosition(clientID, joint_handles[i], 0, vrep.simx_opmode_oneshot_wait)
        print("Setting joint", i + 1, "back to 0")

    sleep(0.5)

    vrep.simxRemoveObject(clientID, dummy_handle_0, vrep.simx_opmode_oneshot_wait)
    vrep.simxRemoveObject(clientID, dummy_handle_1, vrep.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection Failed")
print ('Program ended')
