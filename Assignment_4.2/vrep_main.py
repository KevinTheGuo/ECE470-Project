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

import time

print ('KUKA MOVER started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connected to KUKA Simulator')

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

    # Get a handle for the movable inverse kinematics dummy
    errorCode, movable_dummy_handle = vrep.simxGetObjectHandle(clientID, "Dummy", vrep.simx_opmode_oneshot_wait)

    # Get handles for all the movable collision detection dummies
    errorCode, movable_dummy_handle1 = vrep.simxGetObjectHandle(clientID, "Dummy1", vrep.simx_opmode_oneshot_wait)
    errorCode, movable_dummy_handle2 = vrep.simxGetObjectHandle(clientID, "Dummy2", vrep.simx_opmode_oneshot_wait)
    errorCode, movable_dummy_handle3 = vrep.simxGetObjectHandle(clientID, "Dummy3", vrep.simx_opmode_oneshot_wait)
    errorCode, movable_dummy_handle4 = vrep.simxGetObjectHandle(clientID, "Dummy4", vrep.simx_opmode_oneshot_wait)
    errorCode, movable_dummy_handle5 = vrep.simxGetObjectHandle(clientID, "Dummy5", vrep.simx_opmode_oneshot_wait)

    # Create bounding volume dummies for the joints
    BOUNDING_VOL_RADIUS = 0.15
    END_RADIUS = 0.05
    errorCode, bounding_vol1_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol2_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol3_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol4_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol5_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol6_handle = vrep.simxCreateDummy(clientID, BOUNDING_VOL_RADIUS, None, vrep.simx_opmode_oneshot_wait)
    errorCode, bounding_vol7_handle = vrep.simxCreateDummy(clientID, END_RADIUS, None, vrep.simx_opmode_oneshot_wait)

    # Initialize variable which let us check if the dummy has been moved
    prev_dummy_pos = None

    try:
        while True:
            errorCode, pos = vrep.simxGetObjectPosition(clientID, joint_handles[6], -1, vrep.simx_opmode_oneshot_wait)
            #print("Initial end effector position:", pos)

            # input("Press any key to move robot to desired location")

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
            print("Pose is: \n{}".format(pose))

            # Get the thetas required to move the robot to the desired position
            theta_list = inverse_kinematics.inverse_kinematics(pose)
            print("Thetas are: {}".format(theta_list))
            if theta_list is not None:
                print("Moving robot")
                for i in range(7):              # Set the position of each joint
                    vrep.simxSetJointPosition(clientID, joint_handles[i], theta_list[i], vrep.simx_opmode_oneshot_wait)
                    # print("Setting joint", i+1, "to", theta_list[i])
                    sleep(0.25)

            # Grab the actual pose
            errorCode, pos = vrep.simxGetObjectPosition(clientID, joint_handles[6], joint_handles[0], vrep.simx_opmode_oneshot_wait)
            errorCode, angles = vrep.simxGetObjectQuaternion(clientID, joint_handles[6], joint_handles[0], vrep.simx_opmode_oneshot_wait)
            print("Actual pos:", pos)
            print("Actual qua:", angles)

            # Print each joint's angle
            for i in range(7):
                theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)
                print("Theta", i, "is", theta)

            # Attach each dummy bounding volume to each joint of the robot
            vrep.simxSetObjectPosition(clientID, bounding_vol1_handle, joint_handles[0], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol2_handle, joint_handles[1], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol3_handle, joint_handles[2], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol4_handle, joint_handles[3], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol5_handle, joint_handles[4], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol6_handle, joint_handles[5], (0,0,0), vrep.simx_opmode_oneshot_wait)
            vrep.simxSetObjectPosition(clientID, bounding_vol7_handle, joint_handles[6], (0,0,0), vrep.simx_opmode_oneshot_wait)

            # Run self-collision detection for the robot
            collision_detected = False
            for i in range(7):
                errorCode, pos_i = vrep.simxGetObjectPosition(clientID, joint_handles[i], -1, vrep.simx_opmode_oneshot_wait)
                for j in range(i+1,7):
                    errorCode, pos_j = vrep.simxGetObjectPosition(clientID, joint_handles[j], -1, vrep.simx_opmode_oneshot_wait)
                    RADIUS_I = BOUNDING_VOL_RADIUS/2
                    RADIUS_J = BOUNDING_VOL_RADIUS/2
                    if i == 6:
                        RADIUS_I = END_RADIUS/2
                    if j == 6:
                        RADIUS_J = END_RADIUS/2

                    if (collision_detection.check_collision(pos_i, RADIUS_I, pos_j, RADIUS_J)):
                        print("Robot joint {} collides with robot joint {}".format(i,j))
                        print("Joint{} pos: {} Joint{} pos: {}".format(i, pos_i, j, pos_j))
                        collision_detected = True

            # Run self-collision detection between the robot and the environment sheres
            dummy_pos_list = [0,0,0,0,0]
            errorCode, dummy_pos_list[0] = vrep.simxGetObjectPosition(clientID, movable_dummy_handle1, -1, vrep.simx_opmode_oneshot_wait)
            errorCode, dummy_pos_list[1] = vrep.simxGetObjectPosition(clientID, movable_dummy_handle2, -1, vrep.simx_opmode_oneshot_wait)
            errorCode, dummy_pos_list[2] = vrep.simxGetObjectPosition(clientID, movable_dummy_handle3, -1, vrep.simx_opmode_oneshot_wait)
            errorCode, dummy_pos_list[3] = vrep.simxGetObjectPosition(clientID, movable_dummy_handle4, -1, vrep.simx_opmode_oneshot_wait)
            errorCode, dummy_pos_list[4] = vrep.simxGetObjectPosition(clientID, movable_dummy_handle5, -1, vrep.simx_opmode_oneshot_wait)
            for i in range(7):
                errorCode, pos_i = vrep.simxGetObjectPosition(clientID, joint_handles[i], -1, vrep.simx_opmode_oneshot_wait)
                for j in range(5):
                    if (collision_detection.check_collision(pos_i, BOUNDING_VOL_RADIUS/2, dummy_pos_list[j], BOUNDING_VOL_RADIUS/2)):
                        print("Robot joint {} collides with object {}!".format(i,j))
                        print("Joint pos: {} Object pos: {}".format(pos_i, dummy_pos_list[j]))
                        collision_detected = True

            if not collision_detected:
                print("No collisions detected!")

    except KeyboardInterrupt:
        input("Press any key to revert to origin...")
        print("")

        # Remove all the dummy bounding volumes
        vrep.simxRemoveObject(clientID,bounding_vol1_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol2_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol3_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol4_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol5_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol6_handle,vrep.simx_opmode_oneshot_wait)
        vrep.simxRemoveObject(clientID,bounding_vol7_handle,vrep.simx_opmode_oneshot_wait)

        for i in range(7):
            vrep.simxSetJointPosition(clientID, joint_handles[i], 0, vrep.simx_opmode_oneshot_wait)
            print("Setting joint", i + 1, "back to 0")

    sleep(0.5)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection Failed")
print ('Program ended')
