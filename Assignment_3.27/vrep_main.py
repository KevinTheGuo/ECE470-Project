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

try:
    import vrep
    import forward_kinematics
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

print ('KUKA MOVER started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID != -1:
    print ('Connected to KUKA Simulator')

    # Make a Dummy
    errorCode, dummy_handle_0 = vrep.simxCreateDummy(clientID,0.1,None,vrep.simx_opmode_oneshot_wait)

    # Get the ID of the user-movable dummy to indicate desired position
    movable_dummy_handle = 420

    # Initial thetas:       1   2    3    4   5   6    7
    theta_list = np.array([1.0,1.57,1.0,1.57,0.0,1.57,0.0])

    # Loop through all the robot's joints and get unique ID's for each
    joint_handles = []
    for i in range(7):
        joint_name = 'LBR_iiwa_7_R800_joint' + str(i + 1)
        errorCode, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_oneshot_wait)
        joint_handles.append(handle)

    #Set the dummy to tool end effector
    vrep.simxSetObjectPosition(clientID, dummy_handle_0, joint_handles[6], (0,0,0), vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(clientID,dummy_handle_0,-1,(0,0,0),vrep.simx_opmode_oneshot_wait)

    errorCode, pos = vrep.simxGetObjectPosition(clientID,joint_handles[6],-1,vrep.simx_opmode_oneshot_wait)
    print("Initial end effector pos:",pos)

    # Get the desired position by checking the pose of the user-movable dummy
    errorCode, pos = vrep.simxGetObjectPosition(clientID, movable_dummy_handle , joint_handles[0], vrep.simx_opmode_oneshot_wait)
    errorCode, quaternion = vrep.simxGetObjectQuaternion(clientID, movable_dummy_handle, joint_handles[0], vrep.simx_opmode_oneshot_wait)



    # Show our predicted end effector position
    pose = forward_kinematics.forwardKinematics(theta_list)
    quaternion = forward_kinematics.quaternion_from_matrix(pose)
    temp = quaternion[0]
    quaternion[0] = quaternion[1]
    quaternion[1] = quaternion[2]
    quaternion[2] = quaternion[3]
    quaternion[3] = temp
    position = (pose[0, 3], pose[1, 3], pose[2, 3] - 0.193424112)
    print("Theta:",theta_list)
    print("Calculated qua:",quaternion)

    # Show predicted position
    errorCode, dummy_handle_1 = vrep.simxCreateDummy(clientID,0.1,None,vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectPosition(clientID, dummy_handle_1, joint_handles[0], position, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectQuaternion(clientID,dummy_handle_1,joint_handles[0],quaternion,vrep.simx_opmode_oneshot_wait)
    # print("\nEULER!",euler_angles)

    input("Press any key to move to predicted location")

    # Set the position of each joint
    for i in range (7):
        vrep.simxSetJointTargetPosition(clientID,joint_handles[i],theta_list[i], vrep.simx_opmode_oneshot_wait)
        # print("Setting joint", i+1, "to", theta_list[i])

    sleep(2)

    errorCode, dummy_handle_3 = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectPosition(clientID,dummy_handle_3,joint_handles[6],(0,0,0),vrep.simx_opmode_oneshot_wait)
    vrep.simxSetObjectOrientation(clientID,dummy_handle_3,joint_handles[6],(0,0,0),vrep.simx_opmode_oneshot_wait)

    errorCode, pos = vrep.simxGetObjectPosition(clientID,joint_handles[6],joint_handles[0],vrep.simx_opmode_oneshot_wait)
    # print("Actual pos:", pos)
    errorCode, angles = vrep.simxGetObjectQuaternion(clientID,joint_handles[6],joint_handles[0],vrep.simx_opmode_oneshot_wait)
    print("Actual qua:",angles)

    for i in range(7):
        theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)
        print("Theta",i,"is",theta)

    # sleep(10)
    input("Press any key to revert to origin...")
    print("")

    for i in range (7):
        vrep.simxSetJointTargetPosition(clientID,joint_handles[i],0,vrep.simx_opmode_oneshot_wait)
        print("Setting joint", i+1, "back to 0")

    sleep(0.5)

    vrep.simxRemoveObject(clientID,dummy_handle_0,vrep.simx_opmode_oneshot_wait)
    vrep.simxRemoveObject(clientID,dummy_handle_1,vrep.simx_opmode_oneshot_wait)
    vrep.simxRemoveObject(clientID,dummy_handle_3,vrep.simx_opmode_oneshot_wait)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection Failed")
print ('Program ended')
