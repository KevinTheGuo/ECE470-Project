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
import forward_kinematics
import inverse_kinematics
import quaternion
import collision_detection
import path_planner


print ('KUKA MOVER started')
vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID != -1:
    print ('Connected to KUKA Simulator')

    # Loop through all the robot's joints and get unique ID's for each
    joint_handles = []
    for i in range(1, 8):
        joint_name = 'LBR_iiwa_7_R800_joint' + str(i)
        errorCode, handle = vrep.simxGetObjectHandle(clientID, joint_name, vrep.simx_opmode_oneshot_wait)
        joint_handles.append(handle)

    # Get a handle for the movable inverse kinematics dummy
    errorCode, target_dummy_handle = vrep.simxGetObjectHandle(clientID, "Dummy", vrep.simx_opmode_oneshot_wait)

    robot_joint_bounding_handles = []
    for joint in range(7):
        # Make a Dummy for each joint
        if joint == 5:
            errorCode, bounding_handle = vrep.simxCreateDummy(clientID, 0.10, None, vrep.simx_opmode_oneshot_wait)
        elif joint == 6:
            errorCode, bounding_handle = vrep.simxCreateDummy(clientID, 0.08, None, vrep.simx_opmode_oneshot_wait)
        else:
            errorCode, bounding_handle = vrep.simxCreateDummy(clientID, 0.15, None, vrep.simx_opmode_oneshot_wait)
        robot_joint_bounding_handles.append(bounding_handle)
        vrep.simxSetObjectParent(clientID,robot_joint_bounding_handles[joint],joint_handles[joint],False,vrep.simx_opmode_oneshot_wait)

    # Get handles for all the movable collision detection dummies
    # Set number of obstacles
    NUM_OBSTACLES = 5

    dummy_obstacle_handles = []
    for i in range(1, NUM_OBSTACLES + 1):
        dummy_name = 'Dummy' + str(i)
        errorCode, handle = vrep.simxGetObjectHandle(clientID, dummy_name, vrep.simx_opmode_oneshot_wait)
        dummy_obstacle_handles.append(handle)

    # Create bounding volume dummies for the joints
    BOUNDING_VOL_RADIUS = 0.15
    TINY_RADIUS = 0.05

    # Initialize variable which let us check if the dummy has been moved
    prev_dummy_pos = None

    try:
        while True:
            # Wait for user to say they want to move
            input("Waiting for user to set point...")

            # Get the desired pose by checking the pose of the user-movable dummy
            errorCode, dummy_pos = vrep.simxGetObjectPosition(clientID, target_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
            #print("Dummy pos: {}".format(dummy_pos))
            errorCode, dummy_quaternion = vrep.simxGetObjectQuaternion(clientID, target_dummy_handle, -1, vrep.simx_opmode_oneshot_wait)
            #print("Dummy qua: {}".format(dummy_quaternion))

            # Make sure we don't recalculate if the inverse kinematics dummy is still in the same position
            if (dummy_pos == prev_dummy_pos):
                continue
            prev_dummy_pos = dummy_pos

            # Mush the quaternion and pos into a pose matrix!
            # Our function takes care of the V-REP quaternion mismatch
            pose = quaternion.matrix_from_quaternion(dummy_quaternion)  # This function fills in only [0:3,0:3] (orientation) so we have to fill in the last part ourselves
            pose[0,3] = dummy_pos[0]
            pose[1,3] = dummy_pos[1]
            pose[2,3] = dummy_pos[2]
            print("Desired Pose: \n{}".format(pose))

            # Get theta start (current thetas of robot joints)
            theta_start = np.zeros((7,1))
            for i in range(7):
                errorCode, jointPos = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)
                print("Joint ", i, " pos: ", jointPos)
                theta_start[i] = jointPos

            # Get theta goal
            theta_goal = inverse_kinematics.inverse_kinematics(pose)
            print("Theta goal: {}".format(theta_goal))

            # Get p_robot (position of each robot's collision spheres)
            p_robot = np.zeros((3,8))
            for i in range(7):
                errorCode, robotSpherePos = vrep.simxGetObjectPosition(clientID, joint_handles[i], -1, vrep.simx_opmode_oneshot_wait)
                p_robot[0:3, i+1] = robotSpherePos
                print("P_Robot ", i, " is: ", p_robot[:, i+1])

            # Get r_robot (radius of robot's joints)
            r_robot = np.zeros((1,8))
            r_robot.fill(0.08)
            r_robot[0,0] = 0.01
            r_robot[0,7] = 0.01

            # Get p_obstacle (position of external obstacles)
            print(joint_handles)
            print(dummy_obstacle_handles)
            p_obstacle = np.zeros((3,NUM_OBSTACLES))
            for i in range(0, NUM_OBSTACLES):
                p_obstacle[:,i] = vrep.simxGetObjectPosition(clientID, dummy_obstacle_handles[i], -1, vrep.simx_opmode_oneshot_wait)[1]

            # Get r_obstacle (radius of each external obstacle)
            r_obstacle = np.zeros((1,NUM_OBSTACLES))
            r_obstacle.fill(BOUNDING_VOL_RADIUS)

            # Plan a path!
            print("theta_start: \n{}".format(theta_start))
            print("theta_goal: \n{}".format(theta_goal))
            print("p_robot: \n{}".format(p_robot))
            print("r_robot: \n{}".format(r_robot))
            print("p_obstacle: \n{}".format(p_obstacle))
            print("r_obstacle: \n{}".format(r_obstacle))

            # TESTING STUFF
            S = np.array([[0., 0., 0., 0., 0., 0., 0.], [0., 1., 0., -1., 0., 1., 0.], [1., 0., 1., 0., 1., 0., 1.], [0., -0.34, 0., 0.74, 0., -1.14, 0.], [0., 0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0., 0.]])
            while(True):
                input("Swag")
                rand_theta = np.random.uniform(low=-3.14, high=3.14, size=theta_start.shape)
                print("Collision? {}".format(collision_detection.check_point_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, rand_theta)))
                print(rand_theta)
                print("Forward Kinematics: \n{}".format(forward_kinematics.forwardKinematics(rand_theta)))
                # Also maybe want to getjointposition and print out the actual thetas?
                for j in range(7):                     # Iterate through every joint on our robot
                    vrep.simxSetJointPosition(clientID, joint_handles[j], rand_theta[j], vrep.simx_opmode_oneshot_wait)

            # Make sure that we have a valid theta goal
            if (theta_goal is not None):
                max_iterations = 420
                final_path = path_planner.plan_my_path(p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal, max_iterations)

                # If we generated a valid path, iterate through it and move our robot!
                if final_path is not False:
                    # print(len(final_path[0]))
                    print("Moving robot to goal theta!")
                    for i in range(len(final_path[0])):   # Iterate through each point in our path
                        print("Waypoint {}".format(i))
                        for j in range(7):                     # Iterate through every joint on our robot
                            print(final_path[j,i])
                            vrep.simxSetJointPosition(clientID, joint_handles[j], final_path[j,i], vrep.simx_opmode_oneshot_wait)
                            sleep(0.025)
                else:
                    print("Viable path not found in {} iterations".format(max_iterations()))

    except KeyboardInterrupt:
        pass

    sleep(0.5)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
    sys.exit("Connection Failed")
print ('Program ended')
