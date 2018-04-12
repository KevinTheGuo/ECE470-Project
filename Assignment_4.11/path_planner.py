# File which has the path-planning functions, making heavy use of collision_detection.py
import vrep
import math
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm
import collision_detection
import forward_kinematics


# Create a class to help us implement trees
class Tree_Node:
    def __init__(self, value, parent_index):
        self.value = value
        self.parent_index = parent_index

# Plan My Path
# For a given robot, environment, desired theta, and current theta, return a set of
# thetas which defines a valid path to the desired theta goal. If it cannot find a satisfactory
# path in the given maximum number of iterations, it will just return -1
# INPUT: p_robot- current position of each joint of robot
#        r_robot- radius of each joint of robot
#        p_obstacle- current position of each obstacle
#        r_obstacle- radius of each obstacle
#        theta_start- current theta of each joint of robot
#        theta_goal- desired theta of each joint of robot
#        clientID- needed to interact with VREP
#        base_joint_handle- base reference needed to display dummies properly
# OUTPUT: final_path- if valid path found, array of intermediate theta positions for each joint.
#                     if valid path not found, return False
#         dummy_handle_list- this is really sketchy. because we want our generated dummy handles to stay
#                            while the robot is moving, we're passing this back to the vrep_main.py instead of
#                            deleting the dummies ourselves, and relying on vrep_main to clean up our dummies.
def plan_my_path(p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal, max_iterations, clientID, base_joint_handle):
    # The Kuka robot's predefined S
    S = np.array([[0., 0., 0., 0., 0., 0., 0.], [0., 1., 0., -1., 0., 1., 0.], [1., 0., 1., 0., 1., 0., 1.], [0., -0.34, 0., 0.74, 0., -1.14, 0.], [0., 0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0., 0.]])

    # Properties of our system
    ROBOT_NUM_JOINTS = len(theta_start)

    # Create lists which hold points in configuration-space part of the start and goal trees.
    # The points are represented as objects of 'Theta_Branch' class, which hold their parents' index in the list, and their own theta value
    start_tree = [Tree_Node(theta_start,-1)]
    goal_tree = [Tree_Node(theta_goal,-1)]

    # Create a variable which keeps track of iterations
    iterations = 0

    # Create a list which represents our (hopefully) found path from start to goal theta!
    final_path = np.zeros((ROBOT_NUM_JOINTS,1))

    # Initialize a list which holds all the dummy handles we've created
    dummy_handle_list = []

    # Start running our algorithm to test random points in config space
    while(True):
        # 0. Increment our iterations
        iterations += 1
        print(iterations)
        if (iterations > max_iterations):
            print("ERROR: Exceeded max iterations")
            return False

        # 1. Create a random point in configuration-space, and see if it results in immediate collision
        #curr_theta = np.random.uniform(low=-3.14, high=3.14, size=theta_start.shape)
        curr_theta = np.zeros(theta_start.shape)
        curr_theta[0,0] = np.random.uniform(low=-2.96, high=2.96)
        curr_theta[1,0] = np.random.uniform(low=-2.09, high=2.09)
        curr_theta[2,0] = np.random.uniform(low=-2.96, high=2.96)
        curr_theta[3,0] = np.random.uniform(low=-2.09, high=2.09)
        curr_theta[4,0] = np.random.uniform(low=-2.96, high=2.96)
        curr_theta[5,0] = np.random.uniform(low=-2.09, high=2.09)
        curr_theta[6,0] = np.random.uniform(low=-3.05, high=3.05)


        if (collision_detection.check_point_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta)):
            continue   # If there is an immediate collision, continue to next iteration

        # 2. Iterate through the start-theta tree, find the point in it closest to our random point
        shortest_dist = float('inf')
        closest_index = None
        for index in range(len(start_tree)):
            curr_dist = norm(start_tree[index].value - curr_theta)
            if curr_dist < shortest_dist:
                shortest_dist = curr_dist
                closest_index = index

        # 3. If a collision-free direct path exists from the start-theta tree's closest point to our random point,
        #    add our point to the start-theta tree
        if (collision_detection.check_path_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta, start_tree[closest_index].value) == 0):
            start_tree.append(Tree_Node(curr_theta, closest_index))
            print("appended to start tree: \n{} ".format(repr(curr_theta)))
            dummy_handle_list.append(add_dummy(clientID, 0.1, [255,0,0], curr_theta, base_joint_handle))

        # 4. See if we can connect this same randomly-generated point to our goal-theta tree using steps 2 and 3 above.
        shortest_dist = float('inf')
        closest_index = None
        for index in range(len(goal_tree)):
            curr_dist = norm(goal_tree[index].value - curr_theta)
            if curr_dist < shortest_dist:
                shortest_dist = curr_dist
                closest_index = index
        if (collision_detection.check_path_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta, goal_tree[closest_index].value) == 0):
            goal_tree.append(Tree_Node(curr_theta, closest_index))
            print("appended to goal tree: \n{}".format(repr(curr_theta)))
            dummy_handle_list.append(add_dummy(clientID, 0.1, [0,255,0], curr_theta, base_joint_handle))

        # 5. We've found a viable path if we added the same node to both trees!
        if (start_tree[-1].value.all == goal_tree[-1].value.all):
            print("Found viable path!")

            # First, add the central node
            final_path[0:ROBOT_NUM_JOINTS,0:1] = start_tree[-1].value

            # Next, add all the nodes in the start_tree on the path back to the start_theta
            curr_node = start_tree[-1]
            while(curr_node.parent_index != -1):  # break when parent_index is -1, which happens only for the start_theta root
                curr_node = start_tree[curr_node.parent_index]
                final_path = np.concatenate((curr_node.value, final_path), axis=1)    # insert in front of final_path

            # Finally, add all the nodes in the goal_tree on the path back to the goal_theta
            curr_node = goal_tree[-1]
            while(curr_node.parent_index != -1):  # break when parent_index is -1, which happens only for the goal_theta root
                curr_node = goal_tree[curr_node.parent_index]
                final_path = np.concatenate((final_path, curr_node.value), axis=1)  # append on back of final_path

            # And now we should have our final path! Let's break.
            break

    # print("Your final path is \n{}".format(repr(final_path)))
    return final_path, dummy_handle_list


# add_dummy
# A helper function for the path-planner to visualize points it's added to its trees
# INPUT:   clientID- used to interface with VREP
#          size- desired size of the dummy
#          color- desired color of the dummy
#          theta- calculates position of the dummy using forward kinematics for the robot
# OUTPUT:  dummy_handle- the handle of the dummy we just added

def add_dummy(clientID, size, color, theta, base_joint_handle):
    S = np.array([[0., 0., 0., 0., 0., 0., 0.], [0., 1., 0., -1., 0., 1., 0.], [1., 0., 1., 0., 1., 0., 1.], [0., -0.34, 0., 0.74, 0., -1.14, 0.], [0., 0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0., 0.]])

    errorCode, dummy_handle = vrep.simxCreateDummy(clientID, 0.1, color, vrep.simx_opmode_oneshot_wait)
    end_position = forward_kinematics.forwardKinematics(theta)[0:3,3:4]
    vrep.simxSetObjectPosition(clientID, dummy_handle, base_joint_handle, end_position, vrep.simx_opmode_oneshot_wait)

    return dummy_handle

# Does your robot GOT THEM SMOOTH MOVES? No? Well, say goodbye to your clunky awkward
# robots, 'cause this function is gonna GIMME THEM SMOOTH MOVES.
def gimme_them_smooth_moves(clientID, joint_handles, theta_goal):
    SMOOTHNESS_LEVEL = 15    # how SMOOTH are YOU?

    theta_start = np.zeros((7,1))
    for i in range(7):
        errorCode, jointPos = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_oneshot_wait)
        theta_start[i] = jointPos

    print("theta start is {}".format(theta_start))
    print("theta goal is {}".format(theta_goal))

    samples = 1 + math.ceil(collision_detection.calc_distance(theta_start, theta_goal)*SMOOTHNESS_LEVEL)
    for step in np.arange(0, 1, 1/samples):
        curr_theta = (1-step)*theta_start + step*theta_goal
        for i in range(7):                     # Iterate through every joint on our robot
            vrep.simxSetJointPosition(clientID, joint_handles[i], curr_theta[i,0], vrep.simx_opmode_oneshot_wait)

    for i in range(7):
        vrep.simxSetJointPosition(clientID, joint_handles[i], theta_goal[i,0], vrep.simx_opmode_oneshot_wait)
