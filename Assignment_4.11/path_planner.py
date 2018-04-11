# File which has the path-planning functions, making heavy use of collision_detection.py
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm
import collision_detection

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
# OUTPUT: final_path- if valid path found, array of intermediate theta positions for each joint.
#                     if valid path not found, return False
def plan_my_path(p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal, max_iterations):
    # The Kuka robot's predefined S
    S = np.array([[0., 0., 0., 0., 0., 0., 0.], [0., 1., 0., -1., 0., 1., 0.], [1., 0., 1., 0., 1., 0., 1.], [0., -0.34, 0., 0.74, 0., -1.14, 0.], [0., 0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0., 0.]])

    # p_robot = np.array([[ 0,  0,  0,  0,  0,  0], [ 0, -2, -2,  2,  2,  4], [ 0,  0, -2, -2, -4, -4]])
    # r_robot = np.array([[0.90, 0.90, 0.90, 0.90, 0.90, 0.90]])
    # p_obstacle = np.array([[4.56, 2.59, -0.97, -0.67, 3.34, -0.59, 4.25, 3.28, 2.41, 4.38, -0.38,  0.11, 3.61, 4.95, 2.23, -1.51, 3.44, 0.85, 2.14, -1.99, -1.58, -1.66,  3.82], [0.28, -1.86, 4.78, 2.83, 0.18, 4.79, 1.55, 2.73, 0.03, 2.86, 1.92, 1.65,  -4.42, 2.70, 3.44, 1.72, -4.08, -4.97, -0.44, 1.42, 3.45, 2.85, -0.20], [5.00, -4.53, 1.55, 1.05, -4.33, -3.10, -3.04, 4.27, 4.47, 3.68, 3.82,  -2.95, 4.49, 3.60, 3.59, -3.75, 4.42, 1.93, 1.56, 2.73, 3.08, -4.56,  -1.01]])
    # r_obstacle = np.array([[4.38, 1.07, 2.52, 1.78, 1.63, 0.60, 0.58, 3.54, 0.89, 2.32, 2.58, 2.03,  0.83, 2.62, 1.78, 2.14, 2.65, 2.19, 1.07, 2.45, 3.82, 1.67, 1.94]])
    # theta_start = np.array([[-1.34], [-0.27], [3.07], [1.51]])
    # theta_goal = np.array([[2.50], [2.01], [-1.04], [0.92]])

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

    # Start running our algorithm to test random points in config space
    while(True):
        # 0. Increment our iterations
        iterations += 1
        print(iterations)
        if (iterations > max_iterations):
            print("ERROR: Exceeded max iterations")
            return False

        # 1. Create a random point in configuration-space, and see if it results in immediate collision
        curr_theta = np.random.uniform(low=-3.14, high=3.14, size=theta_start.shape)
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
            # print("appended to goal tree: \n{}".format(repr(curr_theta)))

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

    print("Your final path is \n{}".format(repr(final_path)))
    return final_path
