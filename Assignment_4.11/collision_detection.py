# File which has all the collision-detection functions

# Self-collision detection, given a certain configuration
import numpy as np
import math
from scipy.linalg import expm, logm

import helper_library as helpme

# Defined radius for all collision detection. Units are meters
RADIUS = 0.015


# Calculate the distance between two coordinates in space
def calc_distance(coords1, coords2):
    coords1 = coords1.flatten()
    coords2 = coords2.flatten()
    x = (coords1[0] - coords2[0]) ** 2
    y = (coords1[1] - coords2[1]) ** 2
    z = (coords1[2] - coords2[2]) ** 2

    return math.sqrt(x + y + z)


# Check if a collision has occurred
# This is done by checking if the distance between two points in space is less than the sum of their radii
def check_collision(points1, r1, points2, r2):
    distance = calc_distance(points1, points2)
    return distance < r1 + r2


# Gets the base position from a homogenous transformation matrix
def get_base(M):
    return M[:3, 3].reshape(3)


# Takes in a slice from an S matrix and a theta matrix
# Return the result of the expm operation
def get_expm(s_slice, theta_slice):
    s_slice_bkt = helpme.bracket(s_slice.reshape(6, 1))
    return expm(s_slice_bkt * theta_slice)


# Appends a 1 to each column in a point array
def append_1_to_elements(point_array):
    array_of_ones = np.ones((1, point_array.shape[1]))
    return np.row_stack([point_array, array_of_ones])


    # Take in an S matrix, a theta matrix, and a matrix of points in their zero configuration
    # Return the positions of these points after a transform
    # NOTE: The 1st and 2nd points (the base and 1st "sphere") don't move!
def get_new_points(S, theta, points_initial):
    points_initial = append_1_to_elements(points_initial)
    operational_points = points_initial[:, 2:]
    _, num_points = operational_points.shape
    points_after_move = []
    expm_result = []

    # Get first expm, skip the first two points because they won't move
    expm_result.append(get_expm(S[:, 0], theta[0]))

    # Get the rest
    for i in range(1, num_points):
        expm_result.append(get_expm(S[:, i], theta[i]) @ expm_result[i - 1])

    # Calculate the new points
    for i in range(num_points):
        points_after_move.append(expm_result[i] @ points_initial[:, i + 2])

    # Add back unchanged points
    points_after_move = np.column_stack([points_initial[:, :2]] + points_after_move)

    return points_after_move[:-1, :]


# Check Point Collision:
# For a given robot, environment, and current theta, check if there are collisions
# INPUT: S, p_robot, r_robot, p_obstacle, r_obstacle, theta. Names self-explanatory
# OUTPUT: False if no collisions detected, True if there are collisions detected
def check_point_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta):
    curr_p_robot = get_new_points(S, curr_theta, p_robot)

    # Properties of our system
    NUM_ROBOT_SPHERES = len(p_robot[0])
    # print("NUM ROBOT SPHERES IS {}".format(NUM_ROBOT_SPHERES))
    # print("LEN S IS {}".format(len(S[0])))
    NUM_OBSTACLE_SPHERES = len(p_obstacle[0])

    # Check for robot self-collision
    for i in range(NUM_ROBOT_SPHERES):
        for j in range(i+1, NUM_ROBOT_SPHERES):
            if check_collision(curr_p_robot[0:3,i:i+1], r_robot[0][i], curr_p_robot[0:3,j:j+1], r_robot[0][j]):
                # print("Robot Spheres {} and {} collide".format(i,j))
                # print("Their positions are \n{} and \n{}".format(curr_p_robot[0:3,i:i+1],curr_p_robot[0:3,j:j+1]))
                # print("Their radii are {} and {}".format(r_robot[0][i],r_robot[0][j]))
                # print("Their distance is {}".format(calc_distance(curr_p_robot[0:3,i:i+1],curr_p_robot[0:3,j:j+1])))
                return True

    # Check for robot collision with external obstacles
    for i in range(NUM_ROBOT_SPHERES):
        for j in range(NUM_OBSTACLE_SPHERES):
            if check_collision(curr_p_robot[0:3,i:i+1], r_robot[0][i], p_obstacle[0:3,j:j+1], r_obstacle[0][j]):
                print("Robot Sphere {} collides with obstacle {}".format(i,j))
                print("Their positions are \n{} and \n{}".format(curr_p_robot[0:3,i:i+1],p_obstacle[0:3,j:j+1]))
                print("Their radii are {} and {}".format(r_robot[0][i],r_obstacle[0][j]))
                print("Their distance is {}".format(calc_distance(curr_p_robot[0:3,i:i+1],p_obstacle[0:3,j:j+1])))
                return True

    # Return false otherwise
    return False


# Check Path Collision:
# For a given robot, environment, and desired path, check if the path is collision-free
# INPUT: S, p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal. Names self-explanatory
# OUTPUT: step- the step at which the robot first runs into collision. 0 if no collision detected
def check_path_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal):
    # Properties of our system
    NUM_PATH_CHECKS = 20    # how many samples we will take along our theta path. defined by user

    # Get each new position of the robot along our path
    for step in np.arange(0, 1, 1/NUM_PATH_CHECKS):
        # Grab the current theta for our configuration
        curr_theta = (1-step)*theta_start + step*theta_goal

        if (check_point_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta)):
            return step   # Return the step at which we detected collision

    # If no collision detected, return 0
    return 0
