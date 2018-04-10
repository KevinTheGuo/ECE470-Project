# File which has all the collision-detection functions

# Self-collision detection, given a certain configuration
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm

import helper_library as helpme

# Defined radius for all collision detection. Units are meters
RADIUS = 0.015

# Calculate the distance between two coordinates in space
def calc_distance(p_1, p_2):
    distance_vect = np.zeros((3,1))
    distance_vect[0,0] = abs(p_1[0] - p_2[0])
    distance_vect[1,0] = abs(p_1[1] - p_2[1])
    distance_vect[2,0] = abs(p_1[2] - p_2[2])
    return norm(distance_vect)

# Check if a collision has occurred
# This is done by checking if the distance between two points in space is less than the sum of their radii
# Returns True if colliding, False if not
def check_collision(points1, r1, points2, r2):
    distance = calc_distance(points1, points2)
    return distance < r1 + r2

# Check Point Collision:
# For a given robot, environment, and current theta, check if there are collisions
# INPUT: S, p_robot, r_robot, p_obstacle, r_obstacle, theta. Names self-explanatory
# OUTPUT: False if no collisions detected, True if there are collisions detected
def check_point_collision(S, p_robot, r_robot, p_obstacle, r_obstacle, curr_theta):
    # Parse s into a friendlier version to our library
    S_eval = np.hsplit(S, len(S[0]))

    # Properties of our system
    NUM_ROBOT_SPHERES = len(p_robot[0])
    NUM_OBSTACLE_SPHERES = len(p_obstacle[0])

    # Get the current robot configuration
    curr_p_robot = np.zeros((3, NUM_ROBOT_SPHERES))
    for i in range(1, NUM_ROBOT_SPHERES):
        p = np.ones((4,1))
        p[0:3,0] = p_robot[0:3,i]
        curr_p_robot[0:3,i] = helpme.evalT(S_eval[0:i-1], curr_theta[0:i-1], p)[0:3,0]

    # Check for robot self-collision
    for i in range(NUM_ROBOT_SPHERES):
        for j in range(i+1, NUM_ROBOT_SPHERES):
            if check_collision(curr_p_robot[0:3,i:i+1], r_robot[0][i], curr_p_robot[0:3,j:j+1], r_robot[0][j]):
                print("Robot Spheres {} and {} collide".format(i,j))
                print("Their positions are {} and {}".format(curr_p_robot[0:3,i:i+1],curr_p_robot[0:3,j:j+1]))
                print("Their radii are {} and {}".format(r_robot[0][i],r_robot[0][j]))
                print("Their distance is {}".format(calc_distance(curr_p_robot[0:3,i:i+1],curr_p_robot[0:3,j:j+1])))
                return True

    # Check for robot collision with external obstacles
    for i in range(NUM_ROBOT_SPHERES):
        for j in range(NUM_OBSTACLE_SPHERES):
            if check_collision(curr_p_robot[0:3,i:i+1], r_robot[0][i], p_obstacle[0:3,j:j+1], r_obstacle[0][j]):
                print("Robot Sphere {} collides with obstacle {}".format(i,j))
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
