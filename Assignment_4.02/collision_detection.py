# Self-collision detection, given a certain configuration
import numpy as np
from numpy.linalg import inv, norm
from scipy.linalg import expm, logm

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

# Simplified wrapper for the other collision-detector, using defined radius above
# INPUT: p1 - position of first object to check collision
#        p2 - position of second object to check collision
# OUTPUT: boolean, True if objects are in collision, False if objects not in collision
def collision_wrapper(p1, p2):
    return check_collision(p1, RADIUS, p2, RADIUS)
