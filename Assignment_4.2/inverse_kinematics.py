# Kuka inverse kinematics
# All measurements are in meters and radians
import numpy as np
import scipy.linalg as sl

def skew_sym(arr):
    return np.array([[0,-arr[2],arr[1]],[arr[2],0,-arr[0]],[-arr[1],arr[0],0]])

def inv_skew_sym(arr):
    return np.array([[arr[2,1]],[arr[0,2]],[arr[1,0]]])

def matrix_rep(arr):
    matrix = np.zeros((4,4))
    matrix[0:3,0:3] = skew_sym(arr[0:3,0:1])
    matrix[0:3,3:4] = arr[3:6,0:1]
    return matrix

def inv_matrix_rep(arr):
    inv_matrix = np.zeros((6,1))
    inv_matrix[0:3,0:1] = inv_skew_sym(arr[0:3,0:3])
    inv_matrix[3:6,0:1] = arr[0:3,3:4]
    return inv_matrix

def adjoint(arr):
    adjoint = np.zeros((6,6))
    adjoint[0:3,0:3] = arr[0:3,0:3]
    adjoint[3:6,3:6] = arr[0:3,0:3]
    adjoint[3:6,0:3] = skew_sym(arr[0:3,3:4]) @ arr[0:3,0:3]
    return adjoint

def jacobian(S,theta,joint):
    if joint == 1:
        return S[0:6,0:1]
    return adjoint(jacobian_helper(S,theta,joint-1)) @ S[0:6,joint-1:joint]

def jacobian_helper(S,theta,joint):
    if (joint == 1):
        return sl.expm(matrix_rep(S[0:6,0:1])*theta[0])
    return jacobian_helper(S,theta,joint-1) @ sl.expm(matrix_rep(S[0:6,joint-1:joint])*theta[joint-1])

def prism_screw(a,q):
    S = np.zeros((6,1))
    S[3:6,0:1] = a
    return S

def rotat_screw(a,q):
    S = np.zeros((6,1))
    S[0:3,0:1] = a
    S[3:6,0:1] = -skew_sym(a) @ q
    return S

def inverse_kinematics(T_1in0, initial_guess=np.full((7,1),0), iterationMax=50):
    M = np.array([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 1.266], [0., 0., 0., 1.]])
    S = np.array([[0., 0., 0., 0., 0., 0., 0.], [0., 1., 0., -1., 0., 1., 0.], [1., 0., 1., 0., 1., 0., 1.], [0., -0.34, 0., 0.74, 0., -1.14, 0.], [0., 0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0., 0.]])

    print("Hi, and welcome to the Robo-Swagorithm 3001!\n")

    # Make first theta guess
    theta = np.full((7,1),0)

    print("Please wait... processing question:\n")

    # Variable to check how many iterations we're at so far
    iterations = 0

    while(True):
        # 1: Find current pose of tool given theta guess (frame 2)
        T_2in0 = (sl.expm(matrix_rep(S[0:6,0:1])*theta[0]) @
                  sl.expm(matrix_rep(S[0:6,1:2])*theta[1]) @
                  sl.expm(matrix_rep(S[0:6,2:3])*theta[2]) @
                  sl.expm(matrix_rep(S[0:6,3:4])*theta[3]) @
                  sl.expm(matrix_rep(S[0:6,4:5])*theta[4]) @
                  sl.expm(matrix_rep(S[0:6,5:6])*theta[5]) @
                  sl.expm(matrix_rep(S[0:6,6:7])*theta[6]) @
                  M)

        # 2: Find spatial twist to align frame 2 with frame 1 in one second
        V = inv_matrix_rep(sl.logm(T_1in0 @ sl.inv(T_2in0)))

        # 3: Find space Jacobian as function of current theta
        J = np.zeros((6,7))
        J[0:6,0:1] = jacobian(S,theta,1)
        J[0:6,1:2] = jacobian(S,theta,2)
        J[0:6,2:3] = jacobian(S,theta,3)
        J[0:6,3:4] = jacobian(S,theta,4)
        J[0:6,4:5] = jacobian(S,theta,5)
        J[0:6,5:6] = jacobian(S,theta,6)
        J[0:6,6:7] = jacobian(S,theta,7)

        # 4: Perform inverse velocity kinematics, find thetadot. But use our new algorithm this time!
        mu = 0.01
        thetadot = sl.inv(np.transpose(J)@J + mu*np.identity(7)) @ np.transpose(J) @ V

        # 5: Calculate new theta by applying our calculated thetadot for 1 second
        theta = theta + thetadot

        # 6: Repeat until we are below cutoff
        print("Distance is {}".format(np.linalg.norm(V)))
        if(np.linalg.norm(V) < 0.01):
            print("\nDing!... Your answer is ready!\n")
            print(repr(theta))
            return(theta)

        # 7: Check if we surpass our defined maximum number of iterations
        if (iterations > iterationMax):
            print("\nI'm sorry Tim, I'm afraid I can't do that.\n")
            return
        iterations += 1
