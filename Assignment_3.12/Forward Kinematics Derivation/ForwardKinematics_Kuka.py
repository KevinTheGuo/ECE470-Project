import numpy as np
import scipy.linalg as syl

def skew_sym(arr):
    return np.array([[0,-arr[2],arr[1]],[arr[2],0,-arr[0]],[-arr[1],arr[0],0]])

def matrix_rep(arr):
    matrix = np.zeros((4,4))
    matrix[0:3,0:3] = skew_sym(arr[0:3,0:1])
    matrix[0:3,3:4] = arr[3:6,0:1]
    return matrix

def forwardKinematics(theta):
    M = np.array([[ 1.   ,  0.   ,  0.   ,  0.   ],
                  [ 0.   ,  1.   ,  0.   ,  0.   ],
                  [ 0.   ,  0.   ,  1.   ,  1.266],
                  [ 0.   ,  0.   ,  0.   ,  1.   ]])


    S = np.array([[ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ],
                  [ 0.  ,  1.  ,  0.  , -1.  ,  0.  ,  1.  ,  0.  ],
                  [ 1.  ,  0.  ,  1.  ,  0.  ,  1.  ,  0.  ,  1.  ],
                  [ 0.  , -0.34,  0.  ,  0.74,  0.  , -1.14,  0.  ],
                  [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ],
                  [ 0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ,  0.  ]])

    T = (syl.expm(matrix_rep(S[0:6,0:1])*theta[0]) @
         syl.expm(matrix_rep(S[0:6,1:2])*theta[1]) @
         syl.expm(matrix_rep(S[0:6,2:3])*theta[2]) @
         syl.expm(matrix_rep(S[0:6,3:4])*theta[3]) @
         syl.expm(matrix_rep(S[0:6,4:5])*theta[4]) @
         syl.expm(matrix_rep(S[0:6,5:6])*theta[5]) @
         syl.expm(matrix_rep(S[0:6,6:7])*theta[6]) @ M)

    return(T)

theta = np.array([0, 1.57, 0, 1.57, 0, 1.57, 0])
print(np.round(forwardKinematics(theta), decimals=5))
