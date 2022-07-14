from __future__ import division, print_function

import numpy as np
import rospy
import scipy.linalg


"""@package LQR
Solves the Ricatti equation and solves the Gain Matrix K
"""


class LQR:

    def __init__(self, A, B, Q, R):
        """!
        Constructor of The LQR class
        @param self: The object pointer

        @type A: np.ndarray
        @param A: The A matrix of the state function

        @type B: np.ndarray
        @param B: The B matrix of the state function

        @type Q: np.ndarray
        @param Q: The cost matrix of the gains

        @type R: np.ndarray
        @param R: The cost matrix of the control
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

    def lqr(self):
        """!
        Solve the continuous time lqr controller.

        dx/dt = A x + B u

        cost = integral x.T*Q*x + u.T*R*u

        @param self: The object pointer

        @rtype: np.ndarray, np.ndarray
        @return: Gains matrix, result of the riccati equation
        """
        # ref Bertsekas, p.151

        # init var for readablity
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R

        # first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

        # compute the LQR gain
        K = (1 / R) * (B.T * X)

        return K, X

    def dlqr(self):
        """!
        Solve the discrete time lqr controller.


        x[k+1] = A x[k] + B u[k]

        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]

        @param self: The object pointer

        @rtype: np.ndarray, np.ndarray
        @return: Gains matrix, result of the riccati equation
        """
        # ref Bertsekas, p.151

        # init var for readablity
        A = self.A
        B = self.B
        Q = self.Q
        R = self.R

        # first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))

        # print(np.linalg.det(B.T*X*B+R))
        # compute the LQR gain
        K = (1 / R) * (B.T * X)
        # rest = B.T.dot(X).dot(A)
        # K = inverse.dot(rest)
        eigVals, eigVecs = scipy.linalg.eig(A-B*K)

        return np.array(K), X
