import math

import numpy as np


"""
@package Control
The Control class includes all controlling methods. From building the matrices
to calculating the feedforward and feedbackward loop.
"""


class Control:
    def __init__(self, m, gravity, w_front, tire_coefficient,
                 tire_correction, tire_deg, sa, q1, q2, q3, q4, length, lf, lr, iz, vx, cf, cr):
        """!
        The Control constructor

        @param self: The object pointer

        @type m: Float
        @param m: The mass of the vehicle

        @type gravity: Float
        @param gravity: The gravity of the earth

        @type w_front: Float
        @param w_front: The weight on the front axle

        @type tire_coefficient: Float
        @param tire_coefficient: The coefficient of the tires

        @type tire_correction: Float
        @param tire_correction: The correction value of the tires

        @type tire_deg: Float
        @param tire_deg: The degression value of the tires

        @type sa: Float
        @param sa: The slip angle of the tires

        @type q1: Float
        @param q1: The lateral error gains value

        @type q2: Float
        @param q2: The heading error gains value

        @type q3: Float
        @param q3: The change of lateral error gains value

        @type q4: Float
        @param q4: The change of heading error gains value

        @type length: Float
        @param length: The length of the vehicle

        @type lf: Float
        @param lf: The tire stiffness factor for the front values

        @type lr: Float
        @param lr: The tire stiffness factor for the rear values

        @type iz: Float
        @param iz: The inertia of the vehicle

        @type vx: Float
        @param vx: The longitudinal velocity of the vehicle
        """

        """Weight"""
        self.m = m
        self.gravity = gravity
        self.w_front = w_front
        """Tire"""
        self.tire_coefficient = tire_coefficient
        self.tire_correction = tire_correction
        self.tire_deg = tire_deg
        self.sa = sa
        """Control"""
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4
        """Kinematics"""
        self.length = length
        self.lf = lf
        self.lr = lr
        self.iz = iz
        self.vx = vx
        self.cf = cf
        self.cr = cr
        self.kv = self.__calculateKV()

    def defineMatrixA(self):
        """!
        Calculates the A matrix of the control function X(t+1) = A * x + b

        @param self: The object pointer

        @rtype: np.ndarray
        @return: The A matric of the control function
        """

        a_11 = -(self.cf + self.cr) / (self.m * self.vx)

        a_21 = (self.cf + self.cr) / self.m
        a_31 = (self.lr * self.cr - self.lf * self.cf) / (self.m * self.vx)
        a_13 = (self.lr * self.cr - self.lf * self.cf) / (self.iz * self.vx)
        a_23 = (self.lf * self.cf - self.lr * self.cr) / self.iz
        a_33 = -(self.lf**2 * self.cf + self.lr **
                 2*self.cr) / (self.iz * self.vx)
        A = np.array([[0, 1, 0, 0], [0, a_11, a_21, a_31],
                      [0, 0, 0, 1], [0, a_13, a_23, a_33]])

        return A

    def defineMatrixB(self):
        """!
        Calculates the b vector of the control function X(t+1) = A * x + b

        @param self: The object pointer

        @rtype: np.ndarray
        @return: The b vector of the control function
        """

        b_01 = self.cf / self.m
        b_03 = (self.lf * self.cf) / self.iz

        B = np.array([[0], [b_01], [0], [b_03]])
        return B

    def defineMatrixQ(self):
        """!
        Defines the Q matrix of the Costfunction

        @param self: The object pointer

        @rtype: np.ndarray
        @return: The Costfunction matrix Q
        """
        Q = np.array([[self.q1, 0, 0, 0], [0, self.q2, 0, 0],
                      [0, 0, self.q3, 0], [0, 0, 0, self.q4]])
        return Q

    def __calculateKV(self):
        """!
        Calculates the understeer gradient Kv

        @param self: The object pointer

        @rtype: Float
        @return: The understeer gradient Kv
        """
        m_f = self.m * self.gravity * self.w_front
        m_r = self.m * self.gravity * (1 - self.w_front)
        kv = (m_f / (2*self.cf)) - (m_r / (2*self.cr))
        return kv

    def calculateFeedback(self, K, e_cg, e_cg_dot, heading_error, heading_error_dot):
        """!
        Calculate Feedback angle

        @param self: The object pointer

        @rtype: Float
        @return: The steering angle of the Feedback loop
        """
        delta_fb = K.item(0) * e_cg + K.item(1) * \
            e_cg_dot + K.item(2) * heading_error - \
            K.item(3) * heading_error_dot

        return delta_fb

    def calculateFeedforward(self, radius, K):
        """!
        Calculate Feedforward angle (ackermann) + the steady-state error

        @param self: The object pointer

        @rtype: Float
        @return: The steering angle of the Feedforward loop
        """

        delta_ff = (self.length / radius) + self.kv * \
            (self.vx**2 / radius) - K.item(2) * ((self.lr / radius) -
                                                 (self.lf * self.m * self.vx**2) /
                                                 (2 * self.cr * radius * self.length))

        return delta_ff
