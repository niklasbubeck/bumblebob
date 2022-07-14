#!/usr/bin/python2
from __future__ import division, print_function

import csv
import math
import os
import sys
import time
import timeit

import numpy as np
import rospy
from dynamic_reconfigure.server import Server
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32

from bumblebob_lateral_lq_controller.cfg import LqrConfig
from bumblebob_msgs.msg import PointArray
from control import Control
from helpers import Helpers
from least_square_circle import LeastSquareCircle
from lqr import LQR


"""
@package LateralLqrControllerNode
Calculates the steering angle for the vehicle, based on a dynamic vehicle
model and a linear quadratic control.
The input is given through the communication of ROS.
"""


class LateralLqrControllerNode:
    def __init__(self):
        """
        LateralLqrControllerNode Constructor
        @param self: The object pointer

        """

        self.steering_prev = 0
        self.maxsteering_rate = 80

        self.raceline = []
        self.vx = None
        self.vy = None
        self.vehicle_heading = None
        self.vehicle_position = None
        self.lastdelta = 0
        self.raceline_sub = None
        self.gazebo_sub = None
        self.steering_pub = None
        self.debug_mode = None
        self.write_csv = None

        self.q1 = None
        self.q2 = None
        self.q3 = None
        self.q4 = None
        self.R = None
        self.lookahead = None
        self.lookback = None

        """Read ROS Parameters"""
        self.readROSParams()

        """Init Subscribers and Publishers"""
        rospy.Subscriber(self.raceline_sub, PointArray, self.initRaceLine)
        rospy.Subscriber(self.gazebo_sub,
                         ModelStates, self.modelStatesCallback)
        self.pub_steering = rospy.Publisher(
            self.steering_pub, Float32, queue_size=1)

        """Wait for relevant callback data"""
        self.waitForSurroundings()

        self.lf = self.length * (1 - self.w_front)
        self.lr = self.length * self.w_front

        """Init csv file to write data to """
        if(self.write_csv):
            file = open(os.path.abspath(os.path.dirname(
                sys.argv[0]))
                + self.filepath, mode="w")
            self.writer = csv.writer(file, delimiter=",")

    def readROSParams(self):
        """
        Inits vehicle parameters
        @param self: The object pointer
        """
        self.cf = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/cf")
        self.cr = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/cr")
        self.tire_coefficient = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/tire_coefficient")
        self.tire_correction = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/tire_correction")
        self.tire_deg = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/tiredeg")
        self.sa = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/sa")
        self.w_front = rospy.get_param(
            "/lateral_lqr_controller_node/car/kinematics/w_front")
        self.m = rospy.get_param(
            "/lateral_lqr_controller_node/car/inertia/m")
        self.length = rospy.get_param(
            "/lateral_lqr_controller_node/car/kinematics/l")

        self.iz = rospy.get_param(
            "/lateral_lqr_controller_node/car/inertia/I_z")
        self.gravity = rospy.get_param(
            "/lateral_lqr_controller_node/car/inertia/g")
        self.maxsteering = rospy.get_param(
            "/lateral_lqr_controller_node/car/tire/max_steering")

        self.q1 = rospy.get_param(
            "/lateral_lqr_controller_node/control/Q/q1")
        self.q2 = rospy.get_param(
            "/lateral_lqr_controller_node/control/Q/q2")
        self.q3 = rospy.get_param(
            "/lateral_lqr_controller_node/control/Q/q3")
        self.q4 = rospy.get_param(
            "/lateral_lqr_controller_node/control/Q/q4")

        self.R = rospy.get_param(
            "/lateral_lqr_controller_node/control/R")

        self.lookahead = rospy.get_param(
            "/lateral_lqr_controller_node/control/lookahead")
        self.lookback = rospy.get_param(
            "/lateral_lqr_controller_node/control/lookback")

        self.raceline_sub = rospy.get_param(
            "/lateral_lqr_controller_node/subscribers/raceline_sub")
        self.gazebo_sub = rospy.get_param(
            "/lateral_lqr_controller_node/subscribers/gazebo_sub")

        self.steering_pub = rospy.get_param(
            "/lateral_lqr_controller_node/publishers/steering_pub")

        self.debug_mode = rospy.get_param(
            "/lateral_lqr_controller_node/inits/debug_mode")
        self.write_csv = rospy.get_param(
            "/lateral_lqr_controller_node/inits/write_csv")

        self.filepath = rospy.get_param(
            "/lateral_lqr_controller_node/file/path")

        rospy.loginfo("ROS Params Were Read")

    def initRaceLine(self, data):
        """!
        Callback for the /raceline topic that inits the raceline
        @param self: The object pointer
        @param data: The Pointarray msg
        """
        raceline = []
        for i in data.position:
            point = (- i.y, i.x)
            raceline.append(point)
        self.raceline = raceline

    def modelStatesCallback(self, data):
        """!
        Callback of the vehicle state
        @param self: The object pointer
        @param data: The ModelStates msg
        """

        self.vx = data.twist[data.name.index("byssa")].linear.x
        self.vy = data.twist[data.name.index("byssa")].linear.y

        self.vehicle_position = (- data.pose[data.name.index(
            "byssa")].position.y, data.pose[data.name.index("byssa")].position.x)

        # Euler conversion
        head_x = data.pose[data.name.index("byssa")].orientation.x
        head_y = data.pose[data.name.index("byssa")].orientation.y
        head_z = data.pose[data.name.index("byssa")].orientation.z
        head_w = data.pose[data.name.index("byssa")].orientation.w

        # yaw
        siny_cosp = 2 * (head_w * head_z + head_x * head_y)
        cosy_cosp = 1 - 2*(head_y * head_y + head_z * head_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vehicle_heading = -yaw

    def debug(self):
        """!
        Prints out the relevant data to debug
        @param self: The object pointer
        """

        print("----------------------------Debug Mode----------------------------")
        print("==Settings==")
        print("q1: " + str(self.q1) + "   q2: " + str(self.q2) + "   q3: " +
              str(self.q3) + "   q4: " + str(self.q4) + "  |  R: " + str(self.R))
        print("Lookahead: " + str(self.lookahead) +
              "   Lookback: " + str(self.lookback))
        print("")
        print("==Variables==")
        print("Vehicle Position: " + str(self.vehicle_position))
        print("Lateral Error: " + str(self.e_cg))

        print("Vehicle Heading: " + str(self.vehicle_heading))
        print("Road Heading: " + str(self.road_heading))
        print("Heading Error: " + str(self.heading_error))

        print("Radius: " + str(self.radius))

        print("\n \n \n")

    def updateLoop(self):
        """
        Updates the loop
        @param self: The object pointer
        """
        starttime = time.time()
        helpers = Helpers(self.raceline, self.vehicle_position,
                          self.vehicle_heading, self.lookback, self.lookahead,
                          self.vx, self.lastdelta, self.length)

        self.radius, center_x, center_y = helpers.calculateRadius()

        self.circle_direction, straight_line = helpers.calculateCircleDirection(
            center_x, center_y)

        self.e_cg = helpers.calculateEcg()

        self.heading_error, self.road_heading = helpers.calculateHeadingError(
            self.circle_direction, center_x, center_y)

        # Control

        control = Control(self.m, self.gravity, self.w_front, self.tire_coefficient,
                          self.tire_correction, self.tire_deg, self.sa, self.q1, self.q2,
                          self.q3, self.q4, self.length, self.lf, self.lr, self.iz, self.vx,
                          self.cf, self.cr)

        # Matrices
        A = control.defineMatrixA()
        B = control.defineMatrixB()
        Q = control.defineMatrixQ()

        lqr = LQR(A, B, Q, self.R)
        K, X = lqr.lqr()

        if(self.e_cg < 0):
            e_cg_dot = (self.vy + self.vx *
                        math.cos(self.heading_error)) / self.vx**2
        elif (self.e_cg > 0):
            e_cg_dot = -(self.vy + self.vx *
                         math.cos(self.heading_error)) / self.vx**2

        curvature = 1 / self.radius

        heading_error_dot = helpers.calculateHeadingErrorDot(
            self.heading_error, curvature)

        self.delta_fb = control.calculateFeedback(
            K, self.e_cg, e_cg_dot, self.heading_error, heading_error_dot)

        self.delta_ff = control.calculateFeedforward(self.radius, K)

        endtime = time.time()

        time_diff = endtime - starttime
        print("Time: " + str(time_diff))

        if self.debug_mode:
            self.debug()

    def publish(self):
        """
        Publishes the steering angle and writes the data to the csv
        @param self: The object pointer
        """

        if(self.circle_direction < 0):
            self.delta_ff *= -1

        steering_cmd = math.degrees(self.delta_fb + self.delta_ff)
        diff = abs(steering_cmd - self.steering_prev)
        rate = diff * 5
        newrate = rate
        # Constrain the max steering angle rate
        if(rate > self.maxsteering_rate and steering_cmd > self.steering_prev):
            steering_cmd = self.steering_prev + self.maxsteering_rate / 5
            newrate = self.maxsteering_rate / 5
        if(rate > self.maxsteering_rate and steering_cmd < self.steering_prev):
            steering_cmd = self.steering_prev - self.maxsteering_rate / 5
            newrate = self.maxsteering_rate / 5

        # Constrain the max steering angle
        self.steering_prev = steering_cmd
        if(steering_cmd > self.maxsteering):
            steering_cmd = self.maxsteering
        elif (steering_cmd < - self.maxsteering):
            steering_cmd = - self.maxsteering
        self.lastdelta = math.radians(steering_cmd)

        if(self.write_csv):
            self.writer.writerow([self.vehicle_position[0], self.vehicle_position[1],
                                  self.vehicle_heading, self.road_heading,
                                  self.delta_fb, self.delta_ff,
                                  steering_cmd, self.radius, self.e_cg,
                                  self.heading_error, self.vy, newrate])
        self.pub_steering.publish(steering_cmd)

    def waitForSurroundings(self):
        """
        Waits for the callbacks to be called and the corresponding
        values to be initialized
        @param self: The object pointer
        """
        rospy.loginfo("Ready And Waiting On Raceline To Be Sent...")
        while len(self.raceline) < 5:
            continue
        rospy.loginfo("Raceline Received")

        while self.vx < 0.1:
            continue

        if self.vx is None:
            rospy.spin()

        if self.vy is None:
            rospy.spin()

        if self.vehicle_heading is None:
            rospy.spin()

        if self.vehicle_position is None:
            rospy.spin()

        rospy.loginfo("All Gazebo States Got Loaded")

    def callback(self, config, level):
        """!
        The callback method for the dynamic reconfigure

        @param self: The object pointer
        @param config: The config of the dynamic reconfigure
        @param level: The level of the dynamic reconfigure
        """
        rospy.loginfo("New Values")
        self.q1 = config.q1
        self.q2 = config.q2
        self.q3 = config.q3
        self.q4 = config.q4
        self.R = config.R_matrix
        self.lookahead = config.look_front
        self.lookback = config.look_back
        return config


def main():
    """
    The main method for the LateralLQControllerNode
    """
    rospy.init_node('bumblebob_lqr_controller_node', anonymous=True)

    lqrNode = LateralLqrControllerNode()
    rospy.loginfo("LQ Control Node Started")
    rate = rospy.Rate(5)
    srv = Server(LqrConfig, lqrNode.callback)
    while not rospy.is_shutdown():

        lqrNode.updateLoop()
        lqrNode.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("ROSInterruptException was thrown")
        pass
