import math

import numpy as np
import rospy

from least_square_circle import LeastSquareCircle


"""
@package Helpers
The Helpers class includes helper methods that calculate e.g. errors
"""


class Helpers:
    def __init__(self, raceline, vehicle_position, vehicle_heading,
                 lookback, lookahead, vx, lastdelta, length):
        """!
        The Helpers constructor

        @param self: The object pointer

        @type raceline: List
        @param raceline: The line to follow

        @type vehicle_position: Tuple
        @param vehicle_position: The position of the vehicle

        @type vehicle_heading: Float
        @param vehicle_heading: The heading of the vehicle

        @type lookback: Int
        @param lookback: A value to define the area for calculations

        @type lookahead: Int
        @param lookahead: A value to define the area for calculations

        @type vx: Float
        @param vx: The longitudinal velocity of the vehicle

        @type lastdelta: Float
        @param lastdelta: The steering angle of the previous iteration

        @type length: Float
        @param length: The length of the vehicle

        """
        self.raceline = raceline
        self.vehicle_position = vehicle_position
        self.vehicle_heading = vehicle_heading

        self.lookahead = lookahead
        self.lookback = lookback
        self.length = length
        self.vx = vx
        self.lastdelta = lastdelta

        self.index, self.shortest_distance = self.__calculateIndex()

    def __calculateIndex(self):
        """!
        Calculates the index of the nearest point between raceline and vehicle

        @param self: The object pointer

        @rtype: Float, Float
        @return: The index and the shortest distance
        """

        shortest_distance = np.finfo(float).max
        counter = 0
        index = 0
        for point in self.raceline:
            counter += 1
            distance = math.sqrt(
                (point[0] - self.vehicle_position[0])**2 + (point[1] -
                                                            self.vehicle_position[1])**2)
            if(distance < shortest_distance):
                shortest_distance = distance
                index = counter

        return index, shortest_distance

    def calculateEcg(self):
        """!
        Calculates the lateral error as the nearest Point between raceline and vehicle

        @param self: The object pointer

        @rtype: Float
        @return: The shortest distance
        """
        shortest_distance = self.shortest_distance
        relativeX = self.raceline[self.index][0] - self.vehicle_position[0]
        relativeY = self.raceline[self.index][1] - self.vehicle_position[1]
        rotatedX = math.cos(self.vehicle_heading) * relativeX - \
            math.sin(self.vehicle_heading) * relativeY
        rotatedY = math.cos(self.vehicle_heading) * relativeY + \
            math.sin(self.vehicle_heading) * relativeX

        if(rotatedX < 0):
            shortest_distance *= -1

        return shortest_distance

    def calculateRadius(self):
        """!
        Calculates the Center of the circle and its radius, that is defined
        by the points that are the closest to the vehicle position

        @param self: The object pointer

        @rtype: Float, Float, Float
        @return: The Radius and the Center of the circle in x and y
        """

        x = []
        y = []
        for i in range(self.index - self.lookback, self.index + self.lookahead):
            x.append(self.raceline[i % len(self.raceline)][0])
            y.append(self.raceline[i % len(self.raceline)][1])
        lsc = LeastSquareCircle()
        curvature, center_x, center_y = lsc.fit(x, y)

        return 1 / curvature, center_x, center_y

    def calculateCircleDirection(self, center_x, center_y):
        """!
        Calculate the direction of the path

        @param self: The object pointer

        @type center_x: Float
        @param center_x: The x value of the circles center point

        @type center_y: Float
        @param center_y: The y value of the circles center point

        @rtype: Float, Boolean
        @return: The direction of the circle, and if its straight
        """

        relativeX = center_x - self.vehicle_position[0]
        relativeY = center_y - self.vehicle_position[1]
        rotatedX = math.cos(self.vehicle_heading) * relativeX - \
            math.sin(self.vehicle_heading) * relativeY
        rotatedY = math.cos(self.vehicle_heading) * relativeY + \
            math.sin(self.vehicle_heading) * relativeX

        if(rotatedX < 0):
            circle_direction = -1
            straight_line = False

        elif(rotatedX > 0):
            circle_direction = 1
            straight_line = False

        else:
            straight_line = True

        return circle_direction, straight_line

    def calculateHeadingError(self, circle_direction, center_x, center_y):
        """!
        Calculate the difference between the road heading and the vehicle heading

        @param self: The object pointer

        @type circle_direction: Float
        @param circle_direction: The direction of the circle

        @type center_x: Float
        @param center_x: The x value of the circles center point

        @type center_y: Float
        @param center_y: The y value of the circles center point

        @rtype: Float, Float
        @return The error of the heading and the road heading
        """

        circle_vector = (
            self.vehicle_position[0] - center_x, self.vehicle_position[1] - center_y)

        if(circle_direction == 1):
            road_vector = (- circle_vector[0], circle_vector[1])
        elif(circle_direction == -1):
            road_vector = (circle_vector[0], - circle_vector[1])

        road_heading = math.atan2(road_vector[1], road_vector[0])
        v_heading = math.fmod(self.vehicle_heading, math.pi * 2)

        heading_error = road_heading - self.vehicle_heading

        if(heading_error > math.pi):
            heading_error -= 2 * math.pi
        elif (heading_error < - math.pi):
            heading_error += 2 * math.pi

        if(abs(heading_error) > math.pi / 2):
            rospy.logerr("Mistakes Were Made: Heading too far off!")
            heading_error = 0

        return heading_error, road_heading

    def calculateHeadingErrorDot(self, heading_error, curvature):
        """!
        Calculates the change of the heading error

        @param self: The object pointer

        @type heading_error: Float
        @param heading_error: The difference between road and vehicle heading

        @rtype: Float
        @return: The error of the change of heading
        """
        e_ra_dot = self.vx * math.sin(heading_error)
        acker = math.tan(self.lastdelta) / self.length

        heading_error_dot = (
            acker - (curvature * math.cos(heading_error) / (1 - e_ra_dot*curvature))) * self.vx
        return heading_error_dot
