#!/usr/bin/python2
from __future__ import division, print_function

import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from bumblebob_msgs.msg import Cone, ConeArray, PointArray


class LaneVisualizerNode:
    def __init__(self):

        # publishes a visualization of the detected cones
        self.raceline_publisher = rospy.Publisher("/bumblebob/raceline_visualization",
                                                  Marker, queue_size=1)

        self.cone_restriction_blue_publisher = rospy.Publisher(
            "/bumblebob/cone_restriction_blue_visualization", Marker, queue_size=1)
        self.cone_restriction_yellow_publisher = rospy.Publisher(
            "/bumblebob/cone_restriction_yellow_visualization", Marker, queue_size=1)

        # cones_topic = rospy.get_param(CONE_TOPIC_PARAM, CONE_TOPIC_DEFAULT)
        rospy.Subscriber("/bumblebob/raceline", PointArray,
                         self.raceline_callback)

        rospy.Subscriber("/bumblebob/cone_restriction_blue",
                         PointArray, self.cone_restriction_blue_callback)

        rospy.Subscriber("/bumblebob/cone_restriction_yellow",
                         PointArray, self.cone_restriction_yellow_callback)

    # def feature_callback(self, msg):
    #     assert isinstance(msg, ScanFeatures)
    #     data = DataSet.from_scan_features_message(msg)
    #     if data.positions is not None:
    #         self.publish_objects(msg.point_cloud_seq, data.positions)

    def raceline_callback(self, msg):
        self.publish_line(msg.position, self.raceline_publisher, 1.0, 0.0, 0.0)

    def cone_restriction_blue_callback(self, msg):
        self.publish_line(
            msg.position, self.cone_restriction_blue_publisher, 0.0, 0.0, 1.0)

    def cone_restriction_yellow_callback(self, msg):
        self.publish_line(
            msg.position, self.cone_restriction_yellow_publisher, 1.0, 1.0, 0.0)

    def publish_line(self, points, publisher, red, green, blue):
        print(points)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # marker color
        marker.color.a = 1.0
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points = []

        for point in points:
            temp = Point()
            temp.x = point.x
            temp.y = point.y
            marker.points.append(temp)

        # Publish the Marker
        publisher.publish(marker)


if __name__ == '__main__':
    rospy.init_node('lane_detector', anonymous=True)
    LaneVisualizerNode()
    rospy.spin()
