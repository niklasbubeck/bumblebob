#!/usr/bin/env python

'''
File: pathfinder_tester.py
-----
Description: Sends test tracks from CSV files as cone array messages.
-----
Author: Christian Zoellner
Version: 1.0.0
'''

import rospy
import csv
import os
import sys
from bumblebob_msgs.msg import Cone, ConeArray

topic = '/bumblebob/camera/cones'
publisher = rospy.Publisher(topic, ConeArray, queue_size=1)

rospy.init_node('test_cone_array')

hz = 60
rate = rospy.Rate(hz)

# tracks = os.listdir("/TestTracks")
# track_index = 0

print("Rate = " + (str)(hz))
rospy.loginfo("loginfo")


while not rospy.is_shutdown():

    cone_array = ConeArray()
    cone_array.header.frame_id = "/base_link"
    cone_array.header.stamp = rospy.Time.now()
    file = open(os.path.abspath(os.path.dirname(
        sys.argv[0])) + "/TestTracks/simple.csv", "r")
    csv_reader = csv.reader(file, delimiter=";")

    for row in csv_reader:
        cone1 = Cone()

        cone1.position.x = float(row[0])
        cone1.position.y = float(row[1])
        cone1.type = int(row[2])

        cone_array.cones.append(cone1)

    publisher.publish(cone_array)
    rate.sleep()
