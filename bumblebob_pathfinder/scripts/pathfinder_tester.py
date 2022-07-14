#!/usr/bin/env python

'''
File: send_test_msgs.py
-----
Description: Sends cone array messages with random cone positions and types.
-----
Author: Karim Awad
Version: 1.0.0
'''

import csv
import os
import random
import sys

import rospy

from bumblebob_msgs.msg import Cone, ConeArray

topic = '/bumblebob/camera/cones'
publisher = rospy.Publisher(topic, ConeArray, queue_size=5)

rospy.init_node('test_cone_array')

# Update once per second
rate = rospy.Rate(1)

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
