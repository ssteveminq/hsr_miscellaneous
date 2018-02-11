#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8
import std_msgs.msg
import rospy
import math

topic = 'human_target'
topic_array = 'human_boxes'
publisher = rospy.Publisher(topic, Marker,queue_size=10)
array_publisher = rospy.Publisher(topic_array, MarkerArray,queue_size=10)
pub = rospy.Publisher('detection/num_human', Int8, queue_size=10)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():


   int_msg=Int8()
   int_msg.data=1
   pub.publish(int_msg)
   rospy.sleep(0.4)
