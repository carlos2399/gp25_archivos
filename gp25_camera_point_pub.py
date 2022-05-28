#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import commands
import os

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

from std_msgs.msg import String,Float64MultiArray
from geometry_msgs.msg import PoseStamped



def camera_control():
   
    rospy.init_node('camera_point_control')
    control_publisher = rospy.Publisher('/visp_auto_tracker/object_position', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
	msg = PoseStamped()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = '/map'
	msg.pose.position = [0,0,0]
	msg.pose.orientation = [0,0,0,0]
	time.sleep(1)

    	control_publisher.publish(msg)

	rospy.loginfo("PUBLISIHNG DATA")

def main():
	print("hello")
	

if __name__ == '__main__':
	camera_control()    	
	




