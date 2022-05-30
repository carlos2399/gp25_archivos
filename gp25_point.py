#!/usr/bin/env python

import rospy
import numpy as np


try:
	from math import pi, tau, dist, fabs, cos
except: 
	from math import pi, fabs, cos, sqrt

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Float64MultiArray

def callback(data):
	rospy.loginfo("POSITION: %s",data.pose.position)
	rospy.loginfo("ORIENTATION: %s",data.pose.orientation)

def listener():

	rospy.init_node('joint_states_listener')

	control_subscriber=rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)

	rospy.spin()

if __name__ == '__main__':
    	listener()
    
