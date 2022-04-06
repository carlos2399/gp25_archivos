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

from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray

def my_publisher():
   
    rospy.init_node('my_publisher_control')
    control_publisher = rospy.Publisher('/joint_states', JointState,queue_size=10)

    while not rospy.is_shutdown():
	msg = JointState()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''
	msg.name = ['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t']
	#msg.position = [pi/2, pi/6, 9.50609628111124e-05, 1.5707001586970428, -2.1052583260461685e-05, 1.0471617598412262]
	msg.position = [pi, 0, pi/2, 1.5707001586970428, -2.1052583260461685e-05, 1.0471617598412262]
	msg.velocity = []
	msg.effort = []

	time.sleep(1)

    	control_publisher.publish(msg)

	rospy.loginfo("PUBLISIHNG DATA")

def main():
	print("hello")
	respuesta = int(input("Dígame su respuesta: "))
	
    	if respuesta == 1:
        	my_publisher()
 	else:
		print("ROBOT ACTIVO")
		#result=commands.getoutput('/home/ros/catkin_ws/src/motoman/motoman_gp25_config/scripts/move_to_joint.py')
		os.system ("/home/ros/catkin_ws/src/motoman/motoman_gp25_config/scripts/move_to_joint.py")

if __name__ == '__main__':
	main()    	
	
