#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import commands
import os
import sys 

try:
	from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
	from math import pi, fabs, cos, sqrt

from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray, Int8

def callback(data):
	rospy.loginfo("POSITION: %s",data.data)
	print(data.data)
	   
	if data.data == 1:
		
		print("+++++++ROBOT ESPERANDO LEER QR")
		print(data.data)
	    	
	elif data.data == 3:

		print("+++++++POSICION LEIDA, ESPERANDO MOVIMIENTO")
		print(data.data)
		os.system ("/home/carlos/catkin_ws/src/motoman/motoman_gp25_config_2/scripts/gp25.py")
		#sys.exit("POSICION ALCANZADA") 
		os._exit(os.EX_OK) 

	elif data.data == 4:

		print("+++++++LEYENDO POSICION")
		print(data.data)
	
def listener():

	rospy.init_node('joint_states_listener')

	status_subscriber=rospy.Subscriber('//visp_auto_tracker/status', Int8, callback)

	rospy.spin()


if __name__ == '__main__':

		listener()
	#except rospy.ROSInterruptException:
	#	pass
