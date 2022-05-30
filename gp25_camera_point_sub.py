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

	x = 1
    	y = 0.5
    	z = 1
	
	dato_1 = data.pose.position.x
	dato_2 = data.pose.position.y
	dato_3 = data.pose.position.z

    	A=np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]]) 
    	B=np.array([[dato_1],[dato_2],[dato_3],[1]])
	m_1 = np.dot(A,B)    	
	
	print("Punto inicial del robot= ", m_1)

	#CALCULO Q1
    	if m_1[0] == 0:

		q_1 = pi/2
    	else:

        	q_1 = np.arctan(m_1[1]/m_1[0])
        	print("Punto q_1= ",q_1[0])

	#CALCULO Q3
	l_1 = 0.505
	l_2 = 0.76
	l_3 = 0.795
	
	cos_q_3 = (pow(m_1[0],2)+pow(m_1[1],2)+pow(m_1[2],2)-pow(l_2,2)-pow(l_3,2))/(2*l_2*l_3)
	print(cos_q_3)
	sqrt_3 = np.sqrt(1-pow(cos_q_3,2))
	print(sqrt_3)
	q_3 = np.arctan(-sqrt_3/cos_q_3)
	print('Punto q_3= ',q_3[0])
	print(np.isnan(q_3[0]))

	#CALCULO Q2
	sqrt_2 = np.sqrt(pow(m_1[0],2)+pow(m_1[1],2))

	q_2 = np.arctan(m_1[2]/-sqrt_2)-np.arctan((l_3*np.sin(q_3))/(l_2+l_3*cos_q_3))
	
	print('Punto q_2= ',q_2[0])

def listener():

	rospy.init_node('joint_states_listener')

	control_subscriber=rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)

	rospy.spin()

if __name__ == '__main__':
    	listener()
    
