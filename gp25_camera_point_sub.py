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

    A=np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]]) 
    B=np.array([[data.pose.position.x],[data.pose.position.y],[data.pose.position.z],[1]])
    np.dot(A,B)
    print("Punto inicial del robot= ", np.dot(A,B))

    q_1 = np.arctan(data.pose.position.x/data.pose.position.y)
    print(q_1)

    

def listener():

    rospy.init_node('joint_states_listener')

    control_subscriber=rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
    
