#!/usr/bin/env python

import rospy

##try:
##    from math import pi, tau, dist, fabs, cos
##except:  # For Python 2 compatibility
##   from math import pi, fabs, cos, sqrt

from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray

def my_publisher():
   
    rospy.init_node('my_publisher_control')
    control_publisher = rospy.Publisher('/joint_states', JointState,queue_size=10)

    while not rospy.is_shutdown():
	msg = JointState()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = ''
	msg.name = []
	
	j1 = random.random()
	j2 = random.random()
	j3 = random.random()
	j4 = random.random()
	j5 = random.random()
	j6 = random.random()
	
	msg.position = [j1,j2,j3,j4,j5,j6]
	msg.velocity = []
	msg.effort = []

    	control_publisher.publish(msg)


if __name__ == '__main__':
    try:
        my_publisher()
    except rospy.ROSInterruptException:
        pass
