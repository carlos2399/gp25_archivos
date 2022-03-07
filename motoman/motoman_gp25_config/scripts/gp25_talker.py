#!/usr/bin/env python

import rospy

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('chatter2', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    joint_goal = Float64MultiArray()

    joint_goal.data = array
	joint_goal[0] = 2
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 2
        joint_goal[4] = 0
        joint_goal[5] = 3 

    pub.publish(joint_goal)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
