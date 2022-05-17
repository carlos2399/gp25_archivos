#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray

def callback(data):
    rospy.loginfo("RECEIVE DATA: %s",data.position)
    rospy.loginfo("RECEIVE DATA: %s",data.name)

def listener():

    rospy.init_node('joint_states_listener')

    control_subscriber=rospy.Subscriber('/joint_states', JointState, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
