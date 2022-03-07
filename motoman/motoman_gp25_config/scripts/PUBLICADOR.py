#!/usr/bin/env python

import rospy


from sensor_msgs.msg import JointState
from std_msgs.msg import String,Float64MultiArray


class BagToJointState:
    def __init__(self):
        self.joint_state = JointState()
        self.joint_state.name = []
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.bag_sub = rospy.Subscriber("joint_states", JointState, self.bag_info_callback, queue_size=1)

    def bag_info_callback(self, msg):

        self.joint_state.header.stamp = rospy.Time.now()
        self.joint_state.position = msg.position
        self.joint_pub.publish(self.joint_state)

if __name__ == '__main__':
    try:
        rospy.init_node('bag_to_joint_state', anonymous=True)
        bag_to_joint_state = BagToJointState()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
