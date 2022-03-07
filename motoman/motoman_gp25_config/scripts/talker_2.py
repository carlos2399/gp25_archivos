#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('chatter2', Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    rate = rospy.Rate(10) # 10hz

    my_msg = Float64MultiArray()

    my_msg.data = [1,1,2,34,4,6]
    pub.publish(my_msg)
    rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
