#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

marker = Marker()

marker.header.frame_id = "/map"
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 1
marker.id = 1

# Set the scale of the marker
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set the pose of the marker
marker.pose.position.x = 0.0687380332163
marker.pose.position.y = 1.7182706521862
marker.pose.position.z = 0.351356442401
marker.pose.orientation.x = 0.703353472425
marker.pose.orientation.y = 0.687716366869
marker.pose.orientation.z = 0.0365396305466
marker.pose.orientation.w = 0.176082216502

while not rospy.is_shutdown():
  marker_pub.publish(marker)
  rospy.rostime.wallsleep(1.0)




