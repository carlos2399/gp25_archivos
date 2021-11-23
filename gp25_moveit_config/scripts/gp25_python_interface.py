#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "Motoman"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pi / 2
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = pi / 2
        joint_goal[4] = 0
        joint_goal[5] = pi / 3 
        

        # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)

    def go_to_joint_state_2(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        move_group = self.move_group

        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0 
        

        # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        #current_joints = move_group.get_current_joint_values()
        #return all_close(joint_goal, current_joints, 0.01)

def main():
    #try:
	print("")
	print("----------------------------------------------------------")
	print("Welcome to the GP25 MoveIt Python Interface")
	print("----------------------------------------------------------")
	print("Press Ctrl-D to exit at any time")
	print("")
	input(
	    "============ Press `Enter` to set the robot state"
	)
	tutorial = MoveGroupPythonInterfaceTutorial()
	while not rospy.is_shutdown():
		input(
		    "============ Press `Enter` to execute the first movement using a joint state goal"
		)
		tutorial.go_to_joint_state()

		input(
		    "============ Press `Enter` to return the robot to the initial state using a joint state goal"
		)
		tutorial.go_to_joint_state_2()

		print("============ Movement complete")
		print("============ Press enter to repeat te same movement or Ctrl-D for exit")
    #except rospy.ROSInterruptException:
     #   return
    #except KeyboardInterrupt:
     #   return


if __name__ == "__main__":
    main()

