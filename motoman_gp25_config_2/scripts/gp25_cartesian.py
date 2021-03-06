#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonInterfaceTutorial(object):

	def __init__(self):

	  	super(MoveGroupPythonInterfaceTutorial, self).__init__()

		## Primero iniciamos el nodo
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

		robot = moveit_commander.RobotCommander()

		scene = moveit_commander.PlanningSceneInterface()

		## Llamamos al grupo creado en el asistentente de MoveIt
		group_name = "Motoman"
		move_group = moveit_commander.MoveGroupCommander(group_name)

		## Creamos el publicador para nuestras trayectorias
		display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20,)

		planning_frame = move_group.get_planning_frame()
		print("============ Planning frame: %s" % planning_frame)

		eef_link = move_group.get_end_effector_link()
		print("============ End effector link: %s" % eef_link)

		group_names = robot.get_group_names()
		print("============ Grupos Robot disponibles:", robot.get_group_names())

		print("============ Imprimiendo el estado del robot")
		print(robot.get_current_state())
		print("")

		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.display_trajectory_publisher = display_trajectory_publisher
		self.planning_frame = planning_frame
		self.eef_link = eef_link
		self.group_names = group_names		

	def gp25_python_interface(self):
					  
		move_group = self.move_group

		print ("============ Generating plan 1")
		pose_target = geometry_msgs.msg.Pose()
		pose_target.orientation.w = 0.1878
  		pose_target.position.x = 1.552
    		pose_target.position.y = -0.395
    		pose_target.position.z = 0.654
		move_group.set_pose_target(pose_target)
	
		plan1 = move_group.go(pose_target, wait=True)

		rospy.sleep(2)

def main():

		robot_state = MoveGroupPythonInterfaceTutorial()
		while not rospy.is_shutdown():
			input("============ Pulsa ENTER para establecer el estado del robot")
			robot_state.gp25_python_interface() 


if __name__ == "__main__":
	main()








