#!/usr/bin/env python

from __future__ import print_function
from six.moves import input

import rospy
import numpy as np

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except: 
    from math import pi, fabs, cos, sqrt


from std_msgs.msg import String,Float64MultiArray
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

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

        # Establecemos las variables locales
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names 

def callback(data):
       	rospy.loginfo("POSITION: %s",data.pose.position)
      	rospy.loginfo("ORIENTATION: %s",data.pose.orientation)

        x = 1
        y = 0.5
        z = 1

        A=np.array([[1,0,0,x],[0,1,0,y],[0,0,1,z],[0,0,0,1]]) 
        B=np.array([[data.pose.position.x],[data.pose.position.y],[data.pose.position.z],[1]])
        m_1 = np.dot(A,B)
        print("Punto inicial del robot= ", m_1)
  	if data.pose.position.y == 0:

		q_1 = pi/2
	else:

        	q_1 = np.arctan(data.pose.position.x/data.pose.position.y)
        	print(q_1)

        move_group = robot_state.move_group
       
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = q_1
      
        print("La posicion objetivo es: ",joint_goal)
	
        move_group.go(joint_goal, wait=True)
      
        move_group.stop()

	q1_min = -1
	q1_max = 1
 

def listener():

        #rospy.init_node('joint_states_listener')

        control_subscriber=rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, callback)

        rospy.spin()

robot_state = MoveGroupPythonInterfaceTutorial()

def main():
	print("")
	print("----------------------------------------------------------")
	print("============ Bienvenido a la interfaz de control del Robot Motoman Gp25")
	print("----------------------------------------------------------")
	print("")
	input("============ Pulsa ENTER para establecer el estado del robot")
	#robot_state = MoveGroupPythonInterfaceTutorial()
	print("============ Pulsa ENTER para realizar el movimiento")

	while not rospy.is_shutdown():

		input("")
		listener()
		print("")


if __name__ == "__main__":
    main()

