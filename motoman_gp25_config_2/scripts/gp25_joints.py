#!/usr/bin/env python

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
except: 
    from math import pi, fabs, cos, sqrt

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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

# Movimiento 1
    def go_to_joint_state(self):
        # Convertimos las variables en variables locales
        move_group = self.move_group

        # Cogemos los valores de las articulaciones y establecemos los valores deseados
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pi / 2
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = pi / 2
        joint_goal[4] = -pi / 6
        joint_goal[5] = pi / 3 
	
	print("La posicion objetivo es: ",joint_goal)
	
	# Con el siguiente comando mandamos al robot ir a la posicion indicada
        move_group.go(joint_goal, wait=True)

        # Llamando a stop() nos aseguramos de que no queden movimientos residuales
        move_group.stop()

# Movimiento 2
    def go_to_joint_state_2(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pi
        joint_goal[1] = pi/3
        joint_goal[2] = pi / 4
        joint_goal[3] = -pi
        joint_goal[4] = pi / 2
        joint_goal[5] = -pi / 6

	print("La posicion objetivo es: ",joint_goal)

        move_group.go(joint_goal, wait=True)
       
        move_group.stop()

# Movimiento 3
    def go_to_joint_state_3(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0 

	print("La posicion objetivo es: ",joint_goal)

        move_group.go(joint_goal, wait=True)
       
        move_group.stop()

def main():
	print("")
	print("----------------------------------------------------------")
	print("============ Bienvenido a la interfaz de control del Robot Motoman Gp25")
	print("----------------------------------------------------------")
	print("")
	input("============ Pulsa ENTER para establecer el estado del robot")
	robot_state = MoveGroupPythonInterfaceTutorial()
	print("============ Pulsa ENTER para realizar el movimiento")

	while not rospy.is_shutdown():

		input("")
		robot_state.go_to_joint_state()
		print("")

		input("============  Movimiento completado, pulsa ENTER para el siguiente movimiento o CTRL+D para salir del programa")
		print("")
		robot_state.go_to_joint_state_2()

		print("")
		print("============ Movimiento completado, pulsa ENTER para el siguiente movimiento o CTRL+D para salir del programa")

		input("")
		robot_state.go_to_joint_state_3()

		print("")
		print("============ Movimiento completado, pulsa ENTER repetir los movimientos o CTRL+D para salir del programa")
		print("")


if __name__ == "__main__":
    main()

