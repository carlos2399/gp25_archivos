#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String

def gp25_python_interface():

  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('gp25_python_interface', anonymous=True)

  robot = moveit_commander.RobotCommander()

  scene = moveit_commander.PlanningSceneInterface()

  brazo = moveit_commander.MoveGroupCommander("Motoman")

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

  print "============ Esperando a que RVIZ se abra ..."
  rospy.sleep(1)

  print "============ Reference frame: %s" % brazo.get_planning_frame()

  print "============ Reference frame: %s" % brazo.get_end_effector_link()
  
  print "============ Robot Groups:"
  print robot.get_group_names()

  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"


  ## PLANNING TO A POSE GOAL
  ## ^^^^^^^^^^^^^^^^^^^^^^^
  ## We can plan a motion for this group to a desired pose for the end-effector
  
  print "============ Generating plan 1"
  pose_target = geometry_msgs.msg.Pose()
  pose_target.orientation.w = 1.0
  pose_target.position.x = 0.7
  pose_target.position.y = -0.05
  pose_target.position.z = 1.1
  brazo.set_pose_target(pose_target)
	
  ## Now, we call the planner to compute the plan and visualize it if successful
  plan1 = brazo.plan()

  print "============ Waiting while RVIZ displays plan1"
  rospy.sleep(2)

  print "============ Visualizing plan1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()

  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);

  print "============ Waiting while plan1 for left arm is visualized again"
  rospy.sleep(2)

  ## PLANNING TO A JOINT-SPACE GOAL
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## Let's set a joint space goal and move towards it. 
  ## First, we will clear the pose target we had just set.
  brazo.clear_pose_targets()

  ## Then, we will get the current set of joint values for the group
  group_variable_values = brazo.get_current_joint_values()
  print "============ Joint values: ", group_variable_values

  ## Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan
  group_variable_values[0] = 1.0
  brazo.set_joint_value_target(group_variable_values)

  plan2 = brazo.plan()

  print "============ Waiting while RVIZ displays plan2"
  rospy.sleep(5)

  ## Cartesian Paths
  ## ^^^^^^^^^^^^^^^
  ## You can plan a cartesian path directly by specifying a list of waypoints for the end-effector to go through.
  waypoints = []

  # start with the current pose
  waypoints.append(brazo.get_current_pose().pose)

  # first orient gripper and move forward (+x)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x + 0.1
  wpose.position.y = waypoints[0].position.y 
  wpose.position.z = waypoints[0].position.z
  waypoints.append(copy.deepcopy(wpose))

  # second move down
  wpose.position.z -=0.10
  waypoints.append(copy.deepcopy(wpose))

  # third move to the side
  wpose.position.y += 0.05
  waypoints.append(copy.deepcopy(wpose))


  ## We want the cartesian path to be interpolated at a resolution of 1 cm
  ## which is why we will specify 0.01 as the eef_step in cartesian
  ## translation.  We will specify the jump threshold as 0.0, effectively
  ## disabling it.
  (plan3, fraction) = brazo.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # eef_step
                               0.0)         # jump_threshold
                            
  print "============ Waiting while RVIZ displays plan3..."
  rospy.sleep(5)

  ## Adding/Removing Objects and Attaching/Detaching Objects
  ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  ## First, we will define the collision object message
  collision_object = moveit_msgs.msg.CollisionObject()

  ## When finished shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

  ## END_TUTORIAL

  print "============ STOPPING"



if __name__=='__main__':
  try:
    gp25_python_interface()
  except rospy.ROSInterruptException:
    pass








