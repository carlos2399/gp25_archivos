#!/bin/bash

1 - cd $HOME/catkin_ws && source $HOME/catkin_ws/devel/setup.bash  

2 - roscore (diferente pestaña)

3 - rosparam load /home/ros/catkin_ws/src/motoman/motoman_gp25_support/urdf/gp25.urdf robot_description

4 - rosparam set controller_joint_names "[joint_1_s, joint_2_l, joint_3_u, joint_4_r, joint_5_b, joint_6_t]" &

5 - roslaunch motoman_driver robot_interface_streaming_yrc1000.launch robot_ip:=192.168.3.33 &

# Pestaña 2:
echo "-------------------- robot_enable --------------------"
rosservice call /robot_enable
sleep 2
#Posicion abajo
echo "-------------------- mover brazo hacia abajo --------------------"
rosrun motoman_driver move_to_joint.py "[1.851489543914795, 0.1814519315958023, -0.44645413756370544, -0.23572798073291779, 0.8488105535507202, 3.4821078777313232]"
sleep 10
#Posicion arriba
echo "-------------------- mover brazo hacia arriba --------------------"
rosrun motoman_driver move_to_joint.py "[1.8940110206604004, -0.5322883129119873, -0.4464322328567505, -0.23572798073291779, 0.8488105535507202, 3.4821078777313232]"
