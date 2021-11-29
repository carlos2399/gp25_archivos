#!/bin/bash
# hola 

# Pestaña 1:
echo "-------------------- Init roscore --------------------"
cd $HOME/catkin_ws && source $HOME/catkin_ws/devel/setup.bash && roscore
sleep 2

# Pestaña 2:
echo "-------------------- Load URDF --------------------"
rosparam load /home/ros/URDF_github/yaskawa_gp25_support/urdf/model.urdf robot_description &
sleep 2
echo "Set controller"
rosparam set controller_joint_names "[joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]" &
sleep 2

# Pestaña 1:
echo "-------------------- terminar roscore --------------------"
kill -INT 888 # Ctrl + c ( terminar proceso roscore ) 
sleep 2
echo "-------------------- iniciar motoman --------------------"
roslaunch motoman_driver robot_interface_streaming_yrc1000.launch robot_ip:=192.168.3.33 &
sleep 2

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
