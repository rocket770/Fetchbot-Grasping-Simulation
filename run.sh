#!/bin/bash
source ~/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "roslaunch fetch_gazebo pickplace_playground.launch; exec bash"

sleep 1;
source ~/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "roslaunch fetch_moveit_config move_group.launch; exec bash"

sleep 1;

source ~/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "rosrun matlab_moveit_bridge /src/bridge_node.py; exec bash"
