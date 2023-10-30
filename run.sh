!/bin/bash
gnome-terminal -- bash -c "source ~/catkin_ws/devel/setup.bash && roslaunch fetch_pickplace pickplace_playground.launch; exec bash"

sleep 2;

gnome-terminal -- bash -c "roslaunch fetch_moveit_config move_group.launch; exec bash"

sleep 2;

source ~/catkin_ws/devel/setup.bash
gnome-terminal -- bash -c "rosrun matlab_moveit_bridge /src/bridge_node.py; exec bash"
