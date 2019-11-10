#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find my_map)/house.world" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_map)/map.yaml initial_pose_a:=0" &
sleep 1
xterm -e " rosrun add_markers add_markers_node" &
sleep 1
xterm -e " rosrun pick_objects pick_objects_node" &

trap "kill -TERM -$$" SIGINT
wait
