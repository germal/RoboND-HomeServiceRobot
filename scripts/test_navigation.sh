#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/valerio/workspace/HomeService/src/map/house.world" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/valerio/workspace/HomeService/src/map/map.yaml initial_pose_a:=-1.57" &

trap "kill -TERM -$$" SIGINT
wait
