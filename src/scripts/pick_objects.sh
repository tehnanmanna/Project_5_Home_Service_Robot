#!/bin/sh
xterm  -e  " roscore " &
sleep 5
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"&
sleep 5 
xterm -e "roslaunch pick_object pick_object.launch"