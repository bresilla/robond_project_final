#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/trim/catkin_ws/src/World/MyWorld.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &  
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/trim/catkin_ws/src/World/map.yaml 3d_sensor:=kinect" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers_node"