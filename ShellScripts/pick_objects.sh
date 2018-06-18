#!/bin/sh
export TURTLEBOT_GAZEBO_WORLD_FILE=$HOME/catkin_ws/src/World/MyWorld.world
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &  
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$HOME/catkin_ws/src/World/map.yaml 3d_sensor:=kinect" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node"


