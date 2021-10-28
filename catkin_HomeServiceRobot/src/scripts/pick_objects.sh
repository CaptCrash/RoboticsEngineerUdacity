#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/src/map/mapmyworld.world" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/map.yaml" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch pick_objects pick_objects.launch"