#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=src/map/weird2.world" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch pick_objects pick_objects.launch"