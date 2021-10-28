#!/bin/sh
xterm -e " source devel/setup.bash; export ROBOT_INITIAL_POSE='-x 0 -y -3 -Y 0'; roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/src/map/mapmyworld.world" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/src/map/map.yaml" &
sleep 5
xterm -e " source devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " source devel/setup.bash; roslaunch add_markers add_markers.launch" &
#sleep 5
xterm -e " source devel/setup.bash; roslaunch pick_objects pick_objects.launch"