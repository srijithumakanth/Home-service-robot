#!/bin/bash
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; export TURTLEBOT_GAZEBO_WORLD_FILE="/home/workspace/catkin_ws/src/home_service_robot/world/myworld.world"; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 7
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; export TURTLEBOT_GAZEBO_MAP_FILE="/home/workspace/catkin_ws/src/home_service_robot/world/my_world.yaml"; roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10
xterm -e "source /home/workspace/catkin_ws/devel/setup.bash; rosrun add_markers add_markers"