#!/bin/bash
set -e

echo 1. Create and initialize a catkin_ws
#mkdir -p ~/catkin_ws/src
#cd ~/catkin_ws/src
#catkin_init_workspace

echo 2. Navigate to the src directory of your catkin_ws and create the my_robot package:
cd ~/catkin_ws/src/
catkin_create_pkg my_robot

echo 3. Next, create a worlds directory and a launch directory, that will further define the structure of your package:
cd ~/catkin_ws/src/my_robot/
mkdir launch
mkdir worlds