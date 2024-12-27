#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/

# set ENVs
source devel/setup.bash

# turtlebot3_gazebo パッケージ内の turtlebot3_world.launch という名前のlaunchファイルを実行
roslaunch turtlebot_gazebo turtlebot_world.launch