#!/bin/bash
set -e

cd ~/catkin_ws2_noetic/

# set ENVs
source devel/setup.bash

# turtlebot3_gazebo パッケージ内の turtlebot3_world.launch という名前のlaunchファイルを実行
roslaunch turtlebot3_gazebo turtlebot3_world.launch