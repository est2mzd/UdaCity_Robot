#!/bin/bash
set -e

cd ~/catkin_ws2_noetic/

# set ENVs
source devel/setup.bash

# turtlebot3_teleop パッケージ内の turtlebot3_teleop_key.launch という名前のlaunchファイルを実行
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
