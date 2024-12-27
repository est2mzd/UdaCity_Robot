#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/

# set ENVs
source devel/setup.bash

# turtlebot3_teleop パッケージ内の turtlebot3_teleop_key.launch という名前のlaunchファイルを実行
roslaunch odom_to_trajectory create_trajectory.launch 
