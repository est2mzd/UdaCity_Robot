#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# simple_arm パッケージ内の robot_spawn.launch という名前のlaunchファイルを実行
roslaunch simple_arm robot_spawn.launch