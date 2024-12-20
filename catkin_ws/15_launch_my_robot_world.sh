#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# my_robot パッケージ内の world.launch というノードが起動
roslaunch my_robot world.launch
