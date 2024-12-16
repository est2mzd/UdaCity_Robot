#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# simple_arm パッケージ内の simple_mover というノードが起動
roslaunch my_robot world.launch
