#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# ball_chaser パッケージ内の ball_chaser というノードが起動
roslaunch ball_chaser ball_chaser.launch
