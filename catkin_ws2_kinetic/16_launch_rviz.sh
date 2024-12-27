#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/

# set ENVs
source devel/setup.bash

# 
rosrun rviz rviz
#rosrun rviz rviz -d ~/rviz/EKFLab.rviz

# OpenGLの競合回避策
#X :2 &
#DISPLAY=:2 rosrun rviz rviz -d ~/catkin_ws2_kinetic/src/EKFLab.rviz