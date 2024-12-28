#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/

# set ENVs
source devel/setup.bash

# 
#rosrun rviz rviz
rosrun rviz rviz -d ~/rviz/EKFLab.rviz

# OpenGLの競合回避策
#X :1 &
#DISPLAY=:1 rosrun rviz rviz -d ~/rviz/EKFLab.rviz