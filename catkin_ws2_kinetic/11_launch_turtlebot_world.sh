#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/

# set ENVs
source devel/setup.bash

# 一時的にソフトウェアレンダリングを使用して、OpenGL ドライバとの競合を回避
# .bashrc に追記した
# export LIBGL_ALWAYS_SOFTWARE=1 


roslaunch turtlebot_gazebo turtlebot_world.launch

# OpenGLの競合回避策
#X :1 &
#DISPLAY=:1 roslaunch turtlebot_gazebo turtlebot_world.launch