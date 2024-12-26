#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/
source devel/setup.bash

# パッケージ名 = turtlebot_gazebo
sudo apt-get update
rosdep -i install turtlebot_gazebo