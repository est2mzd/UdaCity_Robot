#!/bin/bash
set -e

cd ~/catkin_ws2/
source devel/setup.bash

# パッケージ名 = turtlebot_gazebo 
rosdep -i install turtlebot_gazebo