#!/bin/bash
set -e

cd ~/catkin_ws2/
source devel/setup.bash
rosdep -i install turtlebot_gazebo