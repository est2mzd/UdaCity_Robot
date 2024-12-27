#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/
source devel/setup.bash

# パッケージ名 = turtlebot_gazebo
#echo "--- sudo apt-get update ---"
#sudo apt-get update

#echo "--- sudo rosdep update ---"
#sudo rosdep update

#echo "--- sudo rosdep fix-permissions ---"
#sudo rosdep fix-permissions

echo "--- rosdep -i install turtlebot_teleop ---"
rosdep -i install turtlebot_teleop