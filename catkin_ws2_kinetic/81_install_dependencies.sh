#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/
source devel/setup.bash

# パッケージ名 = turtlebot_gazebo
echo "--- sudo apt-get update ---"
sudo apt-get update

#echo "--- sudo rosdep init ---"
#sudo rosdep init

echo "--- sudo rosdep update ---"
sudo rosdep update

echo "--- sudo rosdep fix-permissions ---"
sudo rosdep fix-permissions

echo "--- rosdep -i install turtlebot_gazebo ---"
rosdep -i install turtlebot_gazebo

echo "--- rosdep -i install turtlebot_teleop ---"
rosdep -i install turtlebot_teleop

echo "--- rosdep -i install ALL---"
#rosdep install --from-paths src --ignore-src -r -y