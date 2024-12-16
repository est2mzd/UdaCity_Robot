#!/bin/bash
set -e

echo "1- Create a new launch file to load the URDF model file"

cd ~/catkin_ws/src/my_robot/launch/
touch robot_description.launch

echo "2- Copy the following code into robot_description.launch file"

# ここは手書きで対応
#FILE_PATH="robot_description.launch"
#
#cat <<EOF > ${FILE_PATH}
#<?xml version="1.0"?>
#<launch>
#
#<!-- send urdf to param server -->
#  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_robot)/urdf/my_robot.xacro'" />
#
#</launch>
#EOF

echo Finish
