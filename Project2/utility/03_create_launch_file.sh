#!/bin/bash
set -e

echo 1- Create the world.launch file
cd ~/catkin_ws/src/my_robot/launch/
touch world.launch

echo 2- Add the following to world.launch

echo '<?xml version="1.0" encoding="UTF-8"?>

<launch>

  <!-- World File -->
  <arg name="world_file" default="$(find my_robot)/worlds/empty.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

</launch>' > world.launch

echo Finish
