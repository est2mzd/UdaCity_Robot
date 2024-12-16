#!/bin/bash
set -e

echo 1- Create an empty Gazebo world
cd ~/catkin_ws/src/my_robot/worlds/
touch empty.world

echo 2- Add the following to empty.world

echo '<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen="0">
      <camera name="world_camera">
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>' > empty.world

echo Finish