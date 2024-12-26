#!/bin/bash
set -e

cd ~/catkin_ws/src/
catkin_create_pkg ball_chaser roscpp std_msgs message_generation

cd ~/catkin_ws/src/ball_chaser/
mkdir srv
mkdir launch