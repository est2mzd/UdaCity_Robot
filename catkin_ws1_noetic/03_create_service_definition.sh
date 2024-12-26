#!/bin/bash
set -e

cd ~/catkin_ws/src/simple_arm
mkdir -p srv
cd srv
touch GoToPosition.srv