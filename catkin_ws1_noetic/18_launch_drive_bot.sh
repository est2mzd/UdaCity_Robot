#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# ball_chaser パッケージ内の drive_bot というノードが起動
rosrun ball_chaser drive_bot
