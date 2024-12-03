#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# simple_arm パッケージ内の simple_mover というノードが起動
rosservice call /arm_mover/safe_move "{joint_1: 0.0 , joint_2: 0.0}"