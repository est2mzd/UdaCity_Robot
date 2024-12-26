#!/bin/bash
set -e

cd ~/catkin_ws/

# set ENVs
source devel/setup.bash

# rqt_image_view パッケージ内の rqt_image_view というノードが起動
rosrun rqt_image_view rqt_image_view
