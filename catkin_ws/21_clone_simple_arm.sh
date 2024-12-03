#!/bin/bash
set -e

cd ~/catkin_ws/src/
git clone -b first_interaction https://github.com/udacity/RoboND-simple_arm/ simple_arm

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名