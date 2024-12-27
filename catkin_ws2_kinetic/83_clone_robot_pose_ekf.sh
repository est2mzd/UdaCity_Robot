#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/src/
git clone https://github.com/udacity/robot_pose_ekf

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名

cd ./robot_pose_ekf/
sudo rm -rf .git/
sudo rm .gitignore