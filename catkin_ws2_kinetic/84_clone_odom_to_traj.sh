#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/src/
git clone https://github.com/udacity/odom_to_trajectory

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名

cd ./odom_to_trajectory/
sudo rm -rf .git/
sudo rm .gitignore