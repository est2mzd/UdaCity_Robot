#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/src/
git clone https://github.com/turtlebot/turtlebot_simulator

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名

cd ./turtlebot_simulator/
sudo rm -rf .git/
sudo rm .gitignore