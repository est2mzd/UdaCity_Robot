#!/bin/bash
set -e

cd ~/catkin_ws2/src/
git clone https://github.com/turtlebot/turtlebot_simulator

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名