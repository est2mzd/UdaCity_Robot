#!/bin/bash
set -e

cd ~/catkin_ws2_kinetic/src/

# ブランチの調べ方
#git ls-remote --heads https://github.com/turtlebot/turtlebot_create.git

git clone https://github.com/turtlebot/turtlebot.git -b kinetic
git clone https://github.com/turtlebot/turtlebot_apps.git -b indigo
git clone https://github.com/turtlebot/turtlebot_simulator.git -b melodic
git clone https://github.com/turtlebot/turtlebot_interactions.git -b indigo
git clone https://github.com/turtlebot/turtlebot_create.git -b indigo
#
git clone https://github.com/udacity/robot_pose_ekf
git clone https://github.com/udacity/odom_to_trajectory

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名

sudo rm -rf ./turtlebot/.git/
sudo rm -rf ./turtlebot_apps/.git/
sudo rm -rf ./turtlebot_simulator/.git/
sudo rm -rf ./turtlebot_interactions/.git/
sudo rm -rf ./turtlebot_create/.git/
sudo rm -rf ./robot_pose_ekf/.git/
sudo rm -rf ./odom_to_trajectory/.git/
#sudo rm .gitignore