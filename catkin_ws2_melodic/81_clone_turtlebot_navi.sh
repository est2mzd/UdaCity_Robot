#!/bin/bash
set -e

cd ./src/
git clone -b melodic-devel https://github.com/turtlebot/turtlebot_navigation.git

# -b        : ブランチを指定
# 最後の引数 : クローン先のフォルダ名

cd ./turtlebot_navigation/
sudo rm -rf .git/
sudo rm .gitignore