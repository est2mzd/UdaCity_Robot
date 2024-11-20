#!/bin/bash
# 使い方 : source ./common.sh
# ここで定義した変数がシェルの外側でも使える

#=====================================================#
# このファイルのパス
THIS_FILE_PATH=$(realpath "$0")
# このフォルダのパス
THIS_DIR_PATH=$(dirname ${THIS_FILE_PATH})
# 1つ上のディレクトリのパス
PARENT_DIR_PATH=$(dirname "$THIS_DIR_PATH")
# 2つ上のディレクトリのパス
GRANDPARENT_DIR_PATH=$(dirname "$PARENT_DIR_PATH")
#=====================================================#

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${THIS_DIR_PATH}/build
echo ${GAZEBO_PLUGIN_PATH}