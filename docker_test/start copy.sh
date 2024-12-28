#!/bin/bash

# 環境変数の設定
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# DISPLAY環境変数の取得とデフォルト設定
DISPLAY=${DISPLAY:-:0}  # デフォルト: :0 を使用
echo "Using DISPLAY=${DISPLAY}"

# X11アクセス権限付与
xhost +local:docker

# XAUTHファイル作成
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Dockerコンテナ起動
docker run -it --rm \
    --gpus all \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH}" \
    --volume="${XSOCK}:${XSOCK}:rw" \
    --volume="${XAUTH}:${XAUTH}:rw" \
    --device=/dev/dri \
    --name=ubuntu16-opengl \
    ubuntu16-opengl-container
