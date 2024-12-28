#!/bin/bash

# 環境変数の設定
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# DISPLAY設定
DISPLAY=${DISPLAY:-:0}
echo "Using DISPLAY=${DISPLAY}"

# X11のアクセス権限設定
xhost +local:docker

# XAUTHファイル作成
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Docker起動
docker run -it --rm \
    --gpus all \
    --runtime=nvidia \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="LD_LIBRARY_PATH=/usr/lib/nvidia-390:$LD_LIBRARY_PATH" \
    --volume="${XSOCK}:${XSOCK}:rw" \
    --volume="${XAUTH}:${XAUTH}:rw" \
    --volume="/sys/class/drm:/sys/class/drm" \
    --volume="/dev/dri:/dev/dri" \
    --device=/dev/nvidia0 \
    --device=/dev/nvidiactl \
    --device=/dev/nvidia-modeset \
    --device=/dev/nvidia-uvm \
    --name=ubuntu16-opengl \
    ubuntu16-opengl-container
