#!/bin/bash

# 環境変数の初期設定
XSOCK=${XSOCK:-/tmp/.X11-unix}
XAUTH=${XAUTH:-/tmp/.docker.xauth}

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
    --security-opt apparmor=unconfined \
    --privileged \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=${XAUTH}" \
    --env="LIBGL_ALWAYS_INDIRECT=1" \
    --env="__GLX_VENDOR_LIBRARY_NAME=nvidia" \
    --env="LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu/mesa:$LD_LIBRARY_PATH" \
    --volume="${XSOCK}:${XSOCK}:rw" \
    --volume="${XAUTH}:${XAUTH}:rw" \
    --volume="/dev:/dev:rw" \
    --volume="/sys:/sys:rw" \
    --volume="/proc:/proc:rw" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri \
    --device=/dev/nvidia0 \
    --device=/dev/nvidiactl \
    --device=/dev/nvidia-modeset \
    --device=/dev/nvidia-uvm \
    --device=/dev/nvidia-uvm-tools \
    --device=/dev/nvidia-caps \
    --name=ubuntu16-opengl \
    ubuntu16-opengl-container