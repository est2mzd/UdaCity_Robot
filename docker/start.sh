#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

# Dockerコンテナをデタッチモードで起動
# コンテナ内とディスプレイを共有するために下記を追加
#    --env "DISPLAY=$DISPLAY" \
#    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
docker run \
    --rm \
    -itd \
    -p 8888:8888 \
    -v "$WORK_DIR":/home/$USER \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /usr/share/glvnd:/usr/share/glvnd:rw \
    -v /usr/lib/x86_64-linux-gnu/nvidia:/usr/lib/x86_64-linux-gnu/nvidia:rw \
    --privileged \
    --name "$CONTAINER_NAME" \
    --workdir /home/$USER \
    --user $USER_NAME:$USER_NAME \
    --env  USER_NAME=$USER_NAME \
    --env  USER_ID=$USER_ID \
    --env "DISPLAY=$DISPLAY" \
    --device /dev/dri:/dev/dri \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --device /dev/dri:/dev/dri \
    --gpus all \
    "$IMAGE_NAME"