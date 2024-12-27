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
    --privileged \
    --name "$CONTAINER_NAME" \
    --workdir /home/$USER \
    --user $USER_NAME:$USER_NAME \
    --user $USER_ID:$USER_GID \
    --env  USER_ID=$USER_ID \
    --env "DISPLAY=$DISPLAY" \
    --device /dev/dri:/dev/dri \
    --runtime nvidia \
    --gpus '"device=0"' \
    "$IMAGE_NAME"