#!/bin/bash

# common.shを読み込む
source "$(dirname "$0")/common.sh"

# Dockerイメージをビルド
docker build \
    --build-arg USER_NAME=$USER_NAME \
    --build-arg USER_ID=$USER_ID \
    --build-arg USER_GID=$USER_GID \
    -t "$IMAGE_NAME" "$DOCKERFILE_DIR"

#--no-cache \