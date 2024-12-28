#!/bin/bash

# 環境変数設定
USER_NAME=takuya
USER_ID=$(id -u)

# イメージのビルド
docker build --build-arg USER_NAME=${USER_NAME} --build-arg USER_ID=${USER_ID} -t ubuntu16-opengl-container .
