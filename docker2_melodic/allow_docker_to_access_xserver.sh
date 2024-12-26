#!/bin/bash

# docker ユーザー（ローカルからの接続）がXサーバーにアクセス可能になります。
# ホスト側で実行する
sudo xhost +local:docker

sleep 0.1
xhost