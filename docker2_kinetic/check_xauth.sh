#!/bin/bash

#===================================================================#
#$XAUTH は、X11（X Window System）の認証ファイルパスを指す環境変数です。
#
#背景と役割
#X11は、Unix系OS（Linuxなど）でGUIアプリケーションをリモートで表示する際に使われるシステムです。
#このとき、セキュリティを確保するために、Xサーバーへの接続許可を管理する認証ファイルが利用されます。
#
#具体的な用途
#$XAUTH は、通常 ~/.Xauthority（ホームディレクトリ直下の隠しファイル）を指します。このファイルには、Xサーバーへのアクセス権限情報が保存されています。

#使用例：DockerコンテナとX11転送
#Dockerコンテナ内でGUIアプリケーションを起動する場合、ホスト側のXサーバーを利用する必要があります。このとき、ホストの$XAUTH情報をコンテナ内に渡して、Xサーバーへのアクセスを許可します。

#確認方法 : 下記がともに空でなければOK
#  echo $XAUTH
#
#  xauth list
#===================================================================#


# XAUTH のチェック
if [ -z "$XAUTH" ]; then
    echo "XAUTH is empty. Creating .Xauthority and generating X11 authentication..."

    # ~/.Xauthority の作成
    touch ~/.Xauthority

    # DISPLAY 環境変数の確認
    if [ -z "$DISPLAY" ]; then
        echo "DISPLAY is not set. Please set DISPLAY and try again."
        exit 1
    fi

    # 認証情報の生成
    xauth generate $DISPLAY . trusted

    # 上記コマンドで ~/.Xauthority が 0 kb の場合、一時ファイルをコピーしたほうが良い
    # cp /run/user/1000/gdm/Xauthority ~/.Xauthority

    echo "Xauthority file created and authentication generated."
    echo "XAUTH = $XAUTH"
else
    echo "XAUTH is already set to: $XAUTH"
fi

#========================= XAUTHが空の場合の処理 =======================
# XAUTH のチェック
if [ -z "$XAUTH" ]; then
    echo "XAUTH is empty. Adding export to .bashrc..."

    # 既に.bashrc に追加済みかを確認
    if ! grep -q 'export XAUTH=$HOME/.Xauthority' ~/.bashrc; then
        # .bashrc の最後に追加
        echo 'export XAUTH=$HOME/.Xauthority' >> ~/.bashrc
        echo "Added export XAUTH to .bashrc"
    else
        echo "XAUTH export already exists in .bashrc"
    fi
else
    echo "XAUTH is already set to: $XAUTH"
fi