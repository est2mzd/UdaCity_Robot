#!/bin/bash

set -e

# フォルダをGitリポジトリとして初期化します。
git init

# ファイルをステージング
git add .

# 初回のコミットを実行
git commit -m "Initial commit"

# リモートリポジトリを設定（必要な場合）
git remote add origin https://github.com/est2mzd/UdaCity_Robot.git

# 現在のブランチの名前を main にリネームすることです。
git branch -M main

# ローカルの変更をリモートリポジトリにプッシュ
git push -u origin main