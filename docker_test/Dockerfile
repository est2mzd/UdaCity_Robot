# ベースイメージの指定
FROM ubuntu:16.04

# タイムゾーン設定
ENV TZ=Asia/Tokyo
RUN apt-get update && apt-get install -y tzdata && \
    ln -sf /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

# 必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
    x11-apps \
    x11-xserver-utils \
    x11-utils \
    xorg \
    xauth \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    wget \
    sudo \
    curl \
    software-properties-common \
    && apt-get clean

# ユーザー設定
ARG USER_NAME=takuya
ARG USER_ID=1000

# ユーザー作成と権限設定
RUN useradd -m --uid ${USER_ID} --shell /bin/bash ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 互換パッケージのインストール (Ubuntu 16.04対応)
RUN apt-get update && apt-get install -y \
    libgl1-mesa-dev \
    libegl1-mesa \
    libgles2-mesa \
    libxrandr2 \
    libxinerama1 \
    libxcursor1 \
    libxi6 \
    && apt-get clean

# 必要パッケージのインストール
RUN apt-get update && apt-get install -y \
    pciutils \
    mesa-utils \
    && apt-get clean

# 権限の確認とセットアップ
RUN mkdir -p /tmp/.X11-unix && chmod 1777 /tmp/.X11-unix

# 環境変数の設定
ENV LIBGL_ALWAYS_INDIRECT=1
ENV QT_X11_NO_MITSHM=1

# デフォルトのユーザー設定
USER ${USER_NAME}

# 実行コマンド
CMD ["/bin/bash"]
