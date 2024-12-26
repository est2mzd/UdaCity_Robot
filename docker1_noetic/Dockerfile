# ベースイメージ
FROM osrf/ros:noetic-desktop-full

# メンテナ情報
LABEL maintainer="takuya@example.com"

# 非対話型設定
ENV DEBIAN_FRONTEND=noninteractive

# 必要なシステムパッケージをインストール
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    nano \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    mesa-utils \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libegl-mesa0 \
    libosmesa6 \
    x11-apps \
    sudo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 必要な追加ツールをインストール
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    build-essential \
    libeigen3-dev \
    libpcl-dev \
    python3 \
    python3-pip \
    && pip3 install --upgrade pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# OpenCVのインストール
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# NVIDIAリポジトリを追加
RUN curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
    curl -s -L https://nvidia.github.io/libnvidia-container/ubuntu20.04/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list && \
    apt-get update

# NVIDIA対応のライブラリをインストール
RUN apt-get update && apt-get install -y \
    nvidia-container-toolkit \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# 環境変数設定
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics

# ROSデフォルト設定をBashに追加
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Gazebo関連の必要なパッケージをインストール
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-effort-controllers \
    ros-noetic-joint-state-controller \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Python依存パッケージをインストール
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    jupyterlab \
    notebook && \
    pip3 install pcg-gazebo --no-deps && \
    pip3 install \
    trimesh svg.path colorlog numpy-quaternion

RUN sudo apt-get update && sudo apt-get upgrade -y

# ROS関連
RUN apt update && apt install vim -y

# ユーザー情報を設定
ARG USER_NAME
ARG USER_ID
RUN useradd -m -s /bin/bash -u $USER_ID $USER_NAME && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 権限を適切に設定
RUN chown -R $USER_NAME:$USER_NAME /home/$USER_NAME

# ROSの設定 : これはうまく行かなかった.ファイルが作成されない & 追記もできない
#           -> 手作業でファイルを作成したら、うまく読み込まれている
RUN cp /etc/skel/.bashrc /home/$USER_NAME/.bashrc && \
    chown -R ${USER_NAME}:${USER_NAME} /home/$USER_NAME/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/$USER_NAME/.bashrc

# デフォルトのユーザー切り替え
USER $USER_NAME
WORKDIR /home/$USER_NAME

# Gazeboに必須. Nvidia Driverを使うようするための設定
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV NVIDIA_VISIBLE_DEVICES=all
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia
    
# 起動時にBashを実行
CMD ["/bin/bash"]
