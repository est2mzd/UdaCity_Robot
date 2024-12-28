# ベースイメージ
FROM osrf/ros:kinetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ロケールの設定
RUN apt-get update && apt-get install -y locales && \
    locale-gen ja_JP.UTF-8 && \
    update-locale LC_ALL=ja_JP.UTF-8 LANG=ja_JP.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

ENV LANG=ja_JP.UTF-8
ENV LC_ALL=ja_JP.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/Asia/Tokyo /etc/localtime && \
    export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get install -y tzdata && \
    echo "Asia/Tokyo" > /etc/timezone && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

# 基本ツールのインストール
RUN apt-get update && \
    apt-get install -y \
        curl \
        wget \
        git \
        python-pip \
        build-essential \
        cmake \
        udev \
        linux-headers-4.4.0-210-generic

RUN apt-get update && apt-get install -y -qq --no-install-recommends \
        libgl1-mesa-glx \
        libegl1-mesa \
        libxext6 \
        libx11-6

# Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
#ENV QT_X11_NO_MITSHM=1

#===================================================================#
# Install X server and required dependencies
RUN apt-get update && apt-get install -y \
        x11-apps \
        x11-xserver-utils \
        x11-utils \
        xorg \
        xauth \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        mesa-utils \
    && rm -rf /var/lib/apt/lists/*

#===================================================================#
# ROS依存パッケージ（Gazebo関連）のインストール
RUN apt-get update && \
    apt-get install -y \
      ros-kinetic-gazebo-ros \
      ros-kinetic-gazebo-ros-pkgs \
      ros-kinetic-gazebo-ros-control \
      ros-kinetic-rviz

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      udev \
      python-catkin-tools

RUN apt-get update && \
    apt-get install -y \
      ros-kinetic-navigation \
      ros-kinetic-map-server \
      ros-kinetic-move-base \
      ros-kinetic-amcl \
      ros-kinetic-kobuki-gazebo-plugins \
      ros-kinetic-depthimage-to-laserscan \
      ros-kinetic-yocs-cmd-vel-mux \
      ros-kinetic-joy \
      ros-kinetic-yocs-velocity-smoother \
      ros-kinetic-kobuki-dashboard \
      ros-kinetic-turtlebot-msgs \
      ros-kinetic-compressed-image-transport \
      ros-kinetic-gmapping \
      ros-kinetic-kobuki-description \
      ros-kinetic-kobuki-safety-controller \
      ros-kinetic-depth-image-proc \
      ros-kinetic-kobuki-bumper2pc

#===================================================================#
# 環境変数の設定
ENV ROS_DISTRO=kinetic
ENV TURTLEBOT_GAZEBO_WORLD_FILE=/opt/ros/kinetic/share/turtlebot_gazebo/worlds/playground.world

#===================================================================#
# ユーザー情報を設定
ARG USER_NAME
ARG USER_UID

RUN useradd -m -s /bin/bash -u $USER_UID $USER_NAME && \
    echo "$USER_NAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 権限を適切に設定
RUN chown -R $USER_NAME:$USER_NAME /home/$USER_NAME

# デフォルトのユーザー切り替え
USER $USER_NAME
WORKDIR /home/$USER_NAME
#===================================================================#


# デフォルトのコマンドとしてbashを起動
CMD ["/bin/bash"]
