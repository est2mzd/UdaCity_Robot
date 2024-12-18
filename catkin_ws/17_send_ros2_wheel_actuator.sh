#!/bin/bash
set -e

# catkinワークスペースのディレクトリに移動
cd ~/catkin_ws/

# ROSの環境変数を設定
source devel/setup.bash

# ロボットを前進させるために、/cmd_velトピックにTwistメッセージをパブリッシュ
#   cmd_velは、ロボットの移動（前進・回転）を指令する標準的なトピックです。
#   ロボットは前方に0.1 m/sの速度で動きます。
#   ロボットは反時計回りに0.1ラジアン/秒で回転します。
rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1"