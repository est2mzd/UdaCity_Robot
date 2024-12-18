#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
// TODO: ball_chaserの"DriveToTarget"ヘッダーファイルをインクルードする
// このファイルはサービスのリクエストとレスポンス用に定義されます。
#include "ball_chaser/DriveToTarget.h" // ./devel/include/配下に自動生成される

// ROS::Publisher motor commands;
// ロボットのホイール速度指令を送信するためのパブリッシャを定義
ros::Publisher motor_command_publisher;

// TODO: drive_botサービスがリクエストされた時に実行されるコールバック関数を作成する
// この関数はリクエストされた直線速度 (linear x) と角速度 (angular z) を
// ロボットのホイール関節に送信します。
// 送信後、要求されたホイール速度を含むフィードバックメッセージを返します。

int main(int argc, char** argv)
{
    // ROSノードを初期化
    ros::init(argc, argv, "drive_bot");

    // ROSノードハンドルを作成
    ros::NodeHandle n;

    // ROSマスターに対し、geometry_msgs::Twist型のメッセージを
    // トピック"/cmd_vel"に対して送信することを通知。
    // キューサイズは10（最大10個のメッセージを保持）
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO: /ball_chaser/command_robotサービスを定義し、
    // リクエストを処理するhandle_drive_requestコールバック関数を紐付ける

    // TODO: このループを削除し、コードをコールバック関数内に移動する。
    // さらに、定数値ではなく、リクエストされた速度をパブリッシュするように修正する。
    while (ros::ok()) {
        // geometry_msgs::Twist型のmotor_commandオブジェクトを作成
        geometry_msgs::Twist motor_command;
        // 車輪の速度を設定: 前進速度0.5、角速度0.0
        motor_command.linear.x = 0.5;  // x軸方向の直線速度
        motor_command.angular.z = 0.0; // z軸回転の角速度
        // 設定した速度をパブリッシュしてロボットを駆動する
        motor_command_publisher.publish(motor_command);
    }

    // TODO: ROS通信イベントを処理する
    // コールバック関数が呼び出されるのを待機し、通信イベントを処理する。
    // ros::spin();

    return 0;
}
