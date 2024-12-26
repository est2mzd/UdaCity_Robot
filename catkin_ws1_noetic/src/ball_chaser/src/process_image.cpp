#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>

// グローバル変数としてサービスクライアントを定義（サービスをリクエスト可能）
ros::ServiceClient client;

// この関数は、command_robotサービスを呼び出し、指定された方向にロボットを動かします
void drive_robot(float lin_x, float ang_z)
{
    // TODO: サービスをリクエストし、指定された速度（lin_x, ang_z）を渡してロボットを動かす

    // サービスリクエストのオブジェクトを作成
    ball_chaser::DriveToTarget service;

    // リクエストに速度を入力
    service.request.linear_x  = lin_x;
    service.request.angular_z = ang_z;

    // サービスを呼び出して速度を送信
    if (client.call(service)) {
        ROS_INFO("Sent request: linear_x=%1.2f, angular_z=%1.2f", lin_x, ang_z);
    } else {
        ROS_ERROR("Failed to call service DriveToTarget");
    }    
}

// このコールバック関数は、画像データを継続的に処理します
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255-5; // 白色のピクセル値を定義

    // TODO: 画像内のすべてのピクセルをループ処理し、明るい白色のピクセルがあるか確認
    // そのピクセルが画像の左、中、右のどの領域にあるかを特定する
    // 白いボールの位置に応じてdrive_robot関数を呼び出し、速度を渡す
    // カメラで白いボールが検出されない場合は停止をリクエストする
    int height = img.height; // 画像の横幅（ピクセル数）
    int width  = img.width;  // 画像の縦幅（ピクセル数）
    int step   = img.step;   // 1行あたりのデータサイズ（バイト数）

    // encoding = "rgb8"（3チャンネル、1ピクセル = 3バイト）
    int bytes_per_pixel = 3; // encodingに基づく1チャンネルあたりのバイト数 × チャンネル数
    std::cout << "encoding = " << img.encoding << std::endl;

    // white_ball_pos: 0=not found, 1=left, 2=center, 3=right
    int white_ball_pos = 0;
    int col_left_boundary  = int(width/3); //int(width/5*2);
    int col_right_boundary = int(width/3*2); //int(width/5*3);

    // 移動速度
    float lin_x = 0.5;
    float ang_z = 0.2;

    for(int row=0; row < height; row++)
    {
        for(int col=0; col < width; col+=3)
        {
            // ピクセルデータのインデックスを計算
            int pixel_index = (row * step) + (col * bytes_per_pixel);

            // RGBを取得
            uint8_t red   = img.data[pixel_index];
            uint8_t green = img.data[pixel_index+1];
            uint8_t blue  = img.data[pixel_index+2];

            if (red >= white_pixel && green >= white_pixel && blue >= white_pixel)
            {
                if (col < col_left_boundary){
                    white_ball_pos = 1;
                    lin_x *= 0.0;
                    ang_z *= 1.0;
                }else if (col > col_right_boundary){
                    white_ball_pos = 3;
                    lin_x *= 0.0;
                    ang_z *= -1.0;
                }else{
                    white_ball_pos = 2;
                    ang_z *= 0.0;
                }

                break;
            }
        }

        if (white_ball_pos > 0){
            break;
        }
    }

    if (white_ball_pos > 0){
        drive_robot(lin_x, ang_z);
    }else{
        drive_robot(0.0, 0.0);
    }

    // 処理後にスリープを挿入
    //ros::Rate rate(3); // 10Hzで実行（1秒間に10回）
    //rate.sleep();

}

int main(int argc, char** argv)
{
    // process_imageノードを初期化し、ノードハンドルを作成
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // command_robotサービスにリクエストを送信できるクライアントサービスを定義
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // /camera/rgb/image_rawトピックにサブスクライブして、画像データをprocess_image_callback関数で処理
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // ROS通信イベントを処理
    ros::spin();

    return 0;
}
