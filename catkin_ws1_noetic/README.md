# GazeboとROS通信の設計

このプロジェクトでは、以下のパッケージを利用してGazeboとの通信を行っています。

- **`my_robot`**: Gazeboワールドとロボットモデルの定義。
- **`simple_arm`**: アームモデルとその制御に関するGazebo設定。
- **`ball_chaser`**: ボール追跡機能を実現し、Gazebo内でのロボットの動作を制御。

---

## 1. `my_robot`パッケージ

### Gazebo関連ファイル
- **`worlds/`**
  - `UdaCityOffice.world`:
    - 仮想オフィス環境を定義したGazeboワールド。
  - `UdaCityOfficeWithBall.world`:
    - ボールを含むオフィス環境のGazeboワールド。
  - `empty.world`:
    - 空のGazeboワールドファイル。

- **`urdf/`**
  - `my_robot.gazebo`:
    - Gazeboプラグイン（センサーや制御設定）を含むロボットモデル。
  - `my_robot.xacro`:
    - ロボットのリンクやジョイント構造を定義するURDFファイル。

- **`launch/`**
  - `robot_description.launch`:
    - URDFモデルをGazeboやRVizにロードするためのローンチファイル。
  - `world.launch`:
    - 指定したワールドファイル（例: `UdaCityOffice.world`）をGazeboで起動。

---

## 2. `simple_arm`パッケージ

### Gazebo関連ファイル
- **`worlds/`**
  - `willow_garage.world`:
    - Willow Garageを模したGazeboワールド。

- **`urdf/`**
  - `simple_arm.urdf.xacro`:
    - アームの物理構造を定義するURDFファイル。
  - `simple_arm.gazebo.xacro`:
    - Gazeboプラグイン（例: モーターやジョイントの挙動）を定義。

- **`launch/`**
  - `robot_spawn.launch`:
    - アームモデルをGazebo上にスポーンするためのローンチファイル。
  - `robot_control.xml`:
    - ロボットのモーターやジョイント制御を設定。
  - `robot_description.xml`:
    - URDFモデルをGazeboや制御ノードに渡すための記述。

---

## 3. `ball_chaser`パッケージ

### Gazeboとの通信
- `ball_chaser`は、Gazebo内で動作するロボットがボールを追跡する機能を提供します。このため、以下の仕組みでGazeboとの通信が実現されています。

### ファイル構成と通信の詳細
- **`src/`**
  - **`drive_bot.cpp` (サービスサーバー)**:
    - Gazebo上のロボットにモーター指令を送るノード。
    - **サーバー機能**: `DriveToTarget`サービスを提供し、クライアントからのリクエストを処理。
      - **受信データ**: 線形速度（`linear_x`）と角速度（`angular_z`）。
      - **送信データ**: Gazebo内のロボットモデルに対してモーター指令を送信。
  - **`process_image.cpp` (サービスクライアント)**:
    - Gazebo上のカメラセンサーから送られる画像データを解析。
    - **クライアント機能**: ボールの位置に基づいて`DriveToTarget`サービスを呼び出し、ロボットを移動。
      - **送信データ**: 線形速度と角速度のリクエスト。
      - **受信データ**: サーバーからのレスポンス（成功ステータス）。

- **`srv/`**
  - `DriveToTarget.srv`:
    - **データ形式**:
      - **リクエスト**: 線形速度（`float64 linear_x`）と角速度（`float64 angular_z`）。
      - **レスポンス**: 成功ステータス（`bool success`）。

- **`launch/`**
  - `ball_chaser.launch`:
    - 上記ノード（`drive_bot`と`process_image`）を起動し、Gazebo上でボール追跡を実現。

---

## Gazeboとの通信方法

1. **ワールドファイル（`.world`）**:
   - Gazeboのシミュレーション環境を定義。
   - `my_robot`や`simple_arm`のワールドファイルが利用され、ロボットや環境をロードします。

2. **URDFモデル（`.xacro`/`.gazebo`）**:
   - 各パッケージのURDFファイルはGazeboでロボットをシミュレートするための物理構造を定義。
   - Gazeboプラグインを用いて、ロボットのセンサーや制御を実現。

3. **ローンチファイル**:
   - Gazebo上でワールドの起動、ロボットモデルのスポーン、ノードの立ち上げを統合的に管理。

4. **サービス通信**:
   - サーバー（`drive_bot`）とクライアント（`process_image`）が、Gazebo上のロボットの動作を制御。
   - クライアントはカメラ画像を解析し、ボール追跡の指令をサーバーに送信。

---

この設計により、Gazebo内でのロボットの動作をリアルタイムに制御し、複雑なシミュレーション環境を構築することができます。
