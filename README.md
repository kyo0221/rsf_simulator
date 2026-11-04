# rsf_simulator

Gazebo (Ignition Fortress / Gazebo Harmonic) 上に RSF ロボットとワールドを起動する。

## 起動

シミュレーションを使ったナビゲーションは、以下の順に起動する。`rsf_simulator` が Gazebo と ROS bridge を起動し、`rsf_bringup` は実機と同じロボット記述・TF を起動する。bringup から Gazebo は起動しない。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## ワールドの選択

| `world` | 内容 |
|---|---|
| `tsudanuma2-3`（デフォルト）| 津田沼キャンパス 2 号館 3 階 |
| `tsudanuma` | 津田沼キャンパス |
| `tsukuba` | つくばチャレンジ2026コース |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsukuba
```

## topic

| ROS トピック | 向き | 内容 |
|---|---|---|
| `/clock` | output | シミュレーション時刻 |
| `/rsf/hokuyo_cloud2` | output | 3D LiDAR の点群|
| `/rsf/rsf_odom` | output | オドメトリ(Gazebo の真値) |
| `/cmd_vel` | input | 速度指令 |

### rsf関連出力

| ROS トピック | 内容 |
|---|---|
| `/rsf/lio_lidar_rate_odom` | 点群と同じ時刻のセンサ姿勢|
| `/rsf/imu` | 実機の IMU 軸(y 鉛直・z 前方)に回した IMU、ジャイロバイアス付き |
| `/rsf/nav_sat_fix`、`/rsf/gpgga` | センサ位置から、作った GNSS |


## ライセンス

| 対象 | ライセンス |
|---|---|
| コード・設定・`models/orne_boxF`・`worlds/tsudanuma2-3.sdf` など下記以外 | Apache-2.0（`LICENSE`） |
| `models/tsukuba`・`worlds/tsukuba.sdf` | CC BY-NC-SA 4.0。ISE Robotics の点群地図 [map_tc26_gnss_ise-robotics](https://github.com/raucha/tc-datasets/tree/add-map) を変換・改変したもの（`models/tsukuba/NOTICE`、`models/tsukuba/LICENSE`） |
| `models/tsudanuma`・`worlds/tsudanuma.sdf` | BSD 2-Clause, Copyright (c) 2014, Robot Design and Control Lab.。[open-rdc/orne-box](https://github.com/open-rdc/orne-box/tree/humble/3d) の点群地図を変換・改変したもの（`models/tsudanuma/NOTICE`、`models/tsudanuma/LICENSE`） |
