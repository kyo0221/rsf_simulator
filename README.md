# rsf_simulator

Gazebo Harmonic 上につくばチャレンジのワールドと RSF ロボットを起動する。

## 動作環境

- Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic
- Git LFS（メッシュの取得に必要）

## 起動

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

`rsf_simulator` は Gazebo・ロボットの spawn・ROS bridge を起動する。ロボット記述と TF は `rsf_bringup` 側で起動する。

## ワールドの選択

| `world` | 内容 |
|---|---|
| `tsukuba`（デフォルト）| つくばチャレンジ2026コース |
| `tsudanuma` | 津田沼キャンパス |
| `tsudanuma2-3` | 津田沼キャンパス 2 号館 3 階 |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

## ワールドだけを起動

`robot:=none` でロボット・bridge を起動せず、ワールドだけを開く。自分のロボットは `ros_gz_sim create` で置く。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py robot:=none
```

`tsukuba` のスタート地点は x 24.808, y 3.303, z 25.049, yaw 3.065。

## topic

ロボットを起動した場合（既定）の入出力。

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
