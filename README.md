# rsf_simulator

Gazebo (Ignition Fortress / Gazebo Harmonic) 上に RSF ロボットとワールドを起動する。

## 起動

シミュレーションを使ったナビゲーションは、以下の順に起動する。`rsf_simulator` が Gazebo と ROS bridge を起動し、`rsf_bringup` は実機と同じロボット記述・TF を起動する。bringup から Gazebo は起動しない。

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

端末 2 で bringup を起動する:

```bash
ros2 launch rsf_bringup rsf_bringup.launch.py sim:=true
```

端末 3 で Nav2 を起動する:

```bash
ros2 launch rsf_navigation_executor rsf_navigation.launch.py
```

waypoint 実行サービスを呼び出す:

```bash
ros2 service call /waypoint_navigator/start std_srvs/srv/Trigger '{}'
```

実機では Gazebo を起動せず、端末 1 で bringup、端末 2 で Nav2 を起動する:

```bash
ros2 launch rsf_bringup rsf_bringup.launch.py sim:=false
```

```bash
ros2 launch rsf_navigation_executor rsf_navigation.launch.py use_sim_time:=false
```

Gazebo だけを起動する場合:

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py
```

## ワールドの選択

| `world` | 内容 |
|---|---|
| `tsudanuma2-3`（デフォルト）| 津田沼キャンパス 2 号館 3 階 |
| `tsudanuma` | 津田沼キャンパス |
| `tsukuba` | つくば市役所周辺 |

```bash
ros2 launch rsf_simulator rsf_simulator.launch.py world:=tsudanuma
```

`worlds/` に `<名前>.sdf` を置き、launch の `choices` に名前を追加すれば選択肢を増やせる。

## topic

`ros_gz_bridge` が以下を ROS 側に橋渡しする。

| ROS トピック | 向き | 内容 |
|---|---|---|
| `/clock` | output | シミュレーション時刻 |
| `/rsf/hokuyo_cloud2` | output | 3D LiDAR の点群|
| `/rsf/rsf_odom` | output | オドメトリ(Gazebo の真値、モデル原点 = 回転中心の姿勢) |
| `/cmd_vel` | input | 速度指令 |

`rsf_device_emulator`(`scripts/rsf_device_emulator.py`)が、Gazebo に無い RSF-X001 の出力を作る。

| ROS トピック | 内容 |
|---|---|
| `/rsf/lio_lidar_rate_odom` | 点群と同じ時刻のセンサ姿勢(`/rsf/rsf_odom` に `base_to_sensor` を足し、ジッタと縮尺誤差を加える) |
| `/rsf/imu` | 実機の IMU 軸(y 鉛直・z 前方)に回した IMU、ジャイロバイアス付き |
| `/rsf/nav_sat_fix`、`/rsf/gpgga` | センサ位置(map=点群座標)から、rsf_3d_localization と同じ式の逆(原点 `datum_lat/lon`、`datum_alt`)で作った GNSS。`gnss_gap_zones` の区間は gps_qual 5(Float) |

パラメータは `config/rsf_device_emulator.yaml`(`emulator_params` 引数で差し替え)。誤差を 0、区間を空にすれば真値になる。

トピック名とフレーム名は実機ドライバ `hokuyo_rsf` の設定に揃えてあり、
実機とシミュレータで下流ノードの設定を変えずに済むようにしてある。
