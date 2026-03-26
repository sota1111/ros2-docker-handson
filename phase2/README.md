# Phase 2 — EKF による自己位置推定

## 概要

Phase 2 では `robot_localization` パッケージの **EKF（Extended Kalman Filter）** を用いて、
オドメトリ（`/odom`）と IMU（`/imu`）を融合した高精度な自己位置推定を実現します。

```
/odom  ──┐
          ├──► ekf_filter_node ──► /odometry/filtered
/imu   ──┘                   └──► TF: odom → base_footprint
```

---

## ファイル構成

```
phase2/
├── Dockerfile                  # ROS Jazzy + TurtleBot3 + robot_localization
├── docker-compose.yml          # 3サービスの定義
└── ws/
    ├── config/
    │   └── ekf.yaml            # EKF ノードのパラメータ
    └── launch/
        └── ekf.launch.py       # ekf_node を起動する launchファイル
```

---

## 起動手順

### 1. Docker イメージのビルド

```bash
docker compose build
```

### 2. X11 転送の許可（GUI 表示用）

```bash
xhost +local:docker
```

### 3. 全サービスの起動

```bash
docker compose up
```

3つのコンテナが並列起動します。

| コンテナ名 | 内容 |
|-----------|------|
| `p2_gazebo` | TurtleBot3 シミュレーション世界 |
| `p2_ekf` | EKF 自己位置推定ノード |
| `p2_rviz` | RViz2 可視化ツール |

### 4. 停止

```bash
docker compose down
```

---

## 起動順序と依存関係

```
[1] Gazebo (turtlebot3_world.launch.py)
      │
      │  /odom, /imu トピックを publish
      ▼
[2] EKF (ekf.launch.py)
      │
      │  センサーを融合して /odometry/filtered を publish
      │  TF: odom → base_footprint を broadcast
      ▼
[3] RViz2
      │
      │  /odometry/filtered や TF を subscribe して可視化
      ▼
   画面上でロボットの動きを確認
```

> **注意:** Gazebo が起動してトピックを配信し始める前に EKF が起動しても問題ありません。
> EKF はトピックが来るまで待機し、受信を開始した時点でフィルタリングを始めます。

---

## launch ファイル解説

### `ekf.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/ws/config/ekf.yaml']
        )
    ])
```

| 項目 | 説明 |
|------|------|
| `package` | `robot_localization` — ROS 標準の状態推定パッケージ |
| `executable` | `ekf_node` — EKF 実装の実行ファイル |
| `name` | ノード名（`/ekf_filter_node`） |
| `parameters` | `ekf.yaml` をパラメータファイルとして読み込む |

---

## EKF パラメータ解説（`ekf.yaml`）

```yaml
ekf_filter_node:
  ros__parameters:
    use_sim_time: true      # Gazebo のシミュレーション時刻を使用
    frequency: 30.0         # 出力レート [Hz]
    two_d_mode: true        # 2D平面のみで推定（z, roll, pitch を無視）
    publish_tf: true        # TF ツリーへ odom→base_footprint を配信
```

### フレーム構成

```
map
 └── odom          ← world_frame（EKF の基準座標系）
      └── base_footprint   ← base_link_frame（ロボット中心）
```

| フレーム | 役割 |
|---------|------|
| `map` | 地図座標系（Phase 2 では未使用、SLAM で使用） |
| `odom` | オドメトリ原点（起動時のロボット位置） |
| `base_footprint` | ロボットの接地面中心 |

### センサー融合設定

EKF の状態ベクトルは 15 次元：
`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`

#### `/odom`（オドメトリ）

```yaml
odom0: /odom
odom0_config: [true,  true,  false,   # x, y を使用
               false, false, true,    # yaw を使用
               true,  true,  false,   # vx, vy を使用
               false, false, true,    # vyaw を使用
               false, false, false]   # 加速度は使用しない
```

#### `/imu`（IMU センサー）

```yaml
imu0: /imu
imu0_config: [false, false, false,   # 位置は使用しない
              false, false, true,    # yaw を使用
              false, false, false,   # 速度は使用しない
              false, false, true,    # vyaw を使用
              true,  false, false]   # ax（前後加速度）を使用
imu0_relative: true                  # IMU の初期姿勢を基準とする
```

---

## トピック一覧

| トピック名 | 型 | 方向 | 説明 |
|-----------|-----|------|------|
| `/odom` | `nav_msgs/Odometry` | 入力 | Gazebo が配信する車輪オドメトリ |
| `/imu` | `sensor_msgs/Imu` | 入力 | Gazebo が配信する IMU データ |
| `/odometry/filtered` | `nav_msgs/Odometry` | 出力 | EKF 融合済み自己位置 |
| `/tf` | `tf2_msgs/TFMessage` | 出力 | `odom → base_footprint` の TF |

---

## Phase 1 との違い

| 項目 | Phase 1 | Phase 2 |
|------|---------|---------|
| 自己位置 | `/odom` のみ | `/odom` + `/imu` を EKF 融合 |
| 使用ノード | なし（Gazebo 直結） | `ekf_filter_node` |
| TF 配信元 | Gazebo | EKF ノード |
| 誤差特性 | ドリフトしやすい | センサー融合で改善 |
