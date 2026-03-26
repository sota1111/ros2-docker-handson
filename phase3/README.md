# Phase 3 — AMCL による地図ベース自己位置推定

## 概要

Phase 3 では既存の地図（`map.yaml` / `map.pgm`）を使い、**AMCL（Adaptive Monte Carlo Localization）** でグローバルな自己位置推定を行います。
EKF は Phase 2 から引き続き使用し、AMCL と組み合わせることで局所・グローバル両方の推定を実現します。

```
/odom  ──┐
          ├──► ekf_filter_node ──► TF: odom → base_footprint
/imu   ──┘

/scan  ──┐
          ├──► amcl ──► TF: map → odom
/map   ──┘        └──► /particle_cloud（粒子群）
```

---

## ファイル構成

```
phase3/
├── Dockerfile                          # Phase 2 + AMCL 関連パッケージを追加
├── docker-compose.yml                  # 4サービスの定義
└── ws/
    ├── config/
    │   ├── amcl.yaml                   # AMCL ノードのパラメータ
    │   ├── ekf.yaml                    # EKF ノードのパラメータ（Phase 2 から継続）
    │   └── rviz.rviz                   # RViz2 表示設定（ParticleCloud 追加）
    ├── launch/
    │   ├── amcl.launch.py              # AMCL + map_server + lifecycle_manager
    │   ├── ekf.launch.py               # EKF（Phase 2 から継続）
    │   └── localization.launch.py      # EKF + AMCL を時間差で統合起動
    └── maps/
        ├── map.pgm                     # 地図画像（占有格子地図）
        └── map.yaml                    # 地図メタデータ
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

4つのコンテナが起動します。

| コンテナ名 | 内容 |
|-----------|------|
| `p3_gazebo` | TurtleBot3 シミュレーション世界 |
| `p3_localization` | EKF + AMCL 自己位置推定（時間差起動） |
| `p3_rviz` | RViz2 可視化ツール |
| `p3_teleop` | キーボード操縦 |

### 4. 停止

```bash
docker compose down
```

---

## 起動順序と依存関係

Phase 3 では Gazebo の準備が整う前にノードが起動するとエラーが発生するため、`localization.launch.py` 内で **TimerAction による遅延起動** を使っています。

```
[1] Gazebo (turtlebot3_world.launch.py)
      │  /clock, /odom, /imu, /scan, /tf を publish し始める
      │
      ▼  (5秒待機)
[2] EKF (ekf.launch.py)
      │  TF: odom → base_footprint を broadcast
      │
      ▼  (さらに10秒待機、合計15秒)
[3] AMCL (amcl.launch.py)
      │   map_server → amcl の順に lifecycle_manager が起動
      │   /scan と TF: odom → base_footprint が揃ってから動作
      │   TF: map → odom を broadcast
      │   /particle_cloud を publish
      ▼
[4] RViz2
      │   地図・粒子群・スキャンを可視化
      ▼
   画面上でロボットの位置と粒子群を確認
```

> **なぜ遅延が必要か**
> - EKF は `/clock` が来る前に起動すると誤った時刻で動作する
> - AMCL は `/scan` と `odom → base_footprint` TF が揃っていないと `MessageFilter` エラーになる

---

## launch ファイル解説

### `localization.launch.py`（統合起動）

```python
TimerAction(period=5.0,  actions=[IncludeLaunchDescription(...ekf.launch.py...)])
TimerAction(period=15.0, actions=[IncludeLaunchDescription(...amcl.launch.py...)])
```

引数として地図ファイルと初期位置を受け取り、AMCL へ渡します。

```bash
# docker-compose.yml での指定
ros2 launch /ws/launch/localization.launch.py \
  map:=/ws/maps/map.yaml \
  init_x:=-2.0 init_y:=-0.5 init_yaw:=0.0
```

### `amcl.launch.py`（AMCL 起動）

3つのノードを起動します。

| ノード | パッケージ | 役割 |
|--------|-----------|------|
| `map_server` | `nav2_map_server` | 地図画像を `/map` トピックで配信 |
| `amcl` | `nav2_amcl` | パーティクルフィルタで自己位置推定 |
| `lifecycle_manager` | `nav2_lifecycle_manager` | `map_server → amcl` の順に起動管理 |

> **lifecycle_manager とは**
> Nav2 のノードは ROS 2 Lifecycle ノードとして実装されており、起動・設定・アクティブ化のステートを管理する必要があります。`lifecycle_manager` がその順序制御を自動で行います。

---

## AMCL パラメータ解説（`amcl.yaml`）

### 粒子フィルタ設定

```yaml
min_particles: 200
max_particles: 5000
```

粒子数は自動調整されます。位置が不確かなほど多くの粒子を使い、収束するにつれて減ります。

### 更新トリガー

```yaml
update_min_d: 0.05   # 5cm 移動するごとに更新
update_min_a: 0.05   # 0.05rad 回転するごとに更新
```

ロボットを少し動かすだけで粒子が収束していく様子を観察できます。

### センサーモデル

```yaml
# ロボット運動モデル
robot_model_type: nav2_amcl::DifferentialMotionModel

# レーザーモデル
laser_model_type: likelihood_field
laser_max_beams: 60
laser_z_hit: 0.9     # スキャン一致度を重視
laser_z_rand: 0.05
```

### 初期位置の設定

```yaml
set_initial_pose: true
always_reset_initial_pose: true
initial_pose:
  x: 0.0
  y: 0.0
  yaw: 0.0
```

`always_reset_initial_pose: true` により、再起動のたびに初期位置がリセットされます。
launch 引数（`init_x`, `init_y`, `init_yaw`）でこの値を上書きできます。

---

## トピック一覧

| トピック名 | 型 | 方向 | 説明 |
|-----------|-----|------|------|
| `/odom` | `nav_msgs/Odometry` | 入力(EKF) | 車輪オドメトリ |
| `/imu` | `sensor_msgs/Imu` | 入力(EKF) | IMU データ |
| `/scan` | `sensor_msgs/LaserScan` | 入力(AMCL) | LiDAR スキャン |
| `/map` | `nav_msgs/OccupancyGrid` | 入力(AMCL) | map_server が配信する地図 |
| `/odometry/filtered` | `nav_msgs/Odometry` | 出力(EKF) | EKF 融合済み自己位置 |
| `/particle_cloud` | `nav2_msgs/ParticleCloud` | 出力(AMCL) | 粒子群（RViz で可視化） |
| `/tf` | `tf2_msgs/TFMessage` | 出力 | `map → odom → base_footprint` の TF チェーン |

---

## TF ツリー

```
map
 └── odom          ← AMCL が broadcast（グローバル位置補正）
      └── base_footprint   ← EKF が broadcast（局所的な連続推定）
           └── base_scan   ← Gazebo が broadcast（LiDAR の取り付け位置）
```

EKF と AMCL が協調して完全な TF チェーンを構成します。

---

## Phase 2 との違い

| 項目 | Phase 2 | Phase 3 |
|------|---------|---------|
| 自己位置推定 | EKF のみ（局所） | EKF + AMCL（局所 + グローバル） |
| 地図 | 不使用 | 事前に作成した占有格子地図を使用 |
| LiDAR | 未使用 | `/scan` を AMCL に入力 |
| TF: `map → odom` | なし | AMCL が配信 |
| コンテナ数 | 3（gazebo, ekf, rviz） | 4（gazebo, localization, rviz, teleop） |
| 起動方法 | 各ノードを直接起動 | `localization.launch.py` が時間差で統合起動 |
| 追加パッケージ | — | `nav2_amcl`, `nav2_map_server`, `nav2_lifecycle_manager` |
