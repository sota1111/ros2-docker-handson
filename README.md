# ROS2 Jazzy ハンズオン

TurtleBot3を使ったROS2 Jazzy入門ハンズオンです。
Phase 0〜7を通じて、シミュレーション環境の構築から SLAM による地図生成・自律移動・外部 API 連携・Open-RMF 統合までを段階的に学びます。

---

## フェーズ一覧

| Phase | テーマ | 主要パッケージ |
|-------|--------|----------------|
| [Phase 0](#phase-0-環境構築) | Docker 環境構築 | ROS2 Jazzy, Gazebo |
| [Phase 1](#phase-1-turtlebot3シミュレーションと遠隔操作) | シミュレーションと遠隔操作 | turtlebot3-gazebo, teleop-twist-keyboard |
| [Phase 2](#phase-2-ekfによるローカル自己位置推定) | EKF によるローカル自己位置推定 | robot-localization |
| [Phase 3](#phase-3-amclによるグローバル自己位置推定) | AMCL によるグローバル自己位置推定 | nav2-amcl, nav2-map-server |
| [Phase 4](#phase-4-nav2による自律移動) | Nav2 による自律移動 | navigation2 (nav2-planner, nav2-controller, nav2-bt-navigator) |
| [Phase 5](#phase-5-slam_toolboxによる地図生成) | SLAM による地図生成と自律移動 | slam-toolbox |
| [Phase 6](#phase-6-rest-apiブリッジ) | REST API ブリッジ（HTTP でロボット制御） | FastAPI, uvicorn |
| [Phase 7](#phase-7-open-rmf-fleet-adapter) | Open-RMF Fleet Adapter 統合 | rmf_fleet_adapter_python (EasyFullControl) |

---

## Phase 0: 環境構築

### 概要

ROS2 Jazzy の開発環境をDockerで構築します。
以降のフェーズで使用するGazeboシミュレーション実行基盤を整えます。

### 構成ファイル

```
phase0/
└── Dockerfile
```

### 内容

- ベースイメージ: `osrf/ros:jazzy-desktop`
- GazeboのAPTリポジトリ追加と関連パッケージ (`gz-tools2`, `gz-sim8-cli`) のインストール
- GUIサポート用Qtライブラリのインストール
- コンテナ起動時にROS環境 (`/opt/ros/jazzy/setup.bash`) を自動でsource

### 学習ポイント

- DockerによるROS2開発環境の構築方法
- GazeboとROSの関係
- コンテナ内でのROS環境のセットアップ

---

## Phase 1: TurtleBot3シミュレーションと遠隔操作

### 概要

TurtleBot3 Burgerモデルをシミュレータ上で動かし、キーボードで遠隔操作します。
GazeboとRVizを使ってロボットの動作を確認する基本ワークフローを学びます。

### 構成ファイル

```
phase1/
├── Dockerfile
└── docker-compose.yml
```

### 内容

**Dockerfile:**
- ベースイメージ: `osrf/ros:jazzy-desktop-full`（Gazebo Sim, RViz, チュートリアル同梱）
- 追加パッケージ:
  - `ros-jazzy-turtlebot3-gazebo` — TurtleBot3シミュレーションモデル
  - `ros-jazzy-teleop-twist-keyboard` — キーボード遠隔操作ノード
- 環境変数: `TURTLEBOT3_MODEL=burger`

**docker-compose.yml:**
- ネットワークモード: host（ROS通信を容易にするため）
- GPU対応設定（NVIDIA）
- X11転送でGUI表示

### 主なコマンド

```bash
# Gazebo起動
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# キーボード遠隔操作
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# RViz起動
rviz2
```

### 学習ポイント

- TurtleBot3 Burgerの概要
- Gazeboシミュレーション環境の起動と操作
- `/cmd_vel` トピックとロボット制御の基礎
- Docker ComposeによるGUIアプリのコンテナ起動

---

## Phase 2: EKFによるローカル自己位置推定

### 概要

拡張カルマンフィルタ（EKF）を使って、ホイールオドメトリとIMUを融合し、
精度の高いローカル自己位置（`odom → base_footprint`）を推定します。

### 構成ファイル

```
phase2/
├── Dockerfile
├── docker-compose.yml
└── ws/
    ├── config/
    │   └── ekf.yaml
    └── launch/
        └── ekf.launch.py
```

### 内容

**追加パッケージ:**
- `ros-jazzy-robot-localization` — EKF/UKF実装ライブラリ

**ekf.yaml の設定:**

```yaml
# 基本設定
use_sim_time: true
frequency: 30.0         # 更新周波数 (Hz)
two_d_mode: true        # 平面移動を前提

# フレーム設定
map_frame: map
odom_frame: odom
base_link_frame: base_footprint
world_frame: odom       # EKFはodomフレームを推定

# センサ入力
odom0: /odom            # ホイールオドメトリ
imu0: /imu              # IMU（姿勢・角速度）
```

**docker-compose.yml（3サービス構成）:**

| サービス | 役割 | コマンド |
|----------|------|---------|
| `gazebo` | TurtleBot3シミュレータ | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` |
| `localization` | EKFノード | `ros2 launch /ws/launch/ekf.launch.py` |
| `rviz` | 可視化 | `rviz2` |

### 学習ポイント

- センサフュージョンの基礎（EKFの仕組み）
- TFフレームの概念（`map` / `odom` / `base_footprint`）
- ローカル自己位置推定とグローバル自己位置推定の違い
- `use_sim_time` による時刻同期
- Docker Composeのサービス依存関係（`depends_on`）

---

## Phase 3: AMCLによるグローバル自己位置推定

### 概要

AMCL（Adaptive Monte Carlo Localization）を用いて、
既知の地図上でロボットのグローバルな自己位置を推定します。
EKF（ローカル）とAMCL（グローバル）を組み合わせた完全な自己位置推定スタックを構築します。

### 構成ファイル

```
phase3/
├── Dockerfile
├── docker-compose.yml
└── ws/
    ├── config/
    │   ├── ekf.yaml
    │   ├── amcl.yaml
    │   └── rviz.rviz
    ├── launch/
    │   ├── ekf.launch.py
    │   ├── amcl.launch.py
    │   └── localization.launch.py
    └── maps/
        ├── map.pgm       # 384x384 占有格子地図
        └── map.yaml      # 地図メタデータ（解像度: 0.05m/pixel）
```

### 内容

**追加パッケージ:**
- `ros-jazzy-nav2-amcl` — AMCL自己位置推定
- `ros-jazzy-nav2-map-server` — 静的地図配信
- `ros-jazzy-nav2-lifecycle-manager` — ノードライフサイクル管理
- `ros-jazzy-nav2-rviz-plugins` — AMCLパーティクル可視化プラグイン
- `ros-jazzy-rmw-cyclonedds-cpp` — CycloneDDS（高信頼性ミドルウェア）

**amcl.yaml の主な設定:**

```yaml
# フレーム設定
base_frame_id: "base_footprint"
global_frame_id: "map"
odom_frame_id: "odom"
scan_topic: scan

# パーティクル数（収束の様子を見やすく設定）
min_particles: 200      # 確信度が高い場合に収束
max_particles: 5000     # 不確かな場合に広がる

# 更新頻度
update_min_d: 0.05      # 5cm移動ごとに更新
update_min_a: 0.05      # 約3°回転ごとに更新

# モーションモデル
robot_model_type: "nav2_amcl::DifferentialMotionModel"

# 初期位置（起動引数で上書き可能）
set_initial_pose: true
always_reset_initial_pose: true
```

**localization.launch.py（マスターランチファイル）:**

起動タイミングを制御することで、TFキャッシュに関するエラーを防止します。

```
1. EKF起動（即時）
      ↓ 5秒待機（GazeboのROSクロック・TF確立を待つ）
2. AMCL + MapServer + LifecycleManager起動（15秒後）
      ↓（EKFがTFキャッシュを構築するのを待つ）
3. 全システム稼働
```

**docker-compose.yml（5サービス構成）:**

| サービス | 役割 | 備考 |
|----------|------|------|
| `gazebo` | TurtleBot3シミュレータ | `/odom`, `/imu`, `/scan`, `/tf`, `/clock` を配信 |
| `localization` | EKF + AMCL + MapServer | 初期位置を引数で指定 |
| `rviz` | 可視化 | AMCLパーティクルクラウドを表示 |
| `teleop` | キーボード遠隔操作 | インタラクティブ端末 |

**RMWミドルウェア設定:**
```yaml
RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
```
> FastDDS（デフォルト）はDocker環境で共有メモリの問題が発生する場合があるため、CycloneDDSを使用。

### 実行手順

```bash
# 全サービス起動
docker compose up

# 別ターミナルでロボットを手動操作
docker compose exec teleop bash
# → キーボードで移動させながらRVizでパーティクルの収束を観察
```

### TFフレーム構成

```
map
 └── odom          ← AMCLが配信 (map→odom)
      └── base_footprint  ← EKFが配信 (odom→base_footprint)
```

### 学習ポイント

- パーティクルフィルタ（MCL）によるグローバル自己位置推定の仕組み
- 地図フォーマット（PGM画像 + YAMLメタデータ）
- `map` / `odom` / `base_footprint` の3フレーム構成
- Nav2のライフサイクルノード管理
- ROS2のTFキャッシュと起動タイミング問題の対策
- RMWミドルウェアの選択（CycloneDDS vs FastDDS）

---

## Phase 4: Nav2による自律移動

### 概要

Navigation2（Nav2）スタックを使って、ゴール指定による完全自律移動を実現します。
グローバル経路計画・ローカル制御・障害物回避・リカバリ行動を組み合わせた
実用的なナビゲーションパイプラインを構築します。

### 構成ファイル

```
phase4/
├── Dockerfile
├── docker-compose.yml
└── ws/
    ├── config/
    │   ├── ekf.yaml            (Phase3から継続)
    │   ├── amcl.yaml           (Phase3から継続)
    │   ├── nav2_params.yaml    ★ Nav2完全設定 (新規)
    │   └── rviz.rviz           ★ Nav2表示追加 (更新)
    ├── launch/
    │   ├── ekf.launch.py       (Phase3から継続)
    │   ├── amcl.launch.py      (Phase3から継続)
    │   ├── localization.launch.py (Phase3から継続)
    │   └── navigation.launch.py   ★ Nav2起動 (新規)
    └── maps/
        ├── map.pgm
        └── map.yaml
```

### Nav2 スタック構成

```
RViz "2D Goal Pose"
        ↓ /goal_pose
  bt_navigator          ← ビヘイビアツリーでナビゲーション全体を制御
    ├── planner_server  ← NavFn (A*) でグローバル経路を計算 → /plan
    ├── controller_server ← DWBでローカル速度指令を生成 → /cmd_vel_nav
    ├── smoother_server ← グローバルパスを平滑化
    ├── behavior_server ← 詰まった時のリカバリ (spin/backup/wait)
    └── waypoint_follower ← 複数ウェイポイント追跡
            ↓ /cmd_vel_nav
  velocity_smoother     ← 加速度制限で速度を平滑化
            ↓ /cmd_vel
      Gazebo (TurtleBot3)
```

### nav2_params.yaml の主な設定

**グローバルプランナー (NavFnPlanner / A\*):**
```yaml
planner_server:
  planner_plugins: ["GridBased"]
  GridBased:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    use_astar: true       # A*アルゴリズム (false でDijkstra)
    tolerance: 0.5        # ゴール周辺の許容距離 [m]
    allow_unknown: true   # 未知領域を通る経路を許可
```

> 代替: `nav2_smac_planner/SmacPlanner2d` (差動二輪の運動制約を考慮したA*)

**ローカルコントローラー (DWB):**
```yaml
controller_server:
  controller_frequency: 20.0   # 速度指令生成周波数 [Hz]
  FollowPath:
    plugin: "dwb_core::DWBLocalPlanner"
    max_vel_x: 0.22        # TurtleBot3 Burger最大速度 [m/s]
    max_vel_theta: 1.0     # 最大回転速度 [rad/s]
    sim_time: 1.7          # 軌道シミュレーション時間 [s]
    critics: ["RotateToGoal", "Oscillation", "BaseObstacle",
              "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
```

**コストマップ:**
```yaml
# グローバルコストマップ: 地図全体、静的レイヤー+障害物レイヤー+膨張レイヤー
global_costmap:
  robot_radius: 0.22
  inflation_radius: 0.55  # 障害物周囲の安全距離 [m]

# ローカルコストマップ: ロボット周辺3m×3mのローリングウィンドウ
local_costmap:
  rolling_window: true
  width: 3    # [m]
  height: 3   # [m]
```

### docker-compose.yml（5サービス構成）

| サービス | 役割 | 備考 |
|----------|------|------|
| `gazebo` | TurtleBot3シミュレータ | 変更なし |
| `localization` | EKF + AMCL + MapServer | 変更なし |
| `navigation` | Nav2スタック全体 | **新規** 40秒後に起動 |
| `rviz` | Nav2パネル付き可視化 | Nav2表示追加 |
| `teleop` | キーボード操作 (デバッグ用) | `profiles: [debug]` で通常は起動しない |

> **注意:** teleop は Nav2 と `/cmd_vel` が競合するため通常は無効です。
> デバッグ時は `docker compose run --rm teleop` で起動します。

### 実行手順

```bash
# 全サービス起動
docker compose up

# RVizが起動したら:
# 1. Navigation2パネルの "Startup" ボタンを押す (Nav2が起動済みなら不要)
# 2. ツールバーの "2D Goal Pose" を選択
# 3. 地図上でゴール位置をクリック＆ドラッグで姿勢も指定
# 4. ロボットが自律移動を開始する
```

### RViz の見方

| 表示 | 説明 |
|------|------|
| 薄青い地図 | グローバルコストマップ（障害物膨張コスト） |
| 濃青い地図 | ローカルコストマップ（周辺3m動的障害物） |
| 青い線 | グローバル経路（NavFn A*の計算結果） |
| 緑の線 | ローカル経路（DWBが選択した短期軌道） |
| 点群 | AMCLパーティクルクラウド（自己位置信頼度） |

### 起動タイミングの設計

```
t=0s:  gazebo コンテナ起動 → GazeboシミュレーションとROSノード起動
t=5s:  EKF 起動 → odom→base_footprint TF確立
t=15s: AMCL + MapServer 起動 → map→odom TF確立
t=40s: Nav2スタック起動 (navigation.launch.py内のTimerAction)
       コストマップが /map と TFチェーンを受信して初期化完了
```

### 学習ポイント

- ビヘイビアツリー（BT）によるナビゲーション制御の仕組み
- グローバル経路計画（A*）とローカル制御（DWB）の役割分担
- コストマップの静的レイヤー・障害物レイヤー・膨張レイヤーの重ね合わせ
- 速度スムーザーによる急発進・急停止の防止
- リカバリ行動（spin / backup）による詰まり回避
- RVizの「2D Goal Pose」によるインタラクティブなゴール指定
- Nav2ライフサイクルマネージャーによる起動順序管理

---

## Phase 5: slam_toolboxによる地図生成

### 概要

これまでのフェーズではあらかじめ用意された地図を使って自己位置推定・自律移動を行ってきました。
Phase 5 では **SLAM（Simultaneous Localization and Mapping）** を使い、
ロボット自身が未知の環境を探索しながらリアルタイムで地図を生成します。
生成した地図を保存して、Phase 4 と同じ Nav2 スタックで自律移動に再利用するまでの一連の流れを学びます。

### 構成ファイル

```
phase5/
├── Dockerfile
├── docker-compose.yml
└── ws/
    ├── config/
    │   ├── ekf.yaml            (Phase3から継続)
    │   ├── amcl.yaml           (Phase3から継続)
    │   ├── nav2_params.yaml    (Phase4から継続)
    │   ├── slam.yaml           ★ slam_toolbox 設定 (新規)
    │   ├── rviz.rviz           (Phase4から継続)
    │   └── rviz_slam.rviz      ★ SLAM観察用RViz設定 (新規)
    ├── launch/
    │   ├── ekf.launch.py       (Phase3から継続)
    │   ├── amcl.launch.py      (Phase3から継続)
    │   ├── localization.launch.py (Phase3から継続)
    │   ├── navigation.launch.py   (Phase4から継続)
    │   └── slam.launch.py      ★ SLAMスタック起動 (新規)
    └── maps/
        ├── map.pgm / map.yaml       (Phase3の事前用意地図)
        ├── slam_map.pgm             ★ SLAMで生成・保存する地図
        └── slam_map.yaml
```

### システム構成

```
Gazebo (シミュレーター)
  ├─ /scan (LiDAR)   ──→ slam_toolbox ──→ /map (Occupancy Grid)
  ├─ /odom           ──→ EKF          ──→ odom→base_footprint TF
  ├─ /imu            ──→ EKF          ┘
  └─ /clock

slam_toolbox (Lifecycle ノード)
  ├─ 入力: /scan + odom→base_footprint TF
  ├─ スキャンマッチング → 現在位置推定
  ├─ ループ検出 → 累積誤差を修正 (Ceres Solver)
  ├─ 出力: /map (Occupancy Grid, 5秒ごと更新)
  └─ 出力: map→odom TF (AMCLの代わり)

TFツリー (SLAMフェーズ):
  map ──(slam_toolbox)──→ odom ──(EKF)──→ base_footprint ──→ base_scan
```

> Phase 3/4 では AMCL が `map→odom` TF を発行していましたが、
> SLAM モードでは slam_toolbox 自身が発行するため AMCL は不要です。

### slam.yaml の主な設定

```yaml
slam_toolbox:
  ros__parameters:
    # ソルバー
    solver_plugin: solver_plugins::CeresSolver

    # フレーム・トピック
    scan_topic: /scan
    odom_frame: odom
    base_frame: base_footprint
    map_frame: map
    mode: mapping          # online_async モード

    # 地図設定
    map_update_interval: 5.0   # RViz更新間隔 [s]
    resolution: 0.05           # 1セル = 5cm
    max_laser_range: 3.5       # TurtleBot3 Burger LiDAR 最大距離 [m]

    # スキャンマッチング条件
    minimum_travel_distance: 0.5   # 0.5m 移動ごとにスキャン処理
    minimum_travel_heading: 0.5    # 0.5rad 回転ごとにスキャン処理

    # ループ検出
    do_loop_closing: true
```

### docker-compose.yml（プロファイル構成）

| プロファイル | サービス | 役割 |
|------------|---------|------|
| `slam` | `gazebo` | TurtleBot3シミュレータ |
| `slam` | `slam` | EKF + slam_toolbox + lifecycle_manager |
| `slam` | `rviz_slam` | SLAM観察用RViz（地図生成をリアルタイム確認） |
| `nav` | `gazebo` | TurtleBot3シミュレータ |
| `nav` | `localization` | EKF + AMCL + MapServer（slam_map.yaml を読み込み） |
| `nav` | `navigation` | Nav2スタック全体（Phase4と同一） |
| `nav` | `rviz_nav` | Nav2パネル付きRViz |
| `debug` | `teleop` | キーボード遠隔操作（SLAM探索時に使用） |

### 実行手順

**フェーズ A: SLAMで地図を生成する**

```bash
cd phase5

# SLAMスタック起動
docker compose --profile slam up

# 別ターミナルでテレオペ起動（探索用）
docker compose run --rm teleop
```

起動シーケンス（自動）:

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s | Gazebo 起動 |
| t = 5s | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 10s | slam_toolbox + lifecycle_manager 起動 |
| t = 10s〜 | lifecycle_manager が slam_toolbox を configure → activate |

RViz で白・黒・グレーの地図が表示されることを確認し、ロボットを手動操作して環境を探索します。

| 色 | 意味 |
|----|------|
| 白 | 走行可能な自由空間 |
| 黒 | 壁・障害物 |
| グレー | 未探索領域 |
| 赤い点 | リアルタイムのLiDARスキャン点 |

**フェーズ B: 地図を保存する**

```bash
# コンテナ内で map_saver_cli を実行（ホストマシンには入っていない）
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 run nav2_map_server map_saver_cli -f /ws/maps/slam_map"

# SLAMスタック停止
docker compose --profile slam down
```

成功すると `ws/maps/slam_map.pgm` と `ws/maps/slam_map.yaml` が生成されます。

**フェーズ C: 保存した地図で自律移動する**

```bash
docker compose --profile nav up
```

> **重要:** SLAM 座標系と Gazebo ワールド座標系は異なります。
> Nav2 起動後は RViz の **"2D Pose Estimate"** ツールで初期位置を手動設定し、
> AMCLパーティクルが収束したことを確認してから **"2D Goal Pose"** でゴールを指定してください。

### Phase 3/4 との違い

```
Phase 3/4: 既存の地図 → AMCL で自己位置推定 → Nav2 で自律移動

Phase 5:   未知環境 → SLAM で地図生成 → 地図を保存 → AMCL + Nav2 で自律移動
```

### 学習ポイント

- スキャンマッチング（LiDAR点群の照合）によるSLAMの仕組み
- Occupancy Grid（確率的占有格子地図）の生成原理
- ループ検出とグラフ最適化（Ceres Solver）による累積誤差の修正
- slam_toolboxのLifecycleノード管理（configure→activate）
- `map_saver_cli` による地図ファイル（PGM/YAML）の保存
- SLAM座標系とGazeboワールド座標系の違いへの対処
- Dockerプロファイル（`--profile slam` / `--profile nav`）による起動切り替え

---

## Phase 6: REST API ブリッジ

### 概要

Phase 5 では RViz の GUI でゴールを指定していました。
Phase 6 では **HTTP リクエスト**でロボットを操作できるようにします。
Web アプリ・Python スクリプト・管理システムなど ROS2 を知らない外部システムからも
ロボットを制御できる REST API ブリッジを追加します。

### Phase 5 からの変更点

| ファイル | 変更内容 |
|----------|---------|
| `ws/scripts/web_bridge_node.py` | **新規追加** — FastAPI + ROS2 アクションクライアント |
| `Dockerfile` | `python3-fastapi` / `python3-uvicorn` / `python3-pydantic` を追加 |
| `docker-compose.yml` | `api_bridge` サービスを追加（nav プロファイル） |

### システム構成

```
【Phase 5】
  RViz (GUI)
    └─ 2D Goal Pose ──→ /navigate_to_pose (Action) ──→ ロボット移動

【Phase 6】
  curl / Python / Web アプリ
    └─ POST /move_to ──→ web_bridge_node ──→ /navigate_to_pose (Action) ──→ ロボット移動
                              ↑
                     FastAPI + ROS2 ノード (api_bridge コンテナ)
```

### api_bridge コンテナの内部構造

```
api_bridge コンテナ
  ├─ [メインスレッド] Uvicorn (ポート 8000)
  │     GET  /health   → 死活確認
  │     POST /move_to  → ゴール座標受付
  │
  └─ [バックグラウンドスレッド] ROS2 MultiThreadedExecutor
        web_bridge_node
          └─ ActionClient → /navigate_to_pose → Nav2
```

### 起動手順

```bash
docker compose --profile nav up
```

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s  | Gazebo 起動 |
| t = 5s  | EKF 起動 |
| t = 15s | AMCL + map_server 起動 |
| t = 40s | Nav2 スタック起動 |
| t = 40s | **api_bridge 起動** → ポート 8000 で待機開始 |

### API エンドポイント

```bash
# 死活確認
curl http://localhost:8000/health

# ロボット移動指令
curl -X POST http://localhost:8000/move_to \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 2.0, "yaw": 0.0}'
```

Swagger UI: `http://localhost:8000/docs`

### 学習ポイント

- FastAPI + ROS2 ノードを同一プロセスで共存させるスレッド設計
- `use_sim_time` を使ったシミュレーション時刻との同期
- 非同期ゴール受付（移動完了を待たず即座に `accepted` を返す）パターン

---

## Phase 7: Open-RMF Fleet Adapter

### 概要

Phase 6 の REST API ブリッジに加え、**Open-RMF EasyFullControl Fleet Adapter** を追加します。
RMF Core（rmf_traffic_ros2 / rmf_task_ros2）からのタスク指令を受け取り、
既存の `api_bridge` を介して Nav2 に橋渡しする構成です。

### Phase 6 からの変更点

| 追加ファイル | 内容 |
|------------|------|
| `rmf_adapter/fleet_adapter.py` | EasyFullControl fleet adapter 本体 |
| `rmf_adapter/robot_api_client.py` | api_bridge REST クライアント |
| `rmf_adapter/Dockerfile` | rmf_fleet_adapter_python 入り専用イメージ |
| `rmf_adapter/config/fleet_config.yaml` | fleet・ロボット仕様設定 |
| `rmf_adapter/config/nav_graph.yaml` | RMF ナビゲーショングラフ |
| `docker-compose.yml` | `rmf_adapter` サービスを `rmf` プロファイルで追加 |

### アーキテクチャ

```
[RMF Core]
    ↕ DDS (CycloneDDS)
[rmf_adapter]
  fleet_adapter.py
    ├─ /amcl_pose → updater.update_position() → RMF に位置通知
    └─ navigate() → POST /move_to → ゴール到達で execution.finished()
    ↕ HTTP REST
[api_bridge]  →  Nav2  →  Gazebo
```

### ナビゲーショングラフ（nav_graph.yaml）

| waypoint | 座標 (x, y) | 役割 |
|----------|------------|------|
| `start` | (−2.0, −0.5) | ロボット初期位置 / 充電ポイント |
| `center` | (0.0, −0.5) | センター付近 |
| `east` | (1.5, −0.5) | 東側オープンスペース |
| `north` | (0.0, 1.5) | 北側通路 |

### 起動手順

```bash
xhost +local:root
# nav + RMF を同時起動
docker compose --profile nav --profile rmf up
```

### 学習ポイント

- Open-RMF EasyFullControl API によるフリート管理の仕組み
- rclpy と rmf_fleet_adapter_python（rclcpp）を同一プロセスで共存させる方法
- `/amcl_pose` サブスクライブによる RMF への位置通知
- `execution.finished()` による非同期タスク完了通知
- `_lock` を使ったマルチスレッド安全な状態管理

---

## 技術スタックまとめ

```
ROS2 Jazzy
├── シミュレーション: Gazebo (gz-sim8)
├── ロボットモデル: TurtleBot3 Burger
├── ローカル自己位置推定: robot_localization (EKF)
├── グローバル自己位置推定: nav2_amcl (AMCL)
├── 地図配信: nav2_map_server
├── グローバル経路計画: nav2_navfn_planner (NavFn / A*)
├── ローカル制御: nav2_controller + dwb_core (DWB)
├── 自律移動制御: nav2_bt_navigator (BehaviorTree)
├── 障害物回避: nav2_costmap_2d
├── リカバリ: nav2_behaviors
├── 可視化: RViz2 + nav2_rviz_plugins
├── SLAM: slam_toolbox (async_slam_toolbox_node)
├── REST API ブリッジ: FastAPI + uvicorn (Phase 6〜)
├── フリート管理: Open-RMF rmf_fleet_adapter_python / EasyFullControl (Phase 7)
├── ミドルウェア: CycloneDDS
└── コンテナ化: Docker / Docker Compose
```

## 前提環境

- Docker および Docker Compose がインストール済み
- NVIDIA GPU（オプション、GUI表示に推奨）
- X11サーバー（GUIをホストに転送するため）
