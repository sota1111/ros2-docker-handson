# ROS2 Jazzy ハンズオン

TurtleBot3を使ったROS2 Jazzy入門ハンズオンです。
Phase 0〜4を通じて、シミュレーション環境の構築からグローバル自己位置推定・自律移動までを段階的に学びます。

---

## フェーズ一覧

| Phase | テーマ | 主要パッケージ |
|-------|--------|----------------|
| [Phase 0](#phase-0-環境構築) | Docker環境構築 | ROS2 Jazzy, Gazebo |
| [Phase 1](#phase-1-turtlebot3シミュレーションと遠隔操作) | シミュレーションと遠隔操作 | turtlebot3-gazebo, teleop-twist-keyboard |
| [Phase 2](#phase-2-ekfによるローカル自己位置推定) | EKFによるローカル自己位置推定 | robot-localization |
| [Phase 3](#phase-3-amclによるグローバル自己位置推定) | AMCLによるグローバル自己位置推定 | nav2-amcl, nav2-map-server |
| [Phase 4](#phase-4-nav2による自律移動) | Nav2による自律移動 | navigation2 (nav2-planner, nav2-controller, nav2-bt-navigator) |

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
├── ミドルウェア: CycloneDDS
└── コンテナ化: Docker / Docker Compose
```

## 前提環境

- Docker および Docker Compose がインストール済み
- NVIDIA GPU（オプション、GUI表示に推奨）
- X11サーバー（GUIをホストに転送するため）
