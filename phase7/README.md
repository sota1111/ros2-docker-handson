# Phase 7: TurtleBot3 自律移動システム + Open-RMF Fleet Adapter

TurtleBot3 (burger) を Gazebo でシミュレーションし、SLAM による地図生成と Nav2 による自律移動を行うシステムです。
Phase 6 の REST API ブリッジに加え、Phase 7 では **Open-RMF EasyFullControl Fleet Adapter** を追加し、
RMF Core からのタスク指令を Nav2 に橋渡しする構成になっています。

---

## システム全体構成

```
docker compose のプロファイル構成:

  --profile slam        SLAM フェーズ（地図生成）
    gazebo              Gazebo シミュレーター
    slam                EKF + slam_toolbox（地図生成）
    rviz_slam           RViz（地図生成の観察）

  --profile nav         ナビゲーションフェーズ（自律移動）
    gazebo              Gazebo シミュレーター
    localization        EKF + AMCL + map_server（自己位置推定）
    navigation          Nav2 スタック（経路計画・自律移動）
    rviz_nav            RViz（ナビゲーションの観察）
    api_bridge          FastAPI REST ブリッジ（HTTP 経由の移動指令）

  --profile rmf         Open-RMF Fleet Adapter（nav と併用）
    rmf_adapter         EasyFullControl fleet adapter

  --profile debug       （手動操作用、単独使用）
    teleop              キーボードテレオペ
```

### アーキテクチャ概観（Phase 7）

```
[RMF Core (rmf_traffic_ros2 / rmf_task_ros2)]
        ↕  DDS (CycloneDDS, ROS_DOMAIN_ID=0)
[rmf_adapter コンテナ]
  fleet_adapter.py
    ├─ /amcl_pose サブスクライブ → RMF に位置通知 (updater.update_position)
    └─ navigate() コールバック  → POST /move_to → ゴール到達で execution.finished()
        ↕  HTTP REST (localhost:8000)
[api_bridge コンテナ]
  web_bridge_node.py
        ↕  ROS2 ActionClient (/navigate_to_pose)
[navigation コンテナ]
  Nav2 bt_navigator → /cmd_vel → Gazebo
```

---

## フェーズ A: SLAM による地図生成

### 起動

```bash
xhost +local:root
docker compose --profile slam up
```

### 起動シーケンス

| 経過時間 | 起動内容 |
|---------|---------|
| t = 0s  | Gazebo 起動（`/clock` / `/scan` / `/odom` / `/imu` 発行開始） |
| t = 5s  | `ekf_filter_node` 起動 → `odom → base_footprint` TF 発行開始 |
| t = 10s | `async_slam_toolbox_node` + `lifecycle_manager_slam` 起動 → 地図生成開始 |

### TF ツリー（SLAM フェーズ）

```
map ──(slam_toolbox)──→ odom ──(EKF)──→ base_footprint ──→ base_scan
```

- **EKF** (`robot_localization`): `/odom` + `/imu` を融合して `odom → base_footprint` TF を 30Hz で発行
- **slam_toolbox**: LiDAR スキャンと `odom` TF からスキャンマッチングで位置推定し、`/map` トピックと `map → odom` TF を発行

### 地図の保存

RViz でロボットを走行させながら地図を生成したら、以下で保存します:

```bash
docker exec slam bash -lc \
  "source /opt/ros/jazzy/setup.bash && \
   ros2 run nav2_map_server map_saver_cli -f /ws/maps/map"
```

保存されるファイル:
- `ws/maps/map.pgm` — 画素値で障害物/空間を表すグレースケール画像（解像度 0.05m/px、範囲 20×20m）
- `ws/maps/map.yaml` — 地図のメタデータ（解像度・原点・閾値）

---

## フェーズ B: Nav2 による自律移動

### 起動

```bash
xhost +local:root
docker compose --profile nav up
```

### 起動シーケンス

| 経過時間 | コンテナ | 起動内容 |
|---------|---------|---------|
| t = 0s  | gazebo | Gazebo 起動 |
| t = 5s  | localization | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 15s | localization | AMCL + map_server 起動 → `map → odom` TF 確立 |
| t = 40s | navigation | Nav2 全ノード起動 |
| t = 40s | api_bridge | FastAPI サーバー起動（ポート 8000） |

### 自己位置推定（localization コンテナ）

`localization.launch.py` が以下を起動します:

| ノード | 役割 |
|--------|------|
| `ekf_filter_node` | `/odom` + `/imu` 融合 → `odom → base_footprint` TF |
| `map_server` | 保存済み地図 (`map.yaml`) を `/map` トピックで配信 |
| `amcl` | パーティクルフィルタで `map → odom` TF を推定 |
| `lifecycle_manager_localization` | map_server → amcl の順に activate |

AMCL の初期位置は docker-compose で `init_x:=-2.0 init_y:=-0.5 init_yaw:=0.0` として渡されます。
起動後は RViz の **"2D Pose Estimate"** ツールで正確な初期位置を設定してください。

#### AMCL パラメータのポイント（`config/amcl.yaml`）

| パラメータ | 値 | 理由 |
|-----------|-----|------|
| `min_particles` / `max_particles` | 200 / 5000 | 散布→収束の変化を視覚的に確認しやすくする |
| `alpha1`〜`alpha5` | 0.1 | 運動ノイズを小さくして収束を速める |
| `laser_z_hit` | 0.9 | スキャン一致度を強く効かせて収束を促進 |
| `transform_tolerance` | 2.0 | 起動直後の TF タイムスタンプのずれを吸収 |

#### EKF パラメータのポイント（`config/ekf.yaml`）

| パラメータ | 値 | 理由 |
|-----------|-----|------|
| `transform_time_offset` | 0.0 | 0 以外にすると TF タイムスタンプが未来にずれ、AMCL が全スキャンを破棄する |
| `two_d_mode` | true | 平面移動のみのため Z/Roll/Pitch を無視 |
| `frequency` | 30.0 Hz | 十分な TF 更新頻度 |

### Nav2 スタック（navigation コンテナ）

`navigation.launch.py` が 40 秒待機後に以下を起動します:

| ノード | 役割 |
|--------|------|
| `planner_server` | NavFn (A*) によるグローバルパス計画 |
| `controller_server` | DWB ローカルプランナーによる速度指令生成 |
| `smoother_server` | グローバルパスの平滑化 |
| `behavior_server` | リカバリ行動（spin / backup / wait） |
| `bt_navigator` | ビヘイビアツリーでプランナー・コントローラ・リカバリを統合 |
| `waypoint_follower` | 複数ウェイポイントの順次追跡 |
| `velocity_smoother` | 加速度制限を適用した速度指令の平滑化 |
| `lifecycle_manager_navigation` | 上記ノードを順番に activate |

#### 速度指令のフロー

```
controller_server ─┐
                   ├──→ /cmd_vel_nav ──→ velocity_smoother ──→ /cmd_vel ──→ Gazebo
behavior_server   ─┘
```

### RViz でゴール指定（手動操作）

1. RViz の **"2D Goal Pose"** ツールをクリック
2. 地図上の目標位置をクリック＆ドラッグで指定
3. ロボットが自律移動を開始

---

## REST API ブリッジ（api_bridge コンテナ）

`web_bridge_node.py` が FastAPI + ROS2 ノードを同一プロセスで動作させます。

### 内部構造

```
api_bridge コンテナ
  ├─ [メインスレッド] Uvicorn (ポート 8000)
  │     GET  /health   → 死活確認
  │     POST /move_to  → ゴール座標受付
  │
  └─ [バックグラウンドスレッド] ROS2 MultiThreadedExecutor
        WebBridgeNode
          └─ ActionClient → /navigate_to_pose → Nav2 bt_navigator
```

ROS2 エグゼキューターをバックグラウンドスレッドで動かし、Uvicorn をメインスレッドで動かすことで、両者を同一プロセスで共存させています。

### エンドポイント

#### GET /health

```bash
curl http://localhost:8000/health
```

```json
{"status": "ok", "ros2_node": true}
```

#### POST /move_to

ロボットを指定座標へ移動させます。ゴール受付時点で即座にレスポンスを返し、移動完了は待ちません。

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `x` | float | ✓ | マップ座標系 X 位置 [m] |
| `y` | float | ✓ | マップ座標系 Y 位置 [m] |
| `yaw` | float | — | 目標姿勢 Z 軸回転 [rad]、デフォルト 0.0 |

```bash
curl -X POST http://localhost:8000/move_to \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 2.0, "yaw": 0.0}'
```

```json
{"status": "accepted", "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0}, "message": "ゴールを受け付けました。ロボットが移動を開始します。"}
```

**yaw の向き:**

```
yaw = 0.0     → +X 方向（東）
yaw = 1.5708  → +Y 方向（北）（π/2 rad）
yaw = 3.1416  → -X 方向（西）（π rad）
yaw = -1.5708 → -Y 方向（南）（-π/2 rad）
```

**エラーレスポンス (503):** Nav2 がまだ起動していない場合

```json
{"detail": "NavigateToPose アクションサーバーに接続できません。Nav2 の起動を確認してください。"}
```

### Swagger UI

```
http://localhost:8000/docs
```

---

## Open-RMF Fleet Adapter（rmf_adapter コンテナ）

Phase 7 で追加。Open-RMF EasyFullControl API を使って、RMF Core からのタスク指令を受け取り、
既存の `api_bridge` 経由で Nav2 に橋渡しします。

### 起動（nav プロファイルと併用）

```bash
xhost +local:root
docker compose --profile nav --profile rmf up
```

`rmf_adapter` は `api_bridge` が起動してから起動します（`depends_on: api_bridge`）。
起動時に `GET /health` で api_bridge の準備完了を最大 90 秒待機します。

### 構成ファイル

```
rmf_adapter/
  fleet_adapter.py          EasyFullControl fleet adapter 本体
  robot_api_client.py       api_bridge REST クライアント
  Dockerfile                rmf_fleet_adapter_python 入り専用イメージ
  config/
    fleet_config.yaml       fleet・ロボット仕様設定
    nav_graph.yaml          RMF ナビゲーショングラフ（waypoint / lane 定義）
```

### fleet_adapter.py の構造

```
main()
  ├─ fleet_config.yaml / nav_graph.yaml 読み込み
  ├─ rmf_adapter.graph.Graph 構築（waypoint + lane）
  ├─ VehicleTraits 設定（速度・加速度・フットプリント）
  ├─ BatterySystem 設定（ダミー値、消費量計算は無効）
  ├─ EasyFullControl.make(fleet_config) → RMF executor をバックグラウンドで spin 開始
  ├─ fleet.add_robot(initial_state, navigate, stop, dock, action_executor, on_update)
  ├─ /amcl_pose サブスクライバー作成
  └─ rclpy MultiThreadedExecutor.spin()（メインスレッド）
```

**RobotAdapter クラス**（1 台のロボット状態を管理）:

| メソッド | 呼び出し元 | 処理 |
|---------|-----------|------|
| `on_amcl_pose(x, y, yaw)` | rclpy callback | `updater.update_position()` で RMF に位置通知。到達判定も実施 |
| `navigate(destination, execution)` | RMF 内部スレッド | `POST /move_to` でロボットに移動指令を送る |
| `stop(activity)` | RMF 内部スレッド | 内部フラグのみリセット（Nav2 への停止送信は未実装） |
| `dock(dock_name, execution)` | RMF 内部スレッド | 未実装。即座に `execution.finished()` |
| `on_update(updater)` | EasyFullControl | updater ハンドルを保存する |

### ナビゲーショングラフ（`config/nav_graph.yaml`）

TurtleBot3 World の既知オープンスペースに 4 つの waypoint を定義しています。

| waypoint | 座標 (x, y) | 役割 |
|----------|------------|------|
| `start` | (−2.0, −0.5) | ロボット初期位置 / 充電ポイント |
| `center` | (0.0, −0.5) | センター付近（障害物の隙間） |
| `east` | (1.5, −0.5) | 東側オープンスペース |
| `north` | (0.0, 1.5) | 障害物の北側通路 |

レーン構成（すべて双方向）:

```
start ↔ center ↔ east
  ↕         ↕
  └── north ─┘
```

### fleet_config.yaml の主要設定

| 設定項目 | 値 | 説明 |
|---------|-----|------|
| `fleet_name` | `turtlebot3_fleet` | RMF 上の fleet 識別子 |
| `robot_name` | `turtlebot3` | fleet 内のロボット名 |
| `map_name` | `L1` | RMF のレベル名（nav_graph.yaml と一致） |
| `profile.footprint` | 0.18 m | ロボット外形半径（衝突検知用） |
| `limits.linear.velocity` | 0.22 m/s | TurtleBot3 burger 最大速度 |
| `arrival_threshold` | 0.35 m | `/amcl_pose` とゴールの距離がこれ以下で「到着」判定 |
| `initial_waypoint` | `start` | 起動時の初期 waypoint |
| `api.base_url` | `http://localhost:8000` | api_bridge のベース URL |

### スレッドモデル

| スレッド | 担当 |
|---------|------|
| メインスレッド（rclpy） | `/amcl_pose` サブスクライブ、`on_amcl_pose` コールバック処理 |
| RMF バックグラウンドスレッド | EasyFullControl の内部 executor（navigate/stop/dock コールバック） |

`_lock` で共有状態（`_position`, `_navigating`, `_goal`, `_execution`）を保護しています。

### 未実装事項

- Nav2 への停止コマンド送信（`stop()` は内部フラグのリセットのみ）
- バッテリー実測（固定値 1.0 = 100%）
- ドッキング・充電・清掃・搬送タスク
- 複数ロボット対応
- 障害復旧・再計画

---

## 停止

```bash
docker compose --profile nav --profile rmf down
# RMF を使わない場合
docker compose --profile nav down
# または
docker compose --profile slam down
```

オーファンコンテナが残っている場合:

```bash
docker compose --profile nav --profile rmf down --remove-orphans
```

---

## トラブルシューティング

### AMCL が収束しない（ロボットが動かない）

RViz の **"2D Pose Estimate"** で初期位置を再設定してください。
赤いパーティクル群が一点に収束すれば自己位置推定完了です。

### `503: NavigateToPose アクションサーバーに接続できません`

Nav2 はまだ起動中です。起動から 40 秒以上待ってから再試行してください。

```bash
docker compose logs navigation | tail -20
```

### rmf_adapter が api_bridge に接続できない

`fleet_adapter.py` は起動時に `GET /health` を 30 回（3 秒間隔、最大 90 秒）リトライします。
Nav プロファイルが先に起動していることを確認してください。

```bash
docker compose logs rmf_adapter | tail -30
```

### rmf_adapter のログ確認

```bash
docker compose logs rmf_adapter
```

### api_bridge のログ確認

```bash
docker compose logs api_bridge
```

### コンテナ名の競合

```bash
docker rm -f api_bridge rmf_adapter
docker compose --profile nav --profile rmf up
```
