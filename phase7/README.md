# Phase 7: TurtleBot3 自律移動システム

TurtleBot3 (burger) を Gazebo でシミュレーションし、SLAM による地図生成と Nav2 による自律移動を行うシステムです。
外部から HTTP 経由でロボットに移動指令を送る REST API ブリッジも含みます。

---

## システム全体構成

```
docker compose のプロファイル構成:

  --profile slam   SLAM フェーズ（地図生成）
    gazebo         Gazebo シミュレーター
    slam           EKF + slam_toolbox（地図生成）
    rviz_slam      RViz（地図生成の観察）

  --profile nav    ナビゲーションフェーズ（自律移動）
    gazebo         Gazebo シミュレーター
    localization   EKF + AMCL + map_server（自己位置推定）
    navigation     Nav2 スタック（経路計画・自律移動）
    rviz_nav       RViz（ナビゲーションの観察）
    api_bridge     FastAPI REST ブリッジ（HTTP 経由の移動指令）

  --profile debug  （手動操作用、単独使用）
    teleop         キーボードテレオペ
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

## 停止

```bash
docker compose --profile nav down
# または
docker compose --profile slam down
```

オーファンコンテナが残っている場合:

```bash
docker compose --profile nav down --remove-orphans
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

### api_bridge のログ確認

```bash
docker compose logs api_bridge
```

### コンテナ名の競合

```bash
docker rm -f api_bridge
docker compose --profile nav up
```
