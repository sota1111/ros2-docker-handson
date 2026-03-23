# Phase 4: Nav2 による自律移動

## 概要

Phase 3 で構築した自己位置推定（EKF + AMCL）の上に、
**Navigation2（Nav2）スタック**を追加して完全自律移動を実現します。

### このフェーズで学ぶこと

| テーマ | 内容 |
|--------|------|
| グローバル経路計画 | NavFn (A*) で地図全体のルートを計算する仕組み |
| ローカル制御 | DWB コントローラがリアルタイムに速度指令を生成する仕組み |
| 障害物回避 | コストマップの層構造と膨張半径による安全距離の確保 |
| ゴール指定による自律移動 | RViz からゴールを与えてロボットを自律走行させる |

### システム構成

```
Gazebo (シミュレーター)
  ├─ /scan (LiDAR)         ─┬→ global_costmap (障害物レイヤー)
  ├─ /odom (ホイール)       │  → local_costmap  (障害物レイヤー)
  ├─ /imu                   │
  └─ /clock                 │
                             │
EKF   → odom→base TF  ──────┤
AMCL  → map→odom TF   ──────┤
                             │
RViz "2D Goal Pose"          │
  └→ /goal_pose              │
       └→ bt_navigator       │
            ├→ planner_server (NavFn/A*)  → /plan (青い経路)
            │    └─ global_costmap ←──────┘
            ├→ controller_server (DWB)    → /cmd_vel_nav
            │    └─ local_costmap  ←──────┘
            └→ behavior_server (spin/backup 等)
                    ↓
            velocity_smoother → /cmd_vel → Gazebo
```

---

## 前提条件

- Phase 3 が動作すること（EKF + AMCL による自己位置推定）
- Docker / Docker Compose がインストール済みであること
- X11 転送の設定が完了していること

---

## ステップ 1: 起動

```bash
cd phase4
docker compose up
```

**起動シーケンス（自動）:**

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s | Gazebo、シミュレーター起動 |
| t = 5s | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 15s | AMCL + map_server 起動 → `map → odom` TF 確立 |
| t = 40s | **Nav2 スタック起動**（コストマップ・プランナー・コントローラ） |

> RViz が開いてから **約 40〜50 秒**でナビゲーションが使用可能になります。

Nav2 の準備ができると、`navigation` コンテナのログに以下が表示されます:

```
[lifecycle_manager_navigation] Managed nodes are active
```

---

## ステップ 2: RViz の確認

RViz が開いたら、以下の表示が出ていることを確認してください。

### 確認すべき表示

**1. 地図（静的）**
白い領域 = 走行可能、黒い領域 = 壁・障害物、グレー = 未知領域

**2. AMCLパーティクルクラウド**
赤い点群がロボット周辺に集まっていれば自己位置推定が成立しています。
収束していない場合は `2D Pose Estimate` ツールで初期位置を修正してください。

**3. グローバルコストマップ**（薄い色の重ね合わせ）
壁の周囲に色のグラデーションが見えます。これが「膨張コスト」です。

**4. ローカルコストマップ**（ロボット中心の小さな矩形）
ロボット周辺 3m × 3m の範囲にリアルタイムで更新される障害物マップです。

> 表示されない項目は RViz 左パネルの Displays リストで **Enabled** にチェックを入れてください。

---

## ステップ 3: 初めてのゴール指定（自律移動）

### 手順

1. RViz ツールバーの **"2D Goal Pose"** ボタンをクリック
2. 地図上の**障害物のない空きスペース**をクリックし、そのままドラッグして向きを指定
3. マウスを離すとゴールが送信されます

### 観察ポイント

- **青い線（GlobalPath）**: NavFn が計算したグローバル経路が表示される
- **緑の線（LocalPath）**: DWB が選択した短期軌道が追従して動く
- ロボットが経路に沿って自律移動を始める
- ゴール付近に到達すると停止する

### うまくいかない場合

| 症状 | 対処 |
|------|------|
| 青い経路が表示されない | AMCLパーティクルが収束しているか確認。`2D Pose Estimate` で位置を修正 |
| ロボットが動かない | `navigation` コンテナのログで `Managed nodes are active` を確認 |
| 経路が壁を通り抜ける | コストマップが初期化されていない可能性。数秒待ってから再試行 |

---

## ハンズオン 1: グローバル経路計画を理解する（NavFn / A*）

### 概念

グローバルプランナーは「地図全体」を見て、スタートからゴールまでの**最短経路**を計算します。
本フェーズでは **NavFn（Navigation Function）** を使用しています。

```
NavFn = Dijkstra または A* アルゴリズムを格子地図上で実行
        ↓
  コストマップのコスト値を「通行コスト」として経路を探索
  障害物に近いセルはコストが高い → 自然と安全な経路が選ばれる
```

### 実験: A* と Dijkstra を切り替えてみる

`ws/config/nav2_params.yaml` の以下の行を変更します:

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  use_astar: true    # ← false にすると Dijkstra になる
```

変更後はコンテナを再起動してゴールを指定し直します:

```bash
docker compose restart navigation
```

**観察:** `use_astar: true` の方が一般的に速く経路が計算されます。
Dijkstra（`false`）は全方向を均等に探索するため、複雑な環境では遅くなります。

### 実験: SmacPlanner2d に切り替えてみる

NavFn は差動二輪の運動制約（最小回転半径など）を考慮しません。
`SmacPlanner2d` はより実用的な格子ベースプランナーです。

```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner2d"
  tolerance: 0.5
  allow_unknown: true
  max_iterations: 1000000
  max_planning_time: 5.0
  use_final_approach_orientation: false
  smoother:
    max_iterations: 1000
    w_smooth: 0.3
    w_data: 0.2
    tolerance: 1.0e-10
```

**観察:** SmacPlanner2d は角の切り取りがより滑らかです。壁際の経路形状の違いを比較してみてください。

### 経路計画のトピックを確認する

```bash
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /plan --once"
```

`geometry_msgs/msg/PoseStamped` の配列として経路点が出力されます。

---

## ハンズオン 2: ローカル制御を理解する（DWB コントローラー）

### 概念

ローカルコントローラーはグローバル経路に沿いながら、**リアルタイムの障害物を回避**しつつ速度指令を生成します。

```
DWB (Dynamic Window Based):
  1. ロボットの現在速度と加速度制限から「実現可能な速度の組み合わせ」をサンプリング
  2. 各速度で sim_time [s] 先まで軌道をシミュレーション
  3. 複数の評価関数（Critics）でスコアリング
  4. 最高スコアの軌道の速度を /cmd_vel として出力
```

### DWBのCriticsを理解する

`nav2_params.yaml` に定義された評価関数:

| Critic | 役割 |
|--------|------|
| `PathDist` | グローバル経路からの距離を最小化（経路追従） |
| `GoalDist` | ゴールまでの距離を最小化（ゴール接近） |
| `PathAlign` | ロボットの向きをパスの接線方向に合わせる |
| `GoalAlign` | ゴールの向きに合わせる |
| `BaseObstacle` | 障害物への距離を最大化（衝突回避） |
| `Oscillation` | 前後の振動を抑制 |
| `RotateToGoal` | ゴール付近でその場回転して姿勢を合わせる |

### 実験: 速度の上限を変えてみる

```yaml
FollowPath:
  max_vel_x: 0.10    # ← 0.22 → 0.10 に下げる（半速）
  max_vel_theta: 0.5 # ← 1.0 → 0.5 に下げる
```

**観察:** ゆっくり走行し、コーナーでも安定した追従が見られます。

### 実験: sim_time を変えてみる

```yaml
FollowPath:
  sim_time: 3.0    # ← 1.7 → 3.0 に延ばす
```

**観察:** 先読み時間が増えるため、障害物の手前でより早めに回避行動を始めます。
ただし計算コストが上がり、反応が鈍くなる場合もあります。

### ローカルパスをリアルタイムで観察する

RViz で **LocalPath**（緑の線）を表示した状態でロボットを走行させると、
ロボットの前方に短い弧状の軌道が追従して更新されるのが見えます。
これが DWB が「今この瞬間に選んだ軌道」です。

### 制御周波数を確認する

```bash
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic hz /cmd_vel"
```

`controller_frequency: 20.0` の設定通り、約 20 Hz で速度指令が出ているはずです。

---

## ハンズオン 3: コストマップと障害物回避を理解する

### コストマップの層構造

コストマップは複数のレイヤーが重なって構成されています:

```
グローバルコストマップ (map フレーム、固定)
  ├── StaticLayer   : /map の壁情報を 0/255 で書き込む
  ├── ObstacleLayer : LiDARスキャンから動的障害物を書き込む
  └── InflationLayer: 障害物の周囲に「コストの霧」を広げる
                      → 壁に近づくほどコストが高くなる

ローカルコストマップ (odom フレーム、ローリング)
  ├── VoxelLayer    : LiDARスキャンから動的障害物を書き込む
  └── InflationLayer: 同上
```

### 膨張レイヤーの可視化

RViz でコストマップを見ると、壁の周囲に**グラデーション**が見えます。
これが InflationLayer の効果です。

```yaml
inflation_layer:
  inflation_radius: 0.55    # ← 膨張半径 [m]
  cost_scaling_factor: 3.0  # ← コストの減衰率
```

- `inflation_radius` が大きいほど壁から遠くを迂回する経路が選ばれる
- `cost_scaling_factor` が大きいほど膨張コストが急激に下がり、壁スレスレを通りやすくなる

### 実験: inflation_radius を変えてみる

```yaml
# global_costmap と local_costmap の両方を変更する
inflation_layer:
  inflation_radius: 1.0   # ← 0.55 → 1.0 に増やす
```

**観察:** 経路が壁から大きく離れるようになります。
狭い通路では経路が見つからなくなる場合があります。

### 実験: 動的障害物回避を試す

1. Gazebo を開き、テーブルなどのオブジェクトを経路上に配置する
2. RViz の LocalCostmap に新しい障害物が表示されるのを確認
3. 走行中のロボットが自動的に迂回するのを観察する

> Gazebo GUI でオブジェクトを配置: `Insert` タブからモデルを選択して配置

### コストマップをコマンドで確認する

```bash
# グローバルコストマップの情報
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /global_costmap/costmap_updates --once"

# ローカルコストマップの情報
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /local_costmap/costmap_updates --once"
```

---

## ハンズオン 4: ゴール指定ナビゲーションの応用

### 複数ウェイポイントを CLI で送る

複数のゴールを順番に経由させることができます。

```bash
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'"
```

> `x`, `y` の値は地図座標系で指定します。RViz の Publish Point ツールで確認できます。

### ナビゲーション状態を監視する

```bash
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /navigate_to_pose/_action/status"
```

| status | 意味 |
|--------|------|
| `STATUS_ACCEPTED` | ゴール受付 |
| `STATUS_EXECUTING` | 走行中 |
| `STATUS_SUCCEEDED` | 到達完了 |
| `STATUS_ABORTED` | 失敗（経路なし・スタック等） |

### リカバリ行動を観察する

ロボットが障害物に近づきすぎたり、経路計画が失敗すると
**behavior_server** のリカバリ行動が自動実行されます。

| リカバリ行動 | 動作 |
|----------|------|
| `Spin` | その場でゆっくり回転してセンサ情報を更新 |
| `BackUp` | 少し後退して障害物から離れる |
| `Wait` | 一定時間待機して状況が変わるのを待つ |

リカバリ実行中は RViz の **Navigation 2** パネルに状態が表示されます。

---

## ノードグラフの確認

全ノードが正常に起動しているか確認します:

```bash
docker compose exec navigation bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 node list"
```

以下のノードが表示されれば正常です:

```
/bt_navigator
/controller_server
/smoother_server
/planner_server
/behavior_server
/waypoint_follower
/velocity_smoother
/lifecycle_manager_navigation
/map_server          ← localization コンテナから見える
/amcl                ← localization コンテナから見える
/ekf_filter_node     ← localization コンテナから見える
```

---

## トラブルシューティング

### 「Could not find a plan」エラー

経路計画に失敗した場合:

1. **コストマップが初期化されているか確認**
   ```bash
   docker compose exec navigation bash -lc "
     source /opt/ros/jazzy/setup.bash
     ros2 topic echo /global_costmap/costmap --once | head -5"
   ```
2. **ゴール位置が障害物上にないか確認** — 壁の上や膨張コスト領域内はNGです
3. **ゴールのコスト確認** — `inflation_radius` を小さくすると経路が見つかりやすくなります

### ロボットが経路から大きく外れる

AMCLの自己位置推定がずれている可能性があります:

1. RViz の **ParticleCloud** が広がりすぎていないか確認
2. `2D Pose Estimate` ツールでロボットの正確な位置・向きを手動設定
3. ゆっくり動かして AMCL を再収束させる

### Nav2 が起動しない（lifecycle エラー）

```bash
# localization コンテナのログを確認
docker compose logs localization | tail -30

# navigation コンテナのログを確認
docker compose logs navigation | tail -30
```

TFが存在しない場合は起動タイミングの問題です。
`navigation.launch.py` の `TimerAction(period=40.0, ...)` の値を増やしてください。

---

## パラメータ早見表

| パラメータ | デフォルト | 効果 |
|-----------|-----------|------|
| `use_astar` | `true` | `false` で Dijkstra に切替 |
| `inflation_radius` | `0.55 m` | 大きくすると壁から遠ざかる |
| `max_vel_x` | `0.22 m/s` | 最大前進速度 |
| `max_vel_theta` | `1.0 rad/s` | 最大回転速度 |
| `sim_time` | `1.7 s` | DWB の先読み時間 |
| `xy_goal_tolerance` | `0.25 m` | ゴール到達判定の位置誤差 |
| `yaw_goal_tolerance` | `0.25 rad` | ゴール到達判定の角度誤差 |

---

## まとめ: 学習した内容

```
Phase 4 全体の自律移動パイプライン

  [地図] ──→ グローバルコストマップ ──→ NavFn (A*)
                                          │
                                          ↓ /plan (グローバル経路)
  [LiDAR] ─→ ローカルコストマップ ──→ DWB コントローラー
                                          │
                                          ↓ /cmd_vel_nav
                                   velocity_smoother
                                          │
                                          ↓ /cmd_vel
                                     TurtleBot3
```

| 学習項目 | 対応コンポーネント | 設定ファイル |
|---------|-----------------|-------------|
| グローバル経路計画 (A*) | `planner_server` (NavFnPlanner) | `nav2_params.yaml` |
| ローカル制御 (DWB) | `controller_server` (DWBLocalPlanner) | `nav2_params.yaml` |
| 障害物回避 | `global/local_costmap` + InflationLayer | `nav2_params.yaml` |
| ゴール指定ナビゲーション | `bt_navigator` + RViz SetGoal | `rviz.rviz` |
| リカバリ行動 | `behavior_server` | `nav2_params.yaml` |
