# Phase 5: SLAM による地図生成（slam_toolbox）

## 概要

これまでのフェーズでは「あらかじめ用意された地図」を使って自己位置推定や自律移動を行ってきました。
Phase 5 では **SLAM（Simultaneous Localization and Mapping）** を使い、
ロボット自身が未知の環境を探索しながらリアルタイムで地図を生成します。

生成した地図は保存して Phase 4 と同じ Nav2 スタックで再利用できます。

### このフェーズで学ぶこと

| テーマ | 内容 |
|--------|------|
| SLAM の仕組み | スキャンマッチングでロボットの位置と地図を同時推定する原理 |
| slam_toolbox | ROS 2 標準の SLAM 実装（online_async モード） |
| Occupancy Grid | LiDAR データから確率的に生成される占有格子地図 |
| ループ検出 | 既訪場所の再認識で累積誤差を修正する仕組み |
| 地図の保存と再利用 | `map_saver_cli` で PGM/YAML を保存し Nav2 で再利用する |
| SLAM 座標系 | SLAM が作る座標系と Gazebo ワールド座標系の違い |

### Phase 3/4 との違い

```
Phase 3/4: 既存の地図 → AMCL で自己位置推定 → Nav2 で自律移動
                ↑
           あらかじめ用意が必要

Phase 5:   未知環境 → SLAM で地図生成 → 地図を保存 → Nav2 で自律移動
                ↑
           ロボット自身が地図を作る
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
  ├─ ループ検出 → 累積誤差を修正
  ├─ 出力: /map (Occupancy Grid, 5秒ごと更新)
  └─ 出力: map→odom TF (AMCLの代わり)

lifecycle_manager_slam
  └─ slam_toolbox を configure → activate する
     (activate しないと /scan を購読せず地図を生成しない)

TFツリー (SLAMフェーズ):
  map ──(slam_toolbox)──→ odom ──(EKF)──→ base_footprint ──→ base_scan
```

---

## フェーズ A: SLAM で地図を生成する

### ステップ 1: SLAM スタックを起動する

```bash
cd phase5
docker compose --profile slam up
```

**起動シーケンス（自動）:**

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s | Gazebo、シミュレーター起動 |
| t = 5s | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 10s | slam_toolbox + lifecycle_manager 起動 |
| t = 10s~ | lifecycle_manager が slam_toolbox を configure → activate |

slam_toolbox が正常に起動すると以下のログが出ます:

```
[lifecycle_manager_slam]: Managed nodes are active
[slam_toolbox]: Registering sensor: [Custom Described Lidar]
```

> RViz が開いてから **約 15 秒**で地図の初期領域が表示されます。

### ステップ 2: RViz で SLAM の様子を確認する

RViz (`rviz_slam` コンテナ) が起動したら以下を確認してください。

**表示の意味:**

| 色 | 意味 |
|----|------|
| 白 | 走行可能な自由空間（LiDAR が貫通した = 障害物なし） |
| 黒 | 壁・障害物（LiDAR が反射した） |
| グレー | 未探索領域（まだスキャンが届いていない） |
| 赤い点 | 現在の LiDAR スキャン点（リアルタイム） |

> 起動直後はロボットの周辺だけが白くなり、残りはグレーです。
> ロボットを動かすと白い領域が広がっていきます。

### ステップ 3: ロボットを手動操作して環境を探索する

**別のターミナル**でテレオペを起動します:

```bash
cd phase5
docker compose run --rm teleop
```

キー操作:

| キー | 動作 |
|------|------|
| `i` | 前進 |
| `,` | 後退 |
| `j` | 左回転 |
| `l` | 右回転 |
| `k` | 停止 |
| `q` / `z` | 速度を上げる / 下げる |

**探索のコツ:**

- ゆっくり動くとスキャンマッチング精度が上がる
- 壁に沿って一周すると地図の輪郭が完成する
- 同じ場所に複数の方向から戻るとループ検出が働く
- RViz でグレーの領域がなくなるまで探索する

---

## フェーズ B: 地図を保存する

環境を一周して地図が完成したら保存します。

### ステップ 4: 地図を保存する

SLAM が起動中のまま、**別のターミナル**で以下を実行:

```bash
# コンテナ内で実行する（ホストマシンには nav2_map_server が入っていない）
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 run nav2_map_server map_saver_cli -f /ws/maps/slam_map"
```

成功すると以下の 2 ファイルが作成されます:

```
ws/maps/
  ├── slam_map.pgm   ← 地図画像 (白黒グレースケール)
  └── slam_map.yaml  ← メタデータ (解像度・原点など)
```

YAML ファイルの中身の例:

```yaml
image: slam_map.pgm
mode: trinary
resolution: 0.05        # 1セル = 5cm
origin: [-0.94, -2.06, 0]   # 地図左下隅の座標 (SLAM座標系)
negate: 0
occupied_thresh: 0.65   # この確率以上 → 障害物
free_thresh: 0.25       # この確率以下 → 自由空間
```

### ステップ 5: SLAM を停止する

```bash
docker compose --profile slam down
```

---

## フェーズ C: 保存した地図で自律移動する

### ステップ 6: Nav2 スタックを起動する

```bash
docker compose --profile nav up
```

**起動シーケンス（自動）:**

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s | Gazebo 起動 |
| t = 5s | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 15s | AMCL + map_server 起動（`slam_map.yaml` 読み込み） |
| t = 40s | **Nav2 スタック起動**（コストマップ・プランナー・コントローラ） |

### ステップ 7: RViz で初期位置を設定する

**重要: SLAM 座標系と Gazebo ワールド座標系は異なります**

```
Gazebo ワールド座標: ロボットスポーン位置 = (-2.0, -0.5)
SLAM 座標系:         SLAM開始時の位置を基準に独自の座標を持つ
                     → 両者は一致しない
```

そのため、Nav2 起動後は必ず RViz で初期位置を手動設定してください:

1. RViz (`rviz_nav` コンテナ) が開いたら、地図上でロボットの姿を確認する
2. ツールバーの **"2D Pose Estimate"** をクリック
3. Gazebo でロボットが実際にいる場所に対応する地図上の位置をクリック＆ドラッグして向きを指定
4. AMCL のパーティクル（赤い点群）が収束すれば自己位置推定が成立

### ステップ 8: 自律移動を試す

1. AMCL パーティクルが収束したことを確認
2. **`2D Goal Pose`** ツールでゴールを指定して自律移動を開始

> Phase 4 と同じ操作です。今度は自分で作った地図の上で動きます。

---

## ハンズオン 1: SLAM の仕組みを理解する（スキャンマッチング）

### 概念

slam_toolbox は新しいスキャンが来るたびに以下を行います:

```
新スキャン到着
    ↓
前回スキャンとの差分を計算
    ↓
ロボットの移動量を推定 (スキャンマッチング)
    ↓
推定位置で地図を更新 (Occupancy Grid)
    ↓
map→odom TF を更新
```

この処理は **EKF のオドメトリとは独立して**行われます。
スキャンマッチングは LiDAR だけで位置を推定できるため、
ホイールのスリップがあっても地図精度を保てます。

### 実験: スキャン処理の最小移動量を変えてみる

`ws/config/slam.yaml` を変更します:

```yaml
# デフォルト: 0.5m 移動するたびにスキャン処理
minimum_travel_distance: 0.5  # → 0.1 に下げる

# デフォルト: 0.5rad 回転するたびにスキャン処理
minimum_travel_heading: 0.5   # → 0.1 に下げる
```

変更後に SLAM を再起動:

```bash
docker compose --profile slam down
docker compose --profile slam up
```

**観察:** 値を小さくすると地図の更新頻度が上がり精度が上がる反面、CPU 負荷が増えます。

### SLAM のスキャン処理をリアルタイムで確認する

```bash
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic hz /map"
```

地図が更新されるたびにトピックが発行されます（デフォルト: 5秒間隔）。

---

## ハンズオン 2: ループ検出を理解する

### 概念

SLAM の累積誤差問題:

```
スタート → A → B → C → D → スタートに戻る
                               ↓
                    位置の誤差が積み重なって
                    「スタート地点が 2 か所」になる
                               ↓
                    ループ検出: 「ここはスタートだ」と認識
                    → 累積誤差を一括修正 (グラフ最適化)
```

slam_toolbox はループ検出を **Ceres Solver** でグラフ最適化します。

### ループ検出を観察する

1. ロボットで一周してスタート地点に戻る
2. RViz の地図を観察する
3. ループ検出が成功すると、地図がわずかに「ずれ修正」される瞬間が見える

ターミナルに以下のログが出ればループ検出成功:

```
[slam_toolbox]: Registering loop closure ...
```

### 実験: ループ検出を無効にしてみる

```yaml
# ws/config/slam.yaml
do_loop_closing: false
```

**観察:** 長距離移動後にスタートに戻ると、地図がずれたまま接続されます。
ループ検出の効果がわかります。

---

## ハンズオン 3: 地図の解像度と精度を調整する

### 実験: 解像度を変えてみる

```yaml
# ws/config/slam.yaml
resolution: 0.05   # デフォルト: 5cm/cell
# ↓ より精細に
resolution: 0.02   # 2cm/cell (高精度・高メモリ消費)
# ↓ より粗く
resolution: 0.10   # 10cm/cell (低精度・低メモリ消費)
```

**観察:**
- `0.02` → 壁の角や細い通路が詳細に表現されるが、処理が重くなる
- `0.10` → 処理は軽くなるが、狭い通路が通行不可と判定されることがある

### 地図のメタデータを確認する

```bash
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 topic echo /map --once | head -10"
```

`info.resolution` と `info.width` × `info.height` で地図のサイズが確認できます。

---

## ハンズオン 4: slam_toolbox のセーブ/ロード機能

slam_toolbox は探索の途中経過を保存して再開する機能があります。

### 途中経過を保存する

```bash
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    '{name: {data: /ws/maps/slam_session}}'"
```

### 保存した SLAM セッションから再開する

`ws/config/slam.yaml` を変更してから再起動:

```yaml
# slam.yaml に追加
map_file_name: /ws/maps/slam_session
map_start_at_dock: true
```

> 探索を中断してセッションを保存し、翌日続きから再開できます。

---

## ノードグラフの確認

SLAM スタックが正常に起動しているか確認:

```bash
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 node list"
```

以下が表示されれば正常:

```
/ekf_filter_node         ← odom→base_footprint TF を発行
/slam_toolbox            ← 地図生成・map→odom TF を発行
/lifecycle_manager_slam  ← slam_toolbox を activate した後は待機
```

TF チェーンの確認:

```bash
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 run tf2_tools view_frames"
```

`map → odom → base_footprint → base_scan` の繋がりが確認できます。

---

## トラブルシューティング

### 地図が表示されない（RViz が真っ黒）

slam_toolbox が activate されているか確認します:

```bash
docker compose logs slam | tail -30
```

`Managed nodes are active` が出ていれば正常です。
出ていない場合は slam コンテナを再起動してください:

```bash
docker compose --profile slam down
docker compose --profile slam up
```

### 地図がすぐにずれる・歪む

**原因**: スキャンマッチングが低品質な環境（特徴の少ない廊下など）

対処:
1. ゆっくり動く（`minimum_travel_distance` を大きくして間引く）
2. `link_match_minimum_response_fine` を上げて品質閾値を厳しくする

### map_saver_cli でエラーが出る

`map_saver_cli` はコンテナ内で実行する必要があります（ホストマシンには入っていません）:

```bash
# NG: ホストマシンで実行
ros2 run nav2_map_server map_saver_cli ...

# OK: コンテナ内で実行
docker compose exec slam bash -lc "
  source /opt/ros/jazzy/setup.bash
  ros2 run nav2_map_server map_saver_cli -f /ws/maps/slam_map"
```

### Nav2 フェーズで "Sensor origin is out of map bounds" 警告

```
[global_costmap]: Sensor origin at (-2.03, -0.50) is out of map bounds
```

**原因**: SLAM 座標系と Gazebo ワールド座標系のずれ

SLAM マップの座標系は Gazebo のワールド座標系とは異なります。
ロボットのスポーン位置 (-2.0, -0.5) は Gazebo 座標ですが、
SLAM マップにはその座標が存在しない場合があります。

**対処**: RViz の `2D Pose Estimate` ツールで、
SLAM マップ内のロボットが実際にいる位置に初期位置を設定してください。
AMCL が収束すると警告は自然に解消されます。

### Nav2 フェーズで地図と実環境がずれる

SLAM 中にロボットが速く動きすぎると累積誤差が大きくなります。
もう一度 SLAM で地図を作り直してください。

---

## パラメータ早見表

| パラメータ | デフォルト | 効果 |
|-----------|-----------|------|
| `resolution` | `0.05 m` | 小さくすると精細、大きくすると処理が軽い |
| `map_update_interval` | `5.0 s` | 小さくすると RViz の更新頻度が上がる |
| `minimum_travel_distance` | `0.5 m` | 小さくすると処理頻度が上がり精度↑・CPU↑ |
| `do_loop_closing` | `true` | false にするとループ検出なし |
| `max_laser_range` | `3.5 m` | TurtleBot3 Burger の LiDAR 最大距離 |

---

## まとめ: 学習した内容

```
Phase 5 全体の SLAM パイプライン

  [LiDAR スキャン] ─┐
                     ├→ slam_toolbox ─→ /map (Occupancy Grid)
  [odom TF (EKF)] ──┘       │
                             └→ map→odom TF
                                      │
                               [地図を保存]
                               docker compose exec slam ...
                               map_saver_cli -f /ws/maps/slam_map
                                      │
                                      ↓
                              slam_map.pgm / yaml
                                      │
                               [Nav2 で再利用]
                              --profile nav up
                              + RViz で 2D Pose Estimate
                              + 2D Goal Pose で自律移動
```

| 学習項目 | 対応コンポーネント | 設定ファイル |
|---------|-----------------|-------------|
| スキャンマッチング | `slam_toolbox` (async_slam_toolbox_node) | `slam.yaml` |
| オドメトリ融合 | `ekf_filter_node` | `ekf.yaml` |
| ループ検出・グラフ最適化 | `slam_toolbox` (Ceres Solver) | `slam.yaml` |
| Lifecycle 管理 | `lifecycle_manager_slam` | `slam.launch.py` |
| 地図保存 | `nav2_map_server` (map_saver_cli) | — |
| 地図を使った自律移動 | Nav2 スタック (Phase 4 と同一) | `nav2_params.yaml` |
