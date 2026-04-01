# Phase 6: REST API ブリッジ（外部システムからロボットを HTTP で制御）

## Phase 5 からの変更点

Phase 5 では RViz の GUI でゴールを指定していました。
Phase 6 では **HTTP リクエスト** でロボットを操作できるようにします。
これにより、Web アプリ・Python スクリプト・管理システムなど ROS2 を知らない外部システムからもロボットを制御できます。

### 追加・変更したファイル

| ファイル | 変更内容 |
|----------|---------|
| `ws/scripts/web_bridge_node.py` | **新規追加** — FastAPI + ROS2 アクションクライアント |
| `Dockerfile` | `python3-fastapi` / `python3-uvicorn` / `python3-pydantic` を追加 |
| `docker-compose.yml` | `api_bridge` サービスを追加（nav プロファイル） |

### Phase 5 との構成の違い

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

Phase 5 の RViz による操作は引き続き利用できます。Phase 6 はそこに HTTP 経由の操作経路を追加したものです。

---

## システム構成

```
docker compose --profile nav up で起動するコンテナ:

  gazebo        Gazebo シミュレーター（ロボット・センサー）
  localization  EKF + AMCL + map_server（自己位置推定）
  navigation    Nav2 スタック（経路計画・制御）
  rviz_nav      RViz（可視化・手動ゴール指定）
  api_bridge    FastAPI サーバー ★ Phase 6 で追加
```

### api_bridge コンテナの内部構造

```
api_bridge コンテナ
  ├─ [メインスレッド] Uvicorn (ポート 8000)
  │     GET  /health   → 死活確認
  │     POST /move_to  → ゴール座標を受け取る
  │
  └─ [バックグラウンドスレッド] ROS2 MultiThreadedExecutor
        web_bridge_node
          └─ ActionClient → navigate_to_pose → Nav2
```

`POST /move_to` を受け取ると、`NavigateToPose` アクションゴールを Nav2 に送信して即座に `accepted` を返します（移動完了は待ちません）。

---

## 起動手順

### ステップ 1: 起動

```bash
docker compose --profile nav up
```

**起動シーケンス:**

| 経過時間 | 起動内容 |
|----------|---------|
| t = 0s  | Gazebo 起動 |
| t = 5s  | EKF 起動 → `odom → base_footprint` TF 確立 |
| t = 15s | AMCL + map_server 起動（`slam_map.yaml` 読み込み） |
| t = 40s | Nav2 スタック起動 |
| t = 40s | **api_bridge 起動** → ポート 8000 で待機開始 |

### ステップ 2: RViz で初期位置を設定

AMCL が自己位置を推定するため、最初に RViz で初期位置を設定する必要があります。

1. `rviz_nav` コンテナの RViz が開いたことを確認
2. ツールバーの **"2D Pose Estimate"** をクリック
3. 地図上のロボットの実際の位置をクリック＆ドラッグして向きを指定
4. AMCL のパーティクル（赤い点群）が収束すれば準備完了

### ステップ 3: API サーバーの死活確認

```bash
curl http://localhost:8000/health
```

```json
{"status": "ok", "ros2_node": true}
```

### ステップ 4: HTTP でロボットを移動させる

```bash
curl -X POST http://localhost:8000/move_to \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 2.0, "yaw": 0.0}'
```

```json
{
  "status": "accepted",
  "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
  "message": "ゴールを受け付けました。ロボットが移動を開始します。"
}
```

RViz でロボットが目標座標に向けて動き始めることを確認してください。

---

## API リファレンス

### GET /health

サーバーと ROS2 ノードの死活確認。

```bash
curl http://localhost:8000/health
```

| フィールド | 説明 |
|-----------|------|
| `status` | `"ok"` 固定 |
| `ros2_node` | ROS2 ノードが初期化済みなら `true` |

---

### POST /move_to

ロボットを指定座標へ移動させる。

**リクエストボディ:**

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `x` | float | ✓ | マップ座標系での X 位置 [m] |
| `y` | float | ✓ | マップ座標系での Y 位置 [m] |
| `yaw` | float | — | 目標姿勢（Z 軸回転）[rad]。省略時 = `0.0` |

**yaw の向き:**

```
  yaw = 0.0      → +X 方向（東）を向く
  yaw = 1.5708   → +Y 方向（北）を向く（π/2 rad）
  yaw = 3.1416   → -X 方向（西）を向く（π rad）
  yaw = -1.5708  → -Y 方向（南）を向く（-π/2 rad）
```

**レスポンス (200):**

```json
{"status": "accepted", "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0}, "message": "..."}
```

**レスポンス (503):**

```json
{"detail": "NavigateToPose アクションサーバーに接続できません。Nav2 の起動を確認してください。"}
```

**使用例:**

```bash
# 原点 (0, 0) へ移動
curl -X POST http://localhost:8000/move_to \
  -H "Content-Type: application/json" \
  -d '{"x": 0.0, "y": 0.0}'

# 北を向いて (−1, 1) へ移動
curl -X POST http://localhost:8000/move_to \
  -H "Content-Type: application/json" \
  -d '{"x": -1.0, "y": 1.0, "yaw": 1.5708}'
```

**Python からのリクエスト例:**

```python
import requests

response = requests.post(
    "http://localhost:8000/move_to",
    json={"x": 1.0, "y": 2.0, "yaw": 0.0},
)
print(response.json())
```

---

## web_bridge_node.py の解説

### なぜ ROS2 ノードと FastAPI を同一プロセスで動かすのか

ROS2 ノードとして動きながら HTTP サーバーも立てる必要があります。
これを実現するためにスレッドを分けています。

```python
# ROS2 エグゼキューターをバックグラウンドスレッドで起動
executor = MultiThreadedExecutor()
executor.add_node(_node)
ros_thread = threading.Thread(target=executor.spin, daemon=True)
ros_thread.start()

# FastAPI / Uvicorn をメインスレッドで起動（ブロッキング）
uvicorn.run(app, host='0.0.0.0', port=8000)
```

| スレッド | 役割 |
|---------|------|
| メインスレッド | Uvicorn が HTTP リクエストを受け付ける |
| バックグラウンドスレッド | ROS2 エグゼキューターがコールバックを処理する |

### なぜ `use_sim_time:=true` が必要か

Nav2 はシミュレーション時刻（`/clock` トピック）を使っています。
`web_bridge_node` が発行する `PoseStamped` のタイムスタンプも同じ時計で発行しないと、
TF の時刻不一致エラーが発生します。

```python
goal.pose.header.stamp = self.get_clock().now().to_msg()  # シミュレーション時刻で発行
```

### なぜ移動完了を待たないのか

HTTP リクエストを移動完了まで待機させると、長い移動中はタイムアウトしてしまいます。
代わりに「ゴールを受け付けた」時点で即座に `accepted` を返し、
ロボットの移動は Nav2 が非同期で実行します。

```
クライアント → POST /move_to
                    ↓ 即座に返す（移動完了は待たない）
クライアント ← {"status": "accepted"}
                    ↓ 非同期で進行
              Nav2 がロボットを目標まで移動させる
```

---

## 停止方法

```bash
# nav プロファイルのサービスを停止
docker compose --profile nav down
```

過去のセッションで起動したコンテナ（オーファンコンテナ）が残っている場合は `--remove-orphans` を付けると全て停止できます:

```bash
docker compose --profile nav down --remove-orphans
```

> **オーファンコンテナとは**
> compose ファイルから削除されたサービスの残骸コンテナです。
> `docker compose up` 時に `Found orphan containers` という警告が出ていたら該当します。

---

## トラブルシューティング

### `503: NavigateToPose アクションサーバーに接続できません`

Nav2 がまだ起動中です。起動から 40 秒以上待ってから再試行してください。

```bash
# Nav2 の起動状態を確認
docker compose logs navigation | tail -20

# bt_navigator が起動していれば Nav2 は準備完了
docker exec navigation bash -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 node list | grep bt_navigator"
```

### ロボットが動かない（API は accepted を返すのに）

AMCL が自己位置推定できていません。RViz の **"2D Pose Estimate"** で初期位置を再設定してください。

### コンテナ名の競合エラー

前回の `docker compose run` で残ったコンテナが存在する場合:

```bash
docker rm -f api_bridge
docker compose --profile nav up
```

### api_bridge のログを確認する

```bash
docker compose logs api_bridge
```

---

## Swagger UI（自動生成 API ドキュメント）

FastAPI はドキュメントを自動生成します。ブラウザで確認できます:

```
http://localhost:8000/docs
```
