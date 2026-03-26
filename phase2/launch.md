# ROS 2 Launch ファイル解説

## generate_launch_description はどこから呼ばれるか

`ros2 launch` コマンド自体が呼び出します。

```
ros2 launch /ws/launch/ekf.launch.py
      │
      ▼
ros2 launch の内部処理（Python インタープリタ）
  1. 指定された .py ファイルをモジュールとして import
  2. generate_launch_description() を呼び出す
  3. 返ってきた LaunchDescription を解析
  4. 中に含まれる Node 等を順次起動
```

ROS 2 の launch システムが **「`generate_launch_description` という名前の関数を探して呼ぶ」** という規約になっています。
`main()` のような言語仕様ではなく、フレームワーク側の約束事です。

---

## 一行ずつ解説

```python
from launch import LaunchDescription
```
ROS 2 launch フレームワークの基底クラス。「何を起動するか」のリストを束ねるコンテナ。

---

```python
from launch_ros.actions import Node
```
「ROS ノードを1つ起動する」というアクションのクラス。`launch_ros` は ROS 2 専用の拡張ライブラリ。

---

```python
def generate_launch_description():
```
フレームワークが名前で探して呼ぶ、エントリーポイント関数。

---

```python
    return LaunchDescription([
```
起動アクションのリストを `LaunchDescription` に渡して返す。リストなので複数ノードを並べられる。

---

```python
        Node(
            package='robot_localization',
```
`/opt/ros/jazzy/share/robot_localization/` にあるパッケージを指定。

---

```python
            executable='ekf_node',
```
そのパッケージの中の実行ファイル名。`which ekf_node` で場所を確認できる。

---

```python
            name='ekf_filter_node',
```
起動後のノード名（`/ekf_filter_node`）。`ros2 node list` で表示される名前。省略すると executable 名がそのまま使われる。

---

```python
            output='screen',
```
ノードの stdout/stderr をターミナルに流す。省略すると画面に何も表示されない。

---

```python
            parameters=['/ws/config/ekf.yaml']
```
起動時に読み込む YAML パラメータファイル。リストなので複数ファイルを重ねて指定できる。後のファイルが優先される。

---

```python
        )
    ])
```
`Node(...)` と `LaunchDescription([...])` の閉じカッコ。

---

## Node を複数並べる

`LaunchDescription` のリストに複数の `Node` を並べることができます。全ノードが**ほぼ同時に並列起動**します。

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['/ws/config/ekf.yaml']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
        ),
    ])
```

起動順序を制御したい場合は `RegisterEventHandler` を使いますが、
多くの場合はノード側がトピックを待機する設計になっているので順序制御は不要です。

---

## robot_localization パッケージ

`ekf_node` は `robot_localization` パッケージが提供するノードです。

- **リポジトリ:** `ros-planning/robot_localization`
- **インストール:** `apt install ros-jazzy-robot-localization`
- **提供ノード:** `ekf_node`（EKF）、`ukf_node`（UKF）

ROS 本体ではなくコミュニティが管理するパッケージですが、Navigation2（Nav2）スタックでも標準的に使われており、事実上のデファクトスタンダードです。

---

## YAML パラメータファイルの構造

`ekf_node` は起動時に `parameters=['/ws/config/ekf.yaml']` で指定されたファイルを読み込み、その値を使って動作します。

ROS 2 の YAML パラメータファイルの構造は以下の規約になっています：

```yaml
<ノード名>:
  ros__parameters:
    <パラメータ名>: <値>
```

```yaml
ekf_filter_node:        # ← launch.py の name='ekf_filter_node' と一致
  ros__parameters:
    frequency: 30.0
    ...
```

ノード名が一致したブロックだけが読み込まれます。
そのため `name='ekf_filter_node'` を launch.py で変えると、yaml 側も合わせて変える必要があります。

実行中のパラメータは以下で確認できます：

```bash
ros2 param list /ekf_filter_node
```

---

## Node の引数：必須とオプション

`package` と `executable` だけが必須で、あとはオプションです。

### 必須

| 引数 | 理由 |
|------|------|
| `package` | どのパッケージか特定できない |
| `executable` | 何を実行するか特定できない |

### オプションの主な例

```python
Node(
    package='robot_localization',
    executable='ekf_node',

    # ノード名（省略すると executable 名がそのまま使われる）
    name='ekf_filter_node',

    # 出力先（省略するとターミナルに表示されない）
    output='screen',

    # パラメータファイル
    parameters=['/ws/config/ekf.yaml'],

    # トピック名のリマップ（/odom を /my_odom に付け替える）
    remappings=[('/odom', '/my_odom')],

    # ネームスペース（/robot1/ekf_filter_node のようにプレフィックスをつける）
    namespace='robot1',

    # 環境変数の追加
    env={'RCUTILS_LOGGING_MIN_SEVERITY': 'DEBUG'},

    # 別ターミナルで起動（xterm が必要）
    prefix='xterm -e',
)
```

### namespace の活用例

複数ロボットを動かすときは `namespace` がよく使われます：

```python
Node(package='robot_localization', executable='ekf_node', namespace='robot1'),
Node(package='robot_localization', executable='ekf_node', namespace='robot2'),
```

これで `/robot1/odometry/filtered` と `/robot2/odometry/filtered` に分離できます。
