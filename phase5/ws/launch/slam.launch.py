"""
slam.launch.py - SLAM による地図生成スタック起動ファイル

起動するノード:
  ekf_filter_node       : ホイールオドメトリ + IMU を融合して odom→base_footprint TF を発行
  async_slam_toolbox    : LiDARスキャンと odom TF から地図を生成し map→odom TF を発行
  lifecycle_manager_slam: slam_toolbox を configure→activate する Lifecycle マネージャー

TF ツリー (SLAM フェーズ):
  map ──(slam_toolbox)──→ odom ──(EKF)──→ base_footprint ──→ base_scan

AMCL は不要:
  Phase 3/4 では AMCL が map→odom TF を発行していたが、
  SLAM モードでは slam_toolbox 自身が map→odom TF を発行するため不要。

Lifecycle 管理について:
  ROS 2 Jazzy の slam_toolbox は Lifecycle ノードとして実装されている。
  起動しただけでは unconfigured 状態で /scan を購読しない。
  lifecycle_manager が configure→activate を行って初めて動作する。

起動タイミング:
  t =  0s : gazebo コンテナが起動 (このファイルは slam コンテナから実行)
  t =  5s : EKF 起動 → odom→base_footprint TF 確立
  t = 10s : slam_toolbox + lifecycle_manager 起動 → configure→activate 後に地図生成開始
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = {'use_sim_time': True}

    # EKF: ホイールオドメトリ + IMU の融合
    # odom→base_footprint TF を発行する
    # slam_toolbox がこの TF を必要とするため先に起動する
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=['/ws/config/ekf.yaml', use_sim_time],
    )

    # slam_toolbox (online async モード)
    # /scan と odom→base_footprint TF を入力として:
    #   1. スキャンマッチングで現在位置を推定
    #   2. Occupancy Grid (/map) を更新・発行
    #   3. map→odom TF を発行 (AMCL の代わり)
    # ※ Lifecycle ノードのため、lifecycle_manager による activate が必要
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/ws/config/slam.yaml', use_sim_time],
    )

    # Lifecycle マネージャー
    # slam_toolbox (Lifecycle ノード) を自動で configure→activate する
    # autostart: true により起動後すぐにアクティブ化される
    # bond_timeout: 0.0 でボンド（死活監視）を無効化する
    #   デフォルト 4.0s では slam_toolbox がボンドを確立する前に
    #   タイムアウトして強制終了されてしまうため無効化する
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 0.0,
        }],
    )

    return LaunchDescription([
        # EKF を 5 秒後に起動 (Gazebo が /clock を出すのを待つ)
        TimerAction(period=5.0, actions=[ekf_node]),

        # slam_toolbox と lifecycle_manager を EKF より後に起動
        # 理由: slam_toolbox は activate 直後に odom→base_footprint TF を要求する。
        #       EKF が TF を確立する前に起動すると TF タイムアウトエラーが出る。
        TimerAction(period=10.0, actions=[slam_node, lifecycle_manager]),
    ])
