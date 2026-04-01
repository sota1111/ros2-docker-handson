"""
navigation.launch.py - Nav2 自律移動スタック起動ファイル

起動するノード:
  controller_server   : DWBローカルプランナー (速度指令生成)
  smoother_server     : パス平滑化
  planner_server      : NavFn グローバルプランナー (A*)
  behavior_server     : リカバリ行動 (spin / backup / wait など)
  bt_navigator        : ビヘイビアツリーナビゲーター
  waypoint_follower   : 複数ウェイポイント追跡
  velocity_smoother   : 速度指令の加速度制限平滑化
  lifecycle_manager   : 上記ノードのライフサイクル管理

速度トピックのフロー:
  controller_server  ─→ /cmd_vel_nav ─→ velocity_smoother ─→ /cmd_vel ─→ Gazebo
  behavior_server    ─→ /cmd_vel_nav ┘

タイミング:
  localization コンテナの起動シーケンスは:
    t=5s:  EKF (odom→base_footprint TF) 起動
    t=15s: AMCL + map_server 起動
  Nav2 は map→odom→base_footprint の TF チェーンが必要なため、
  TimerAction で 40 秒待機してから起動する。
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # 全 Nav2 ノードに共通するパラメータファイルとオプション
    params_file = '/ws/config/nav2_params.yaml'
    use_sim_time = {'use_sim_time': True}

    # ------------------------------------------------------------------
    # Nav2 ノード群
    # controller / behavior は速度指令を /cmd_vel_nav にリマップし、
    # velocity_smoother が /cmd_vel_nav を受け取って /cmd_vel に変換する
    # ------------------------------------------------------------------
    nav2_nodes = [

        # ローカルプランナー (DWB): グローバルパスに沿った速度指令を生成
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, use_sim_time],
            remappings=[
                # DWB の出力を cmd_vel_nav へ (velocity_smoother が受け取る)
                ('cmd_vel', 'cmd_vel_nav'),
            ],
        ),

        # パス平滑化サーバー
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),

        # グローバルプランナー (NavFn / A*): スタート→ゴールの大局経路を計算
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),

        # リカバリ行動サーバー (spin / backup / wait など)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, use_sim_time],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
            ],
        ),

        # ビヘイビアツリーナビゲーター
        # RViz の "2D Goal Pose" → /goal_pose トピック → このノードが受信
        # BT に従いプランナー・コントローラ・リカバリを組み合わせてナビゲート
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),

        # 複数ウェイポイント追跡
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),

        # 速度スムーザー: 加速度制限を適用して滑らかな /cmd_vel を生成
        # /cmd_vel_nav (controller出力) → /cmd_vel (Gazebo入力)
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, use_sim_time],
            remappings=[
                ('cmd_vel',          'cmd_vel_nav'),  # 入力: controllerからの指令
                ('cmd_vel_smoothed', 'cmd_vel'),       # 出力: Gazeboへの指令
            ],
        ),

        # ライフサイクルマネージャー
        # 上記ノードを順番に configure → activate する
        # autostart: true なので起動後に自動でアクティブ化される
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file, use_sim_time],
        ),
    ]

    return LaunchDescription([
        # 40 秒待機してから Nav2 ノード群を起動
        #
        # 待機が必要な理由:
        # 1. Gazebo が /clock / /scan / /tf を出し揃えるのを待つ
        # 2. localization コンテナの EKF が odom→base_footprint TF を確立 (t≈5s)
        # 3. AMCL が map→odom TF を確立 (t≈15s〜30s)
        # 4. Nav2 のコストマップは起動時に /map トピックと TF チェーンが
        #    必要なため、これらが揃った後でないと configure に失敗する
        TimerAction(
            period=40.0,
            actions=nav2_nodes,
        ),
    ])
