# ROS公式Dockerイメージ
# jazzy-desktop-full は以下を含む：
# ・ROS2 Jazzy core
# ・RViz
# ・Gazebo (gz-sim)
# ・デバッグツール
# ・GUI系依存
# 学習用途では最もトラブルが少ないベース
FROM osrf/ros:jazzy-desktop-full

# bashをlogin shell化（ROS環境変数展開用）
SHELL ["/bin/bash", "-lc"]

# apt対話抑止
ENV DEBIAN_FRONTEND=noninteractive

# Phase5に必要なパッケージを導入
RUN apt-get update && apt-get install -y --no-install-recommends \
    # TurtleBot3 シミュレーション
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-gazebo \
    # 手動操作
    ros-jazzy-teleop-twist-keyboard \
    # EKF (ローカル自己位置推定)
    ros-jazzy-robot-localization \
    # SLAM (地図生成)
    ros-jazzy-slam-toolbox \
    # AMCL関連 (グローバル自己位置推定 / 保存済み地図での再利用)
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-lifecycle-manager \
    # RVizでparticle・Nav2表示用プラグイン
    ros-jazzy-nav2-rviz-plugins \
    # Nav2 自律移動スタック一式
    # nav2-bt-navigator, nav2-planner, nav2-controller, nav2-smoother,
    # nav2-behaviors, nav2-costmap-2d, nav2-navfn-planner, nav2-smac-planner,
    # nav2-waypoint-follower, nav2-velocity-smoother を含むメタパッケージ
    ros-jazzy-navigation2 \
    # Nav2 デフォルトBTツリーXMLと設定ファイル
    ros-jazzy-nav2-bringup \
    # CycloneDDS (FastDDS の stale shared memory 問題を回避)
    ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*


# TurtleBot3モデル指定（必須）
ENV TURTLEBOT3_MODEL=burger

# Gazeboリソースパス
# TB3モデルをGazeboが見つけるため必要
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models


# 作業ディレクトリ
WORKDIR /ws

# デフォルト起動シェル
CMD ["bash"]