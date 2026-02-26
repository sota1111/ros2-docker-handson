FROM osrf/ros:jazzy-desktop-full
# jazzy-desktop ＋ 以下が追加される。

# 追加されるもの（重要）
# ・Gazebo Sim（gz-sim, gz-gui）
# ・Gazebo classic 互換周辺
# ・デモ用ワールド・リソース
# ・シミュレーション系の依存一式
# ・チュートリアル向けパッケージ

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# Phase 1: TurtleBot3 + Gazebo + RViz + Teleop（最小）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*

# TurtleBot3 設定（Dockerfileでは固定値のみ）
ENV TURTLEBOT3_MODEL=burger
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models

# ROS 環境を常時有効化（既存値があれば bash 側でマージ）
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc \
 && echo 'export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH}' >> /root/.bashrc

WORKDIR /ws
CMD ["bash"]