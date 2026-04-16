# rmf_adapter/Dockerfile
#
# Open-RMF EasyFullControl fleet adapter for TurtleBot3 (Phase 7)
#
# ベースイメージ: 既存コンテナと同じ jazzy-desktop-full を使用する
# 理由: ros-jazzy-rmf-* バイナリパッケージが同じ apt リポジトリから取得できること、
#        かつ既存の CycloneDDS 設定と整合が取れることを優先する。
#        adapter 専用に軽量イメージを使うことも可能だが、
#        パッケージバージョン不一致のトラブルを避けるため統一する。
FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# ─── RMF 関連パッケージのインストール ─────────────────────────────────────────
#
# 【重要】パッケージ名について
#
# ros-jazzy-rmf-fleet-adapter-python は 2025 時点で apt から利用可能なはずだが、
# Open-RMF の apt リリースが遅れている場合はソースビルドが必要になる。
#
# apt でインストールできない場合の代替手順 (Dockerfile 末尾のコメントを参照):
#   1. open-rmf/rmf_ros2 を /ws_rmf にクローン
#   2. colcon build --packages-select rmf_fleet_adapter_python
#   3. source /ws_rmf/install/setup.bash を CMD に追加

RUN apt-get update && apt-get install -y --no-install-recommends \
    # RMF fleet adapter C++ コアライブラリ
    ros-jazzy-rmf-fleet-adapter \
    # RMF fleet adapter Python バインディング (rmf_adapter モジュール)
    ros-jazzy-rmf-fleet-adapter-python \
    # RMF traffic (Graph / VehicleTraits / etc.)
    ros-jazzy-rmf-traffic \
    # RMF メッセージ型 (fleet_msgs / task_msgs / traffic_msgs)
    ros-jazzy-rmf-fleet-msgs \
    ros-jazzy-rmf-task \
    ros-jazzy-rmf-task-msgs \
    ros-jazzy-rmf-traffic-msgs \
    # RMF building map (任意: rmf_adapter.graph のロード補助)
    ros-jazzy-rmf-building-map-msgs \
    # CycloneDDS (既存コンテナと RMW 統一)
    ros-jazzy-rmw-cyclonedds-cpp \
    # Python ライブラリ
    python3-requests \
    python3-yaml \
 && rm -rf /var/lib/apt/lists/*

# ─── 環境変数 ─────────────────────────────────────────────────────────────────
ENV TURTLEBOT3_MODEL=burger
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# ─── 作業ディレクトリ ─────────────────────────────────────────────────────────
WORKDIR /rmf_adapter

CMD ["bash"]

# ─── ソースビルド手順 (apt で rmf-fleet-adapter-python が入らない場合) ─────────
#
# 上記 RUN ブロックを以下に差し替えて使用:
#
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     ros-jazzy-rmf-fleet-adapter \
#     ros-jazzy-rmf-traffic \
#     ros-jazzy-rmf-fleet-msgs \
#     ros-jazzy-rmf-task \
#     ros-jazzy-rmf-task-msgs \
#     ros-jazzy-rmf-traffic-msgs \
#     ros-jazzy-rmf-building-map-msgs \
#     ros-jazzy-rmw-cyclonedds-cpp \
#     python3-requests \
#     python3-yaml \
#     python3-pybind11 \
#     git build-essential \
#  && rm -rf /var/lib/apt/lists/*
#
# RUN mkdir -p /ws_rmf/src && cd /ws_rmf/src \
#  && git clone --depth 1 -b jazzy https://github.com/open-rmf/rmf_ros2.git \
#  && cd /ws_rmf \
#  && bash -lc "source /opt/ros/jazzy/setup.bash && \
#               colcon build --packages-select rmf_fleet_adapter_python \
#                            --cmake-args -DCMAKE_BUILD_TYPE=Release"
#
# ENV COLCON_PREFIX_PATH=/ws_rmf/install
# ENV LD_LIBRARY_PATH=/ws_rmf/install/lib:$LD_LIBRARY_PATH
#
# CMD 側に "source /ws_rmf/install/setup.bash &&" を追加すること
