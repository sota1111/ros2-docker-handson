FROM osrf/ros:jazzy-desktop
SHELL ["/bin/bash", "-lc"]

# repo追加に必要な基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
  && rm -rf /var/lib/apt/lists/*

# OSRF Gazebo APT repo (ubuntu-stable)
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      | gpg --dearmor -o /etc/apt/keyrings/gazebo.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# gz実行バイナリ + sim(CLI) + GUIに必要なQt/QML
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-tools2 \
    gz-sim8-cli \
    qml-module-qtquick-controls \
    qml-module-qtquick-controls2 \
    qml-module-qtquick-dialogs \
    qml-module-qtquick-layouts \
    qml-module-qtquick-window2 \
    qml-module-qt-labs-settings \
    qml-module-qt-labs-folderlistmodel \
  && rm -rf /var/lib/apt/lists/*

# execで入ってもROS環境が入る
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /ws
CMD ["bash"]