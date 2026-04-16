#!/usr/bin/env python3
"""
fleet_adapter.py

Open-RMF EasyFullControl fleet adapter for TurtleBot3 (Phase 7)

アーキテクチャ:
  [RMF Core (rmf_traffic_ros2 / rmf_task_ros2)]
         ↕  DDS (CycloneDDS, ROS_DOMAIN_ID=0)
  [fleet_adapter.py]  ←  /amcl_pose  (rclpy)
         ↕  HTTP REST
  [api_bridge:8000  POST /move_to]
         ↕  ROS2 action
  [Nav2 bt_navigator → /cmd_vel → Gazebo]

設計方針:
  - rmf_fleet_adapter_python の EasyFullControl を使用する正式 adapter
  - FleetState / DestinationRequest の ROS msg 直結実装は行わない
  - ロボットへの命令は既存 api_bridge の POST /move_to のみ使用
  - 位置更新は /amcl_pose を rclpy でサブスクライブして RMF に通知
  - EasyFullControl が内部スレッドで RMF executor を spin する
  - rclpy の MultiThreadedExecutor で /amcl_pose をサブスクライブ

未実装 (制約):
  - 停止コマンドの Nav2 への伝達 (stop() は内部フラグのみリセット)
  - バッテリー実測 (固定値 1.0)
  - ドッキング・充電・清掃・搬送タスク
  - 複数ロボット
  - 障害復旧・再計画
"""

import argparse
import logging
import math
import sys
import threading
import time
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseWithCovarianceStamped

# rmf_fleet_adapter_python が提供する rmf_adapter Python モジュール
# パッケージインストール後: source /opt/ros/jazzy/setup.bash で利用可能
import rmf_adapter as adpt
import rmf_adapter.vehicletraits as vehicletraits
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph_lib
import rmf_adapter.easy_full_control as efc

from robot_api_client import RobotApiClient


logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s][%(name)s][%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('rmf_fleet_adapter')


# ─────────────────────────────────────────────────────────────────────────────
# ユーティリティ
# ─────────────────────────────────────────────────────────────────────────────

def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """クォータニオン → yaw 変換 (Z 軸回転のみ)"""
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    )


def load_yaml(path: str) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def build_nav_graph(nav_cfg: dict, map_name: str):
    """
    nav_graph.yaml から rmf_adapter.graph.Graph を構築する。

    Returns
    -------
    (Graph, dict[waypoint_name -> index])
    """
    g = graph_lib.Graph()
    wp_list = nav_cfg['waypoints']
    lane_list = nav_cfg['lanes']

    idx_by_name: dict[str, int] = {}

    for wp in wp_list:
        idx = g.add_waypoint(map_name, [wp['x'], wp['y']])
        wpt = g.get_waypoint(idx)
        if wp.get('is_charger', False):
            wpt.set_charger(True)
        if wp.get('is_holding_point', True):
            wpt.set_holding_point(True)
        idx_by_name[wp['name']] = idx
        logger.info(
            f"  waypoint '{wp['name']}' → graph_idx={idx}"
            f" ({wp['x']:.2f}, {wp['y']:.2f})"
            + (' [charger]' if wp.get('is_charger') else '')
        )

    for lane in lane_list:
        a = idx_by_name[lane['from']]
        b = idx_by_name[lane['to']]
        g.add_lane(a, b)
        if lane.get('bidirectional', True):
            g.add_lane(b, a)
        logger.info(
            f"  lane '{lane['from']}' ↔ '{lane['to']}'"
            if lane.get('bidirectional', True)
            else f"  lane '{lane['from']}' → '{lane['to']}'"
        )

    return g, idx_by_name


# ─────────────────────────────────────────────────────────────────────────────
# ロボットアダプター
# ─────────────────────────────────────────────────────────────────────────────

class RobotAdapter:
    """
    1台の TurtleBot3 の状態と EasyFullControl コールバックを管理する。

    スレッドモデル:
      - _lock で共有状態 (_position, _navigating, _goal, _execution) を保護
      - on_amcl_pose() は rclpy callback スレッドから呼ばれる
      - navigate/stop/dock は RMF の内部スレッドから呼ばれる
      - execution.finished() は both スレッドから呼ばれうる
    """

    def __init__(self, name: str, api: RobotApiClient, arrival_threshold: float = 0.3):
        self.name = name
        self.api = api
        self.arrival_threshold = arrival_threshold
        self._lock = threading.Lock()

        # ロボット状態
        self._position: list | None = None    # [x, y, yaw] map フレーム
        self._battery_soc: float = 1.0        # 固定ダミー値

        # ナビゲーション状態
        self._navigating: bool = False
        self._goal: list | None = None        # [x, y, yaw]
        self._execution = None                # EasyFullControl Execution ハンドル

        # EasyFullControl の Updater (add_robot 後に on_update で設定)
        self._updater = None

    # ── /amcl_pose コールバック ──────────────────────────────────────────────

    def on_amcl_pose(self, x: float, y: float, yaw: float):
        """
        rclpy の /amcl_pose サブスクライバーから呼ばれる。
        1. RMF へ位置更新
        2. 目標到達チェック
        """
        with self._lock:
            self._position = [x, y, yaw]
            updater = self._updater

        # RMF に現在位置を通知
        if updater is not None:
            updater.update_position([x, y, yaw], self._battery_soc)

        # 到達判定
        self._check_arrival(x, y)

    def _check_arrival(self, cx: float, cy: float):
        """目標との距離が閾値以下なら execution.finished() を呼ぶ"""
        with self._lock:
            if not self._navigating or self._goal is None:
                return
            gx, gy = self._goal[0], self._goal[1]
            dist = math.sqrt((cx - gx) ** 2 + (cy - gy) ** 2)
            if dist > self.arrival_threshold:
                return
            # 到達
            logger.info(
                f'[{self.name}] Arrived at ({gx:.2f}, {gy:.2f}), '
                f'dist={dist:.3f}m (threshold={self.arrival_threshold}m)'
            )
            self._navigating = False
            execution = self._execution
            self._execution = None
            self._goal = None

        # ロック外で呼ぶ (RMF ハンドルがロック不要前提)
        if execution is not None:
            execution.finished()

    # ── EasyFullControl コールバック ─────────────────────────────────────────

    def navigate(self, destination, execution):
        """
        RMF から navigation 指令を受け取ったときに呼ばれる。

        Parameters
        ----------
        destination : EasyFullControl.Destination
            .position : [x, y, yaw]
            .name     : waypoint 名 (None のこともある)
        execution   : EasyFullControl.Execution
            .finished()                  → 完了通知
            .update_remaining_time(sec)  → ETA 更新
        """
        pos = destination.position          # [x, y, yaw]
        wp_name = getattr(destination, 'name', '?')
        logger.info(
            f'[{self.name}] navigate → "{wp_name}" '
            f'({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f} rad)'
        )

        with self._lock:
            self._navigating = True
            self._goal = list(pos)
            self._execution = execution
            current = self._position

        # 大まかな ETA を設定
        if current is not None:
            dx = pos[0] - current[0]
            dy = pos[1] - current[1]
            dist = math.sqrt(dx * dx + dy * dy)
            # TurtleBot3 最大速度 0.22 m/s + 余裕
            eta_secs = dist / 0.22 + 5.0
            execution.update_remaining_time(eta_secs)
        else:
            execution.update_remaining_time(30.0)

        # api_bridge へ移動指令
        ok = self.api.move_to(pos[0], pos[1], pos[2])
        if not ok:
            logger.error(
                f'[{self.name}] move_to({pos[0]:.2f}, {pos[1]:.2f}) failed. '
                f'Marking navigation finished (error state).'
            )
            with self._lock:
                self._navigating = False
                self._goal = None
                self._execution = None
            # RMF に完了通知 (失敗だが finished で返す。re-plan は RMF に任せる)
            execution.finished()

    def stop(self, activity):
        """
        RMF からの停止指令。
        注: この実装では Nav2 への停止コマンドは送らない (最小実装)。
            Nav2 は現在のゴールを継続する。RMF 側の内部状態のみリセット。
        """
        logger.info(f'[{self.name}] Stop requested by RMF')
        with self._lock:
            self._navigating = False
            self._goal = None
            self._execution = None

    def dock(self, dock_name: str, execution):
        """
        ドッキング未実装。即座に finished() を呼んで完了扱いにする。
        """
        logger.warning(
            f'[{self.name}] dock("{dock_name}") requested — '
            f'not implemented, finishing immediately'
        )
        execution.finished()

    def action_executor(self, category: str, description, execution):
        """
        カスタムアクション未実装。即座に finished()。
        """
        logger.warning(
            f'[{self.name}] action "{category}" not implemented, finishing immediately'
        )
        execution.finished()

    def on_update(self, updater):
        """
        EasyFullControl が robot を fleet に登録後、updater を渡すコールバック。
        updater.update_position([x, y, yaw], battery_soc) で RMF に位置を通知できる。
        """
        with self._lock:
            self._updater = updater
        logger.info(f'[{self.name}] RobotUpdateHandle (updater) received from EasyFullControl')


# ─────────────────────────────────────────────────────────────────────────────
# エントリーポイント
# ─────────────────────────────────────────────────────────────────────────────

def main(argv=None):
    if argv is None:
        argv = sys.argv

    parser = argparse.ArgumentParser(
        description='TurtleBot3 RMF EasyFullControl Fleet Adapter'
    )
    parser.add_argument('--config',    required=True, help='fleet_config.yaml のパス')
    parser.add_argument('--nav-graph', required=True, help='nav_graph.yaml のパス')
    args, _ = parser.parse_known_args(argv[1:])

    # ── 設定ファイル読み込み ─────────────────────────────────────────────────
    cfg     = load_yaml(args.config)
    nav_cfg = load_yaml(args.nav_graph)

    fleet_name        = cfg['fleet_name']
    robot_name        = cfg['robot_name']
    map_name          = cfg['map_name']
    api_base_url      = cfg['api']['base_url']
    api_timeout       = float(cfg['api'].get('timeout', 5.0))
    initial_wp_name   = cfg.get('initial_waypoint', 'start')
    charger_wp_name   = cfg.get('charger_waypoint', initial_wp_name)
    arrival_threshold = float(cfg.get('arrival_threshold', 0.3))

    logger.info(
        f'Starting RMF fleet adapter: fleet={fleet_name}, robot={robot_name}, '
        f'map={map_name}, api={api_base_url}'
    )

    # ── ROS2 初期化 ───────────────────────────────────────────────────────────
    # rclpy.init() が rcl を初期化し、rmf_fleet_adapter_python (rclcpp) も
    # 同じ rcl コンテキストを共有するため、両者が同一プロセスで共存できる
    rclpy.init(args=argv)

    # ── Nav graph 構築 ────────────────────────────────────────────────────────
    logger.info('Building nav graph...')
    nav_graph, wp_idx = build_nav_graph(nav_cfg, map_name)
    logger.info(
        f'Nav graph: {nav_graph.num_waypoints} waypoints, '
        f'{nav_graph.num_lanes} lanes'
    )

    # ── Vehicle traits ────────────────────────────────────────────────────────
    lim      = cfg['limits']
    # 円形フットプリント (TurtleBot3 burger ≈ 半径 0.18m)
    footprint = geometry.make_final_convex_circle(cfg['profile']['footprint'])
    # vicinity: 衝突検知より広い領域 (デフォルト footprint * 1.5)
    vicinity  = geometry.make_final_convex_circle(
        cfg['profile'].get('vicinity', cfg['profile']['footprint'] * 1.5)
    )
    profile  = geometry.Profile(footprint, vicinity)
    v_traits = vehicletraits.VehicleTraits(
        linear=vehicletraits.Limits(
            lim['linear']['velocity'],
            lim['linear']['acceleration']
        ),
        angular=vehicletraits.Limits(
            lim['angular']['velocity'],
            lim['angular']['acceleration']
        ),
        profile=profile
    )
    v_traits.reversible = False
    logger.info(
        f'VehicleTraits: linear={lim["linear"]["velocity"]}m/s, '
        f'angular={lim["angular"]["velocity"]}rad/s, reversible=False'
    )

    # ── Battery system (最小設定) ─────────────────────────────────────────────
    bs          = cfg['battery_system']
    battery_sys = battery.BatterySystem.make(
        bs['nominal_voltage'],
        bs['nominal_capacity'],
        bs['charging_current']
    )
    motion_sink  = battery.SimpleMotionPowerSink.make(battery_sys, v_traits)
    ambient_sink = battery.SimpleDevicePowerSink.make(battery_sys, 20.0)  # 20W
    tool_sink    = battery.SimpleDevicePowerSink.make(battery_sys, 0.0)

    # ── EasyFullControl 設定 ──────────────────────────────────────────────────
    fleet_config = efc.EasyFullControl.Configuration(
        fleet_name,          # fleet_name
        v_traits,            # vehicle_traits
        nav_graph,           # nav_graph
        battery_sys,         # battery_system
        motion_sink,         # motion_sink
        ambient_sink,        # ambient_sink
        tool_sink,           # tool_sink
        0.1,                 # recharge_threshold (SOC 10% 以下で充電)
        1.0,                 # recharge_soc (100% まで充電)
        False,               # account_for_battery_drain (実測バッテリー不使用)
        {},                  # task_categories
        []                   # action_categories
    )

    # ── EasyFullControl adapter 作成 ──────────────────────────────────────────
    # make() は内部で rclcpp::Node と MultiThreadedExecutor を作り、
    # バックグラウンドスレッドで spin を開始する
    fleet = efc.EasyFullControl.make(fleet_config)
    logger.info(f"EasyFullControl fleet '{fleet_name}' created")

    # ── REST API クライアント ─────────────────────────────────────────────────
    api_client = RobotApiClient(api_base_url, timeout=api_timeout)

    # api_bridge の起動を待つ (最大 90 秒)
    logger.info('Waiting for api_bridge (GET /health)...')
    for attempt in range(30):
        if api_client.health():
            logger.info('api_bridge is healthy')
            break
        logger.info(f'  health check {attempt+1}/30 — waiting 3s')
        time.sleep(3.0)
    else:
        logger.warning(
            'api_bridge health check failed after 30 attempts. '
            'Nav2 may not be ready yet. Continuing anyway.'
        )

    # ── 初期位置の決定 ────────────────────────────────────────────────────────
    wp_list = nav_cfg['waypoints']
    initial_wp = next(
        (w for w in wp_list if w['name'] == initial_wp_name),
        wp_list[0]
    )
    init_pos = [
        float(initial_wp['x']),
        float(initial_wp['y']),
        float(initial_wp.get('yaw', 0.0))
    ]
    logger.info(
        f"Initial position: {init_pos} (waypoint '{initial_wp['name']}')"
    )

    # ── ロボットアダプター作成 ────────────────────────────────────────────────
    robot = RobotAdapter(robot_name, api_client, arrival_threshold)
    robot._position = list(init_pos)

    # ── EasyFullControl に robot を登録 ────────────────────────────────────────
    initial_state = efc.EasyFullControl.RobotState(
        name=robot_name,
        charger=charger_wp_name,
        position=init_pos,
        battery_soc=1.0,
        replan=False
    )

    fleet.add_robot(
        initial_state,          # RobotState
        robot.navigate,         # on_navigate(destination, execution)
        robot.stop,             # on_stop(activity)
        robot.dock,             # on_dock(dock_name, execution)
        robot.action_executor,  # on_action(category, description, execution)
        robot.on_update         # on_update(updater)
    )
    logger.info(f"Robot '{robot_name}' registered in fleet '{fleet_name}'")

    # ── /amcl_pose サブスクライバー ───────────────────────────────────────────
    ros_node = rclpy.create_node('rmf_adapter_pose_listener')

    def amcl_callback(msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
        robot.on_amcl_pose(p.x, p.y, yaw)

    ros_node.create_subscription(
        PoseWithCovarianceStamped,
        '/amcl_pose',
        amcl_callback,
        10
    )
    logger.info('Subscribed to /amcl_pose')
    logger.info('=== Fleet adapter is running. Waiting for RMF tasks and AMCL updates. ===')

    # ── rclpy スピン (メインスレッド) ──────────────────────────────────────────
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(ros_node)
        executor.spin()
    except KeyboardInterrupt:
        logger.info('KeyboardInterrupt — shutting down fleet adapter')
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
