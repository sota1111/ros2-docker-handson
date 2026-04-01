#!/usr/bin/env python3
"""
web_bridge_node.py

外部システムから HTTP 経由でロボットに移動指令を送る REST API ブリッジ。

アーキテクチャ:
  - FastAPI (Uvicorn) がポート 8000 で REST API を提供
  - ROS2 ノードがバックグラウンドスレッドで動作
  - POST /move_to を受け取ったら NavigateToPose アクションゴールを送信し、
    受付完了をレスポンスで返す（移動完了は待たない）

使用方法:
  ros2 run web_bridge web_bridge_node --ros-args -p use_sim_time:=true
"""

import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel


# ──────────────────────────────────────────────────────────
# Pydantic リクエストモデル
# ──────────────────────────────────────────────────────────

class MoveRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0  # ラジアン。デフォルトは正面向き


# ──────────────────────────────────────────────────────────
# ROS2 ノード
# ──────────────────────────────────────────────────────────

class WebBridgeNode(Node):
    """
    NavigateToPose アクションクライアントを持つ ROS2 ノード。
    FastAPI のリクエストハンドラから呼び出される。
    """

    def __init__(self):
        super().__init__('web_bridge_node')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('WebBridgeNode 起動完了 — POST /move_to を待機中')

    def send_goal(self, x: float, y: float, yaw: float) -> bool:
        """
        NavigateToPose ゴールを送信する。
        アクションサーバーが 5 秒以内に応答しない場合は False を返す。
        ゴール送信は非同期（移動完了を待たない）。
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'NavigateToPose アクションサーバーに接続できません。'
                'Nav2 が起動しているか確認してください。'
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # yaw (Z 軸回転) → クォータニオン変換
        # 注意: tf_transformations への依存を避けるため手動計算
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'ゴール送信: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad'
        )

        # 非同期送信（完了を待たずに即座に返る）
        self._client.send_goal_async(goal)
        return True


# ──────────────────────────────────────────────────────────
# グローバルノード参照（FastAPI ハンドラから参照）
# ──────────────────────────────────────────────────────────

_node: WebBridgeNode = None


# ──────────────────────────────────────────────────────────
# FastAPI アプリケーション
# ──────────────────────────────────────────────────────────

app = FastAPI(
    title='ROS2 REST Bridge',
    description='HTTP 経由で Nav2 の NavigateToPose アクションを呼び出すブリッジ',
    version='1.0.0',
)


@app.get('/health')
async def health():
    """死活確認エンドポイント"""
    return {'status': 'ok', 'ros2_node': _node is not None}


@app.post('/move_to')
async def move_to(req: MoveRequest):
    """
    ロボットを指定座標へ移動させる。

    - **x**: マップ座標系での X 位置 [m]
    - **y**: マップ座標系での Y 位置 [m]
    - **yaw**: 目標姿勢（Z 軸回転）[rad]、デフォルト 0.0

    レスポンス: ゴール受付完了（移動完了を保証しない）
    """
    if _node is None:
        raise HTTPException(status_code=503, detail='ROS2 ノードが初期化されていません')

    ok = _node.send_goal(req.x, req.y, req.yaw)
    if not ok:
        raise HTTPException(
            status_code=503,
            detail='NavigateToPose アクションサーバーに接続できません。Nav2 の起動を確認してください。',
        )

    return {
        'status': 'accepted',
        'goal': {'x': req.x, 'y': req.y, 'yaw': req.yaw},
        'message': 'ゴールを受け付けました。ロボットが移動を開始します。',
    }


# ──────────────────────────────────────────────────────────
# エントリーポイント
# ──────────────────────────────────────────────────────────

def main():
    global _node

    # ROS2 初期化
    rclpy.init()
    _node = WebBridgeNode()

    # ROS2 エグゼキューターをバックグラウンドスレッドで起動
    # MultiThreadedExecutor でアクションコールバックを並行処理
    executor = MultiThreadedExecutor()
    executor.add_node(_node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # FastAPI / Uvicorn をメインスレッドで起動（ブロッキング）
    # network_mode: host のため 0.0.0.0 でバインドすればホストからアクセス可能
    uvicorn.run(app, host='0.0.0.0', port=8000, log_level='info')

    # Uvicorn 終了後のクリーンアップ
    executor.shutdown()
    _node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
