#!/usr/bin/env python3
"""
robot_api_client.py

既存 api_bridge (FastAPI, port 8000) の REST クライアント。
GET /health と POST /move_to のみをラップする。
"""

import logging
import requests


logger = logging.getLogger(__name__)


class RobotApiClient:
    """
    api_bridge へのシンプルな HTTP クライアント。
    タイムアウトを設定し、例外を握り潰さずに bool で返す。
    """

    def __init__(self, base_url: str, timeout: float = 5.0):
        """
        Parameters
        ----------
        base_url  : "http://localhost:8000" など末尾スラッシュなし
        timeout   : 各リクエストのタイムアウト [秒]
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout

    def health(self) -> bool:
        """
        GET /health で api_bridge の死活確認。
        {"status": "ok"} が返れば True。
        """
        try:
            resp = requests.get(
                f'{self.base_url}/health',
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            ok = data.get('status') == 'ok'
            if not ok:
                logger.warning(f'health: unexpected response: {data}')
            return ok
        except requests.exceptions.ConnectionError as e:
            logger.error(f'health: connection error: {e}')
            return False
        except requests.exceptions.Timeout:
            logger.error(f'health: timeout ({self.timeout}s)')
            return False
        except Exception as e:
            logger.error(f'health: unexpected error: {e}')
            return False

    def move_to(self, x: float, y: float, yaw: float) -> bool:
        """
        POST /move_to でロボットに移動指令を送る。

        Parameters
        ----------
        x, y : マップ座標系での目標位置 [m]
        yaw  : 目標姿勢 Z 軸回転 [rad]

        Returns
        -------
        True  : ゴール受付成功 ({"status": "accepted"})
        False : 接続失敗 / Nav2 未起動 / 予期しないレスポンス
        """
        try:
            payload = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
            resp = requests.post(
                f'{self.base_url}/move_to',
                json=payload,
                timeout=self.timeout
            )
            resp.raise_for_status()
            data = resp.json()
            accepted = data.get('status') == 'accepted'
            if accepted:
                logger.info(
                    f'move_to accepted: x={x:.3f} y={y:.3f} yaw={yaw:.3f}'
                )
            else:
                logger.warning(f'move_to: unexpected response: {data}')
            return accepted
        except requests.exceptions.HTTPError as e:
            # 503 などのエラーレスポンス (Nav2 未起動など)
            logger.error(f'move_to HTTP error: {e}')
            return False
        except requests.exceptions.ConnectionError as e:
            logger.error(f'move_to: connection error: {e}')
            return False
        except requests.exceptions.Timeout:
            logger.error(f'move_to: timeout ({self.timeout}s)')
            return False
        except Exception as e:
            logger.error(f'move_to: unexpected error: {e}')
            return False
